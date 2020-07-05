// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f ( 2.6, 1.7,-0.4,1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    {
    	inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr objectCloud(new pcl::PointCloud<PointT>());

	for(int index : inliers->indices)
	{
		planeCloud->points.push_back(cloud->points[index]);
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*objectCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, objectCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
	std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);

    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(KdTree<PointT>* tree, typename pcl::PointCloud<PointT>::Ptr& cloud, std::vector<int>& cluster, int index, std::vector<bool>& processed, float distanceTol)
{

	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearbyPoints = tree->search(cloud->points[index],distanceTol);

	for(int id : nearbyPoints)
	{

		if(processed[id] == false)
		{
			Proximity(tree,cloud,cluster,id,processed,distanceTol);
		}
	}

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	KdTree<PointT>* tree = new KdTree<PointT>();

    for (int i=0; i<cloud->points.size(); i++)
    	tree->insert(cloud->points[i],i);

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	std::vector<bool> processed(cloud->points.size(), false);

	int i = 0;
	while(i < cloud->points.size())
	{
		std::vector<int> cluster_ids;
		if(processed[i] == false)
		{
			typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());

			Proximity(tree,cloud,cluster_ids,i,processed,clusterTolerance);
			if(cluster_ids.size() >= minSize && cluster_ids.size() <= maxSize)
			{
	            for (int i = 0; i < cluster_ids.size(); ++i)
	            {
	                cluster->points.push_back(cloud->points[cluster_ids[i]]);
	            }

	            clusters.push_back(cluster);
			}

		}

        else
        {
          for (int i = 0; i < cluster_ids.size(); ++i)
          {
        	  processed[cluster_ids[i]] = false;
          }
        }

		i++;
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
int ProcessPointClouds<PointT>::calcRandom(int min, int max)
{
   return min + std::rand() % (max+1 - min);
}

template<typename PointT>
std::unordered_set<int>  ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	for(int itr = 0; itr < maxIterations; itr++)
	{
		std::unordered_set<int> inliersTemp;
		PointT p1;
		PointT p2;
		PointT p3;
		int randId;
		double A,B,C,D;

		// Randomly sample subset and fit line
		randId = calcRandom(0,cloud->size());
		p1 = cloud->points[randId];

		randId = calcRandom(0,cloud->size());
		p2 = cloud->points[randId];

		randId = calcRandom(0,cloud->size());
		p3 = cloud->points[randId];

		A = ((p2.y - p1.y) * (p3.z - p1.z)) - ((p2.z - p1.z)*(p3.y-p1.y));
		B = ((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x)*(p3.z-p1.z));
		C = ((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y)*(p3.x-p1.x));
		D = -((A * p1.x) + (B * p1.y) + (C * p1.z));

		for(int index = 0; index < cloud->size(); index++)
		{
			if(inliersTemp.count(index)>0)
				continue;
			PointT current_p;
			double distance = 0.0;

			current_p = cloud->points[index];

			// Measure distance between every point and fitted line
			distance = fabs((A * current_p.x) + (B * current_p.y) + (C * current_p.z) + D)/sqrt((A*A) + (B*B) + (C*C));

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol)
			{
				inliersTemp.insert(index);
			}
		}

		if( inliersTemp.size() >  inliersResult.size())
		{
			inliersResult = inliersTemp;
		}
	}

	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
