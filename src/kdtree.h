/* \author Aaron Brown */
// Quiz on implementing kd tree



// Structure to represent node of kd tree
template<typename PointT>
class Node
{
public:
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
class KdTree
{
public:

	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT>** node, int depth, PointT point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node<PointT>(point,id);
		}
		else
		{
			int cd = depth % 2;
			float p,p_in;

			if (cd == 0)
			{
				p = (*node)->point.x;
				p_in = point.x;
			}
			else
			{
				p = (*node)->point.y;
				p_in = point.y;
			}

			if(p_in < p)
			{
				insertHelper(&((*node)->left),depth+1,point,id);
			}
			else
			{
				insertHelper(&((*node)->right),depth+1,point,id);
			}
		}
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		insertHelper(&root,0,point,id);

	}

	void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if(node->point.x >= (target.x - distanceTol) &&
			   node->point.x <= (target.x + distanceTol) &&
			   node->point.y >= (target.y - distanceTol) &&
			   node->point.y <= (target.y + distanceTol))
			{
				float distance = sqrt( ((node->point.x-target.x) * (node->point.x-target.x)) +  ((node->point.y-target.y) * (node->point.y-target.y)));
				if(distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			//check accross boundary

			int cd = depth % 2;
			float p,p_trgt;

			if (cd == 0)
			{
				p = node->point.x;
				p_trgt = target.x;
			}
			else
			{
				p = node->point.y;
				p_trgt = target.y;
			}

			if((p_trgt - distanceTol) < p)
			{
				searchHelper(target, node->left, depth+1 , distanceTol, ids);
			}
			if((p_trgt + distanceTol) > p)
			{
				searchHelper(target, node->right, depth+1 , distanceTol, ids);
			}

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0 , distanceTol, ids);

		return ids;
	}
	

};



