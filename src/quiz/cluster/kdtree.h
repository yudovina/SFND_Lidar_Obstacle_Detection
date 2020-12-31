/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
		: root(NULL)
	{}

	void insertHelper(Node*& startNode, std::vector<float> point, int id, unsigned int depth)
	{
		unsigned int dir = depth % 2;
		if (startNode == NULL)
		{
			startNode = new Node(point, id);
		}
		else if (point[dir] < startNode->point[dir])
		{
			insertHelper(startNode->left, point, id, depth + 1);
		}
		else
		{
			insertHelper(startNode->right, point, id, depth + 1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(root, point, id, 0);
	}

	void searchHelper(std::vector<float> target, float distanceTol, Node* startNode, unsigned int depth, std::vector<int>& ids)
	{
		if (startNode == NULL)
			return;

		if (fabs(target[0] - startNode->point[0]) <= distanceTol &&
			fabs(target[1] - startNode->point[1]) <= distanceTol)
		{
			float distance = sqrt((target[0] - startNode->point[0]) * (target[0] - startNode->point[0]) +
				(target[1] - startNode->point[1]) * (target[1] - startNode->point[1]));
			if (distance <= distanceTol)
			{
				ids.push_back(startNode->id);
			}
		}

		unsigned int dir = depth % 2;
		if ((startNode->point[dir] - target[dir]) >= -distanceTol)
			searchHelper(target, distanceTol, startNode->left, depth + 1, ids);
		if ((startNode->point[dir] - target[dir]) <= distanceTol)
			searchHelper(target, distanceTol, startNode->right, depth + 1, ids);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, root, 0, ids);
		return ids;
	}
};




