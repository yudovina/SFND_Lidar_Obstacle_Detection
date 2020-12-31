/* \author Elena Yudovina */
// KD tree implementation
// Borrows heavily from the quiz

#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>

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

class KdTree
{
public:
	Node* root;

	// constructor
	KdTree()
		: root(NULL)
	{}

	// insert labelled point into the tree
	void insert(std::vector<float> point, int id);

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol);
};

#endif /* KDTREE_H_ */