#ifndef KD_TREE
#define KD_TREE

/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node **node, int depth, pcl::PointXYZI point, int id){
		if(*node == NULL){
			*node = new Node(point, id); 
		}
		else{
			float comp1;
			float comp2;
			
			int cd = depth % 3;
			if(cd == 0){
				comp1 = point.x;
				comp2 = (*node)->point.x;
			}
			else if(cd == 1){
				comp1 = point.y;
				comp2 = (*node)->point.y;
			}
			else if(cd == 2){
				comp1 = point.z;
				comp2 = (*node)->point.z;
			}
			else{
				std::cout << "wtf" << cd << std::endl;
			}
			if(comp2 < comp2){
				insertHelper(&(*node)->left, depth+1, point, id);
			}
			else{
				insertHelper(&(*node)->right, depth+1, point, id);
			}
		}
	}
	void insert(pcl::PointXYZI point, int id)
	{
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids){
		if (node != NULL){
			// std::cout<< "not null: node " << node->point.x << "  " << target.x  << "  " << node->point.y << "  " << target.y<< std::endl;;
			if((node->point.x >= (target.x - distanceTol) && 
			    node->point.x <= (target.x + distanceTol)) &&
			   (node->point.y >= (target.y - distanceTol) && 
			    node->point.y <= (target.y + distanceTol)))// &&
			//    (node->point.z >= (target.z - distanceTol) && 
			//     node->point.z <= (target.z + distanceTol)))
			{
				float distance = sqrt((node->point.x - target.x)*(node->point.x - target.x) + 
				   				 	  (node->point.y - target.y)*(node->point.y - target.y)); //+
									//   (node->point.z - target.z)*(node->point.z - target.z));
					std::cout<< "distance = " << distance << "points: " << node->point.x << "  " 
					         << target.x  << "  " << node->point.y << "  " << target.y  << std::endl;
				if(distance <= distanceTol){
					ids.push_back(node->id);
					// std::cout << "search size = " << ids.size() <<std::endl;
				}
			}
			float comp1;
			float comp2;
			
			int cd = depth % 3;
			if(cd == 0){
				comp1 = target.x;
				comp2 = node->point.x;
			}
			else if(cd == 1){
				comp1 = target.y;
				comp2 = node->point.y;
			}
			else if(cd == 2){
				comp1 = target.z;
				comp2 = node->point.z;
			}
			else{
				std::cout << "wtf  searchHelper -> " << cd <<  std::endl;
			}
			if ((comp1 - distanceTol) < comp2 ){
				searchHelper(target, node->left, (depth + 1), distanceTol, ids);
				// std::cout << "left" << std::endl;
			}
			if ((comp1 + distanceTol) > comp2 ){
				searchHelper(target, node->right, (depth + 1), distanceTol, ids);
				// std::cout << "right" << std::endl;
			}			

		}

	}



	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		std::cout << "searchHelper search end size +++++ " << ids.size() << std::endl;
		return ids;
	}
	

};




#endif