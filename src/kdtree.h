
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		insertHelper(root,0,point,id);
		// the function should create a new node and place correctly with in the root 

	}
    void insertHelper(Node* &node,uint depth,std::vector<float> point,int id){
	    if(node == NULL){
	        node = new Node(point,id);
	    }else{
	        uint cd = depth % 3;

	        if(point[cd]<node->point[cd]){
	            insertHelper(node->left,depth+1,point,id);
	        }else{
                insertHelper(node->right,depth+1,point,id);
	        }
	    }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}

	void searchHelper(std::vector<float> target,Node* node,int depth, float distanceTol, std::vector<int> &ids){
	    if(node!=NULL){
	        if(node->point[0]>=(target[0]-distanceTol)
	        &&node->point[0]<=(target[0]+distanceTol)
	        &&node->point[1]>=(target[1]-distanceTol)
	        &&node->point[1]<=(target[1]+distanceTol)
	        &&node->point[2]>=(target[2]-distanceTol)
	        &&node->point[2]<=(target[2]+distanceTol)){
	            float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1])+(node->point[2]-target[2])*(node->point[2]-target[2]));
	            if(distance<=distanceTol){
                    ids.push_back(node->id);
	            }
	        }
	        int cd = depth%3;
	        if((target[cd]-distanceTol)<node->point[cd]){
                searchHelper(target,node->left,depth+1,distanceTol,ids);
	        }
            if((target[cd]+distanceTol)>node->point[cd]){
                searchHelper(target,node->right,depth+1,distanceTol,ids);
            }
	    }
	}

	

};




