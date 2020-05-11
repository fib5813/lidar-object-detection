

class Cluster{
    public:
    void setClusterParams(float tolerance, int minSize, int maxSize){
        kTolerance = tolerance;
        kMinSize = minSize;
        kMaxSize = maxSize;
    }

    void KdTree();

    private:
    float kTolerance = 1;
    int kMinSize = 3;
    int kMaxSize = 50;

};