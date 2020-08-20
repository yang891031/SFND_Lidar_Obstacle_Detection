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
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point:indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for(int i : inliers->indices){
        planeCloud->points.push_back(cloud->points[i]);
    }
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients) ;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    inliers = RansacPlane(cloud,maxIterations,distanceThreshold);

    if(inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    pcl::PointIndices::Ptr IndicesResult(new pcl::PointIndices);
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    // For max iterations
    while(maxIterations--){

        std::unordered_set<int> inliers;
        // Randomly sample subset and fit line
        while(inliers.size()<3)
            inliers.insert(rand()%(cloud->points.size()));
        float x1,y1,z1,x2,y2,z2,x3,y3,z3;
        auto itr = inliers.begin();

        int idx1 = *itr;
        itr++;
        int idx2 = *itr;
        itr++;
        int idx3 = *itr;

        x1 = cloud->points[idx1].x;
        y1 = cloud->points[idx1].y;
        z1 = cloud->points[idx1].z;
        x2 = cloud->points[idx2].x;
        y2 = cloud->points[idx2].y;
        z2 = cloud->points[idx2].z;
        x3 = cloud->points[idx3].x;
        y3 = cloud->points[idx3].y;
        z3 = cloud->points[idx3].z;

        float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        float d = -(a*x1+b*y1+c*z1);

        // Measure distance between every point and fitted line
        for(int i = 0;i<cloud->points.size();i++){
            if(inliers.count(i)>0){
                continue;
            }
            float x = cloud->points[i].x;
            float y = cloud->points[i].y;
            float z = cloud->points[i].z;
            float dist = fabs(a*x + b*y + c*z + d)/sqrt(a*a+b*b+c*c);

            // If distance is smaller than threshold count it as inlier
            if(dist<distanceTol){
                inliers.insert(i);
            }
        }

        if(inliers.size()>inliersResult.size()){
            inliersResult = inliers;
        }

    }
    // Return indicies of inliers from fitted line with most inliers
    for(int i = 0;i<cloud->points.size();i++){
        if(inliersResult.count(i)>0){
            IndicesResult->indices.push_back(i);
        }
    }

    return IndicesResult;

}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice,typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool>& processed,KdTree* tree,float distanceTol ){
    processed[indice] = true;
    cluster.push_back(indice);
    std::vector<float> point;
    point.push_back(cloud->points[indice].x);
    point.push_back(cloud->points[indice].y);
    point.push_back(cloud->points[indice].z);
    std::vector<int> nearest = tree->search(point,distanceTol);
    for(int id:nearest){
        if(processed[id]){
            continue;
        }
        clusterHelper(id,cloud,cluster,processed,tree,distanceTol);

    }
}

template<typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<pcl::PointIndices> clusters;
    std::vector<bool> processed_table(cloud->points.size(),false);

    for(int i =0;i<cloud->points.size();i++){
        if(processed_table[i]){
            continue;
        }
        std::vector<int> cluster;

        clusterHelper(i,cloud,cluster,processed_table,tree,distanceTol);

        pcl::PointIndices clusterIndices;
        for(int i:cluster){
            clusterIndices.indices.push_back(i);
        }
        clusters.push_back(clusterIndices);

    }

    return clusters;

}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
//    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    tree->setInputCloud (cloud);
//
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<PointT> ec;
//    ec.setClusterTolerance (clusterTolerance); // 2cm
//    ec.setMinClusterSize (minSize);
//    ec.setMaxClusterSize (maxSize);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (cloud);
//    ec.extract (cluster_indices);
    KdTree* tree = new KdTree;

    for (int i=0; i<cloud->points.size(); i++){
        std::vector<float> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        point.push_back(cloud->points[i].z);
        tree->insert(point,i);
    }
    std::vector<pcl::PointIndices> cluster_indices = euclideanCluster(cloud,tree,clusterTolerance);



    for(pcl::PointIndices getIndices:cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int i:getIndices.indices){
            cloudCluster->points.push_back(cloud->points[i]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
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


