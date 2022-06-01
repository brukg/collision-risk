#include <iostream>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

/* 
* @brief print 4x4 matrix 
This function takes the reference of a 4x4 matrix and prints the rigid transformation in an human readable way.

**/
void print4x4Matrix (const Eigen::Matrix4f &matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


/* 
* @brief Cropping the cloud using Box filter 
**/
void RGBD::cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(_minX, _minY, _minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(_maxX, _maxY, _maxZ, 1.0));
    boxFilter.setInputCloud(in_cloud_ptr);
    boxFilter.filter(*out_cloud_ptr);


    return;
}

/* 
* @brief Removing Noise using Statistical outlier method 
**/
void RGBD::removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setMeanK(_mean_k);
    sor.setStddevMulThresh(_std_mul);
    sor.filter(*out_cloud_ptr);

    return;
}

/* 
* @brief Downsampling using Aprroximate Voxel grid filter 
**/
void RGBD::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_vg;
    approx_vg.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    approx_vg.setInputCloud(in_cloud_ptr);
    approx_vg.filter(*out_cloud_ptr);

    return;
}

/* 
* @brief Removes ground plane using perpendicular plane model with RANSAC 
**/
void RGBD::removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr)
{
   
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //Creating the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg; // Create the segmentation object 
    seg.setOptimizeCoefficients(true); //
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // 
    seg.setMethodType(pcl::SAC_RANSAC); //
    seg.setDistanceThreshold(_dist_threshold);
    seg.setAxis(Eigen::Vector3f(0,0,1)); //z-axis
    seg.setEpsAngle(_eps_angle);
    seg.setInputCloud(in_cloud_ptr);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout<<"Could not estimate the plane"<<endl;
    }

    //Remove ground from the cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true); //true removes the indices
    extract.filter(*out_cloud_ptr);

    //Extract ground from the cloud
    extract.setNegative(false); //false leaves only the indices
    extract.filter(*ground_plane_ptr);

    return;
}

/* 
* @brief Filters the point cloud using cropping, ground and noise removal filters and then downsamples 
**/
void RGBD::filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr only_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_noise_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);


    cropCloud(in_cloud_ptr, cropped_cloud_ptr);
    removeGround(cropped_cloud_ptr, no_ground_cloud_ptr, only_ground_cloud_ptr);
    removeNoise(no_ground_cloud_ptr, no_noise_cloud_ptr);
    downsampleCloud(cropped_cloud_ptr, out_cloud_ptr);

    return;
}




/*
* @brief cluster point cloud using euclidean cluster extraction and push to vector
*/
void RGBD::clusterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > &clustered_clouds_ptr)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in_cloud_ptr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(_cluster_tolerance);
    ec.setMinClusterSize(_min_cluster_size);
    ec.setMaxClusterSize(_max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_cloud_ptr);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator idx = it->indices.begin(); idx != it->indices.end(); ++idx)
        {
            cloud_cluster->points.push_back(in_cloud_ptr->points[*idx]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }
        clustered_clouds_ptr.push_back(cloud_cluster);
    }
    return;
}

/*
 *  @brief This function creates a PCL visualizer
 **/
shared_ptr<pcl::visualization::PCLVisualizer> RGBD::createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(0.50);
    // viewer->addCoordinateSystem(1.0, 0.0, 0.0 , -M_PI / 2);
    viewer->initCameraParameters();
    // viewer.setCameraPosition(0, 0, 0,    0, 0, 0,   0, 0, 1);

    return (viewer);
}



/**
 *  @brief This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float RGBD::convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}