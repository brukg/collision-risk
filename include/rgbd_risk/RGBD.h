
#ifndef RGBD_ICP_H
#define RGBD_ICP_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //

class RGBD
{
    public:
        RGBD(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~RGBD() {};

    private:
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); //point cloud callback
        // void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); //imu data callback
        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); //imu data callback
        bool getCamera2BaseTransform(); //tf frame transformation
        
        
        void cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //crops cloud using box filter
        void removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //removes noise using Statistical outlier removal
        void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //downsampling the point cloud using Voxelgrid
        void removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr); //ground removal using RANSAC
        void filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //filtering the point cloud
        void clusterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > &clustered_clouds_ptr); //clustering the point cloud
        void correspondenceCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_cloud_ptr1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_cloud_ptr2); //correspondence cloud
        
        std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud); //visualize the point cloud
        inline float convertColor(float colorIn); // 
        
        int dist(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pt, const int i); //distance between robot and point cloud
        
        ros::Subscriber pc_sub; //point cloud subscriber
        ros::Subscriber imu_sub; //IMU data subscriber
        ros::Subscriber pose_sub; //IMU data subscriber
        ros::Publisher pose_pub; //publishes geometry_msgs::PoseWithCovariance msgs
        ros::Publisher odom_pub; //publishes nav_msgs::Odometry msgs
        ros::Publisher pc_pub;  //publishes sensor_msgs::PointCloud2
        ros::Publisher signal_pub;
        ros::Publisher vis_pub; //publishes force direction
        ros::Publisher vis_pub_centroid;
        
        pcl::PointCloud<pcl::PointXYZ> _prev_cloud; //point cloud at the previous time instance
        pcl::PointCloud<pcl::PointXYZRGBA> _prev_rgba_cloud; //point cloud at the previous time instance
        std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > _prev_clustered_clouds;

        Eigen::Matrix4f prev_transformation; //cumulative transformation until the previous time instance
        bool is_initial, is_imu_start, is_pose_start; //boolean to tell if this is 1st iteration of the algo and start of imu and pose reading
        double _prev_acc, _curr_acc; //accleration in consecutive time stamps
        Eigen::Quaternionf  _prev_quat; 
        Eigen::Vector3f     _prev_trans; 
        double _prev_imu_time, _curr_imu_time; //consecutive time stamps of imu data 
        double _prev_time_stamp; //time stamp of the previous point cloud



        /*---------sub-pub parameters----------*/
        std::string _point_cloud_sub_topic; //point cloud ros topic to subscribe to
        std::string _imu_topic; //imu ros topic to subscribe to
        std::string _pose_sub_topic; //pose ros topic to which to subscribe
        std::string _pose_pub_topic; //pose ros topic to which to publish
        std::string _odom_pub_topic; //odom ros topic to which to publish
        std::string _point_cloud_pub_topic;
        std::string _stop_signal_topic;
        
        /*----------ICP parameters------------*/
        double _leaf_size; //leaf size for voxel grid
        double _minX, _maxX, _minY, _maxY, _minZ, _maxZ; //min and max pts for box filter
        int _mean_k; //number of neighbors to analyze for each point for noise removal
        double _std_mul; //standard deviation multiplication threshold for noise removal

        double _dist_threshold; //distance threshold for RASNSAC to consider a point as inlier (for ground removal)
        int _eps_angle; //allowed difference of angles in degrees for perpendicular plane model

        double _time_between_cloud_points; // time between two cloud points to do ICP
        double _transformation_epsilon; //minimum transformation difference for ICP termination condition
        int _max_iters; //max number of registration iterations
        double _euclidean_fitness_epsilon; //maximum allowed Euclidean error between two consecutive steps in the ICP loop
        double _max_correspondence_distance; //correspondences with higher distances will be ignored
        double _speed; //speed for initial guess
        double _yaw_rate; //change in yaw for initial guess

        int _max_cluster_size, _min_cluster_size; //max and min cluster size for clustering
        double _cluster_tolerance; //cluster tolerance for clustering
        /* frames*/
        std::string _frame_id, _base_frame_id, _camera_frame_id; //frame ids
        tf2::Transform mCamera2BaseTransf;    // Coordinates of the base frame in camera frame
        bool mCamera2BaseTransfValid = false;

        // initialization Transform listener
        boost::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> mTfListener;
          
        tf2::Transform robot_pose;//robot pose from SLAM
        geometry_msgs::TransformStamped pose_msg;

        Eigen::Vector3d robot_pose_1, robot_pose_2, robot_pose_pred; // previous robot pose

};

#endif