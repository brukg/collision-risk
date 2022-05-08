#include "rgbd_risk/RGBD.h"
#include "rgbd_risk/utils.h"
// #include "rgbd_risk/correspondence.h"
#include <iostream>
#include <chrono>
#include <mutex>
#include <math.h> 

#include <geometry_msgs/PoseWithCovarianceStamped.h> //rose pose msg with covariance
#include <geometry_msgs/PoseStamped.h> //ros pose msg
#include <nav_msgs/Odometry.h> //ros odometry msg

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h> //conversion between tf and eigen





// #include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h> //generalized iterative closest point algorithm from pcl
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// risk visualizer libraries
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>

// ZED includes
#include <sl/Camera.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace sl;

Mat data_cloud;
std::mutex mutex_input;
sl::Resolution cloud_res;

/* @brief Constructor */
RGBD::RGBD(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    /* Loading parameters  */

    // filterin params from yaml file
    private_nh.param("leaf_size", _leaf_size, 0.1);
    private_nh.param("dist_threshold", _dist_threshold, 0.1);
    private_nh.param("eps_angle", _eps_angle, 15);
    private_nh.param("minX", _minX, 0.0);
    private_nh.param("minY", _minY, -25.0);
    private_nh.param("minZ", _minZ, -3.0);
    private_nh.param("maxX", _maxX, 40.0);
    private_nh.param("maxY", _maxY, 25.0);
    private_nh.param("maxZ", _maxZ, 3.0);
    private_nh.param("mean_k", _mean_k, 50);
    private_nh.param("std_mul", _std_mul, 1.0);

    // ICP params from yaml file
    private_nh.param("time_between_cloud_points", _time_between_cloud_points, 0.1);
    private_nh.param("transformation_epsilon", _transformation_epsilon, 0.01);
    private_nh.param("max_iterations", _max_iters, 75);
    private_nh.param("euclidean_fitness_epsilon", _euclidean_fitness_epsilon, 0.1);
    private_nh.param("max_correspondence_distance", _max_correspondence_distance, 1.0);

    // clustering params from yaml file
    private_nh.param("cluster_tolerance", _cluster_tolerance, 0.1);
    private_nh.param("min_cluster_size", _min_cluster_size, 100);
    private_nh.param("max_cluster_size", _max_cluster_size, 10000);

    // publishers params
    private_nh.param<std::string>("imu_topic", _imu_topic, "imu/data");
    private_nh.param<std::string>("pose_pub_topic", _pose_pub_topic, "rgbd_pose");
    private_nh.param<std::string>("odom_pub_topic", _odom_pub_topic, "rgbd_pose");
    private_nh.param<std::string>("point_cloud_pub_topic", _point_cloud_pub_topic, "rgbd_pose");
    
    // subscribers params
    private_nh.param<std::string>("pose_sub_topic", _pose_sub_topic, "");
    private_nh.param<std::string>("point_cloud_sub_topic", _point_cloud_sub_topic, "");
    

    // frames
    private_nh.param<std::string>("frame_id", _frame_id, "odom");
    private_nh.param<std::string>("camera_frame_id", _camera_frame_id, "realsense_link");
    private_nh.param<std::string>("base_frame_id", _base_frame_id, "base_link");
    


    // subscribers
    pc_sub = node.subscribe(_point_cloud_sub_topic, 1, &RGBD::cloudCallback, this);
    // imu_sub = node.subscribe(_imu_topic, 1, &RGBD::imuCallback, this);
    pose_sub = node.subscribe(_pose_sub_topic, 1, &RGBD::poseCallback, this);
    
    // publishers
    pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>(_pose_pub_topic, 1);
    odom_pub = node.advertise<nav_msgs::Odometry>(_odom_pub_topic, 1);
    pc_pub = node.advertise<sensor_msgs::PointCloud2>(_point_cloud_pub_topic, 5);

    //initialising values
    _prev_acc = 0.0;
    _curr_acc = 0.0;
    _yaw_rate = 0.0;
    _speed = 0.0;
    // _prev_quat = Eigen::Quaternion<float>::Zero();
    // _prev_trans = Eigen::Matrix<float>::Zero();

    is_initial = true;
    is_imu_start = true;
    is_pose_start = true;

    // Initialization transformation listener
    mTfBuffer.reset(new tf2_ros::Buffer);
    mTfListener.reset(new tf2_ros::TransformListener(*mTfBuffer));
    robot_pose.setIdentity();//initialize robot pose
        // Eigen::Vector3d robot_pose_1(3);
}


/**
 * @brief eucleadian distance
 * 
 */
// int RGBD::dist (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pt, const int i){
// 	cout;
// 	int D = hypot(abs(*pt->points[i].x), abs(*pt->points[i].y), abs(*pt->points[i].z));
// 	return D;
// }

/*
* tf transformation between camera frame and base frame
*/
bool RGBD::getCamera2BaseTransform()
{
    ROS_INFO("Getting static TF from '%s' to '%s'", _camera_frame_id.c_str(), _base_frame_id.c_str());

    mCamera2BaseTransfValid = false;
    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Base link
    try {
        // Save the transformation
        ROS_INFO("Getting static TF from '%s' to '%s'", _camera_frame_id.c_str(), _base_frame_id.c_str());
        geometry_msgs::TransformStamped c2b = mTfBuffer->lookupTransform(_base_frame_id, _camera_frame_id, ros::Time(0), ros::Duration(0.1));

        // Get the TF2 transformation
        tf2::fromMsg(c2b.transform, mCamera2BaseTransf);

        double roll, pitch, yaw;
        tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);
        ROS_INFO("%.3f", roll);
    } catch (tf2::TransformException& ex) {
        ROS_INFO("%s", ex.what());
        if (!first_error) {
            first_error = false;
        }

        mCamera2BaseTransf.setIdentity();
        return false;
    }
    // <---- Static transforms
    mCamera2BaseTransfValid = true;
    return true;
}

/*
* @\brief pose topic callback
*/
void RGBD::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    robot_pose.setRotation(q);
    robot_pose.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    // cout<<"xyz"<<robot_pose.getOrigin().x()<<"y"<<robot_pose.getOrigin().y()<<"z"<<robot_pose.getOrigin().z()<<endl;
    return;
}

/*
callback for ICP algorithm uses pcl::IterativeClosestPoint
*/
void RGBD::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // cout<<"-------Entered callback---------"<<endl;

    // sensor_msgs::PointCloud2::ConstPtr msgOut;
    // // boost::shared_ptr<sensor_msgs::PointCloud2> msgOut;
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    tf2::Matrix3x3 rot_1;
    if (!mCamera2BaseTransfValid) getCamera2BaseTransform();

    rot_1 =  tf2::Matrix3x3(mCamera2BaseTransf.getRotation());

    transform_1 << rot_1[0][0], rot_1[0][1], rot_1[0][2], mCamera2BaseTransf.getOrigin().x(),
                    rot_1[1][0], rot_1[1][1], rot_1[1][2], mCamera2BaseTransf.getOrigin().y(),
                    rot_1[2][0], rot_1[2][1], rot_1[2][2], mCamera2BaseTransf.getOrigin().z(),
                    0, 0, 0, 1;
    
    Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

    rot_1 =  tf2::Matrix3x3(robot_pose.getRotation());

    transform_2 << rot_1[0][0], rot_1[0][1], rot_1[0][2], robot_pose.getOrigin().x(),
                    rot_1[1][0], rot_1[1][1], rot_1[1][2], mCamera2BaseTransf.getOrigin().y(),
                    rot_1[2][0], rot_1[2][1], rot_1[2][2], mCamera2BaseTransf.getOrigin().z(),
                    0, 0, 0, 1;
    // cout<<"transform_1 "<<endl<<transform_1<<endl;
    if(is_initial){
	    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr_w_frame (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr prev_pc(new pcl::PointCloud<pcl::PointXYZRGBA>);
        std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > prev_clustered_clouds_ptr;

        pcl::fromROSMsg(*msg, *prev_cloud_ptr);
        pcl::fromROSMsg(*msg, *prev_pc);
        pcl::transformPointCloud(*prev_cloud_ptr, *trans_prev_cloud_ptr, transform_2);
        // clusterCloud(filtered_cloud_ptr, prev_clustered_clouds_ptr);
        
        //get robot position 
        robot_pose_1 << robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), robot_pose.getOrigin().z();
        filterCloud(trans_prev_cloud_ptr, filtered_cloud_ptr);
        pcl::transformPointCloud(*filtered_cloud_ptr, *prev_cloud_ptr_w_frame, transform_2);



        _prev_cloud = *prev_cloud_ptr_w_frame;
        _prev_time_stamp = msg->header.stamp.toSec();
        _prev_rgba_cloud = *prev_pc;
        _prev_clustered_clouds = prev_clustered_clouds_ptr;
        //initialising the previous transformation
        prev_transformation = Eigen::Matrix4f::Identity();

        is_initial = false;
    }
    if(msg->header.stamp.toSec()- _prev_time_stamp > _time_between_cloud_points){//icp every T seconds
	    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(_prev_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_curr_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud_ptr_w_frame (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloudrgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr n_pcs(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr res_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr prev_rgba_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>(_prev_rgba_cloud));
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr curr_pc(new pcl::PointCloud<pcl::PointXYZRGBA>);
        std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > prev_clustered_clouds_ptr (_prev_clustered_clouds);
        std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > clustered_clouds_ptr;
        chrono::time_point<chrono::system_clock> start, end;
        start = chrono::system_clock::now(); 
        pcl::fromROSMsg(*msg, *current_cloud_ptr);
        pcl::fromROSMsg(*msg, *curr_pc);
        pcl::transformPointCloud(*current_cloud_ptr, *trans_curr_cloud_ptr, transform_1);    //     
        robot_pose_2 << robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), robot_pose.getOrigin().z(); //current robot pose

        double roll, pitch, yaw;
        tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);
        robot_pose_pred <<robot_pose_2[0]+0.2*cos(yaw), robot_pose_2[1] + 0.2* sin(yaw), robot_pose_2[2]; // predict robot pose
        // cout<< "robot_pose"<<endl<<robot_pose_2<<endl;
        filterCloud(trans_curr_cloud_ptr, filtered_cloud_ptr);
        pcl::transformPointCloud(*filtered_cloud_ptr, *curr_cloud_ptr_w_frame, transform_2);  //       
       
        Eigen::Vector3d robot_pose_change = robot_pose_2 - robot_pose_1; // chane of robot pose in x,y,z
        cout<<"robot_pose_change"<<endl<<robot_pose_change<<endl;
        Eigen::Vector3d pt, f_pt, Force;  
        pcl::copyPointCloud(*curr_cloud_ptr_w_frame, *n_pcs);
        pcl::copyPointCloud(*filtered_cloud_ptr, *res_pc);
        Eigen::Vector3d robot_pose_2_pt_diff;
        double time_to_dmin, ttc, d_min_temp, min_d_norm, sigma, x, risk_function, risk, max_dist, k =1;
        double norm_factor = robot_pose_change.norm();
        std::vector<Eigen::Vector3d> force_vector;
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D (*curr_cloud_ptr_w_frame, min_pt, max_pt);//get 
        max_dist = hypot(hypot(max_pt.x, min_pt.y), max_pt.z);//
        int Offset = 0.5, max_value = 1, step_saturation = 5;
        step_saturation = step_saturation + Offset;
        Force<<0,0,0;
        for (auto &it : n_pcs->points) {
            pt << it.x, it.y, it.z;
            robot_pose_2_pt_diff = robot_pose_pred - pt;
            time_to_dmin = -(robot_pose_2_pt_diff).dot(robot_pose_change)/(pow(robot_pose_change.norm(),2));

            d_min_temp = ((robot_pose_change.cross(robot_pose_2_pt_diff)).norm()) / robot_pose_change.norm();
            
            ttc = ((time_to_dmin - 1) * _time_between_cloud_points) + Offset;
            
            min_d_norm = d_min_temp/norm_factor;

            if(ttc < 0.01){
                ttc = 100;
            }
            sigma = k * (ttc);
            x = min_d_norm;

            /// Modified code
            risk_function = k/(sigma) * exp(- pow(x,2) / (2 * pow(sigma,2)) );
            if(max_value != 0){
                risk_function = (step_saturation*max_value) * risk_function;
                if(risk_function > max_value){
                    risk_function = max_value;
                }
            }else{
                risk_function = step_saturation * risk_function;
            }
            
            it.rgb = risk_function;
            f_pt << -pt/pt.norm();
            // force_vector.push_back((risk_function/pow(pt.norm(),2))*f_pt);
            Force+= (risk_function/pow(pt.norm(),2))*f_pt;
            // res_pc->points.at()
        }        
        cout<<Force;
        
        robot_pose_1 = robot_pose_2; //update robot pose
        _prev_time_stamp = msg->header.stamp.toSec();

        // pcl::PointXYZ min_pt, max_pt;
        // pcl::getMinMax3D (*filtered_cloud_ptr, min_pt, max_pt);//get 
        // max_dist = hypot(hypot(max_pt.x, min_pt.y), max_pt.z);//
        // cout<<"distances "<<max_pt.x <<min_pt.y <<max_pt.z<<endl;
        // cout<<"max distance "<<max_dist<<endl;

        // //Euclidean instantanious risk
        // int index = 0;
        // for (auto &it : n_pcs->points) {
        //     risk = hypot(hypot(it.x, it.y), it.z)/max_dist;
        //     it.rgb = (risk==risk)?risk:10;//convertColor(n_pcs->points[index].z); // put risk value as color 
        //     // cout<<it.rgb<<endl;
        //     // index += 4;
        // }
        n_pcs->header.frame_id = _frame_id;
        // n_pcs->header.frame_id = _base_frame_id;

        pc_pub.publish(n_pcs); //publishing the aligned pointcloud
        _prev_cloud = *curr_cloud_ptr_w_frame;
        // prev_transformation = curr_transformation;
        // _prev_rgba_cloud = *curr_pc;


        // // correspondence grouping and feature matching using 
        // correspondenceCloud(prev_rgba_cloud_ptr, curr_pc);
        // cout << "ICP has converged:" << icp.hasConverged()
        //     << " score: " << icp.getFitnessScore() << endl;


        // // clustering point clouds using euclidean clustering 
        // clusterCloud(filtered_cloud_ptr, clustered_clouds_ptr);
        // cout<<clustered_clouds_ptr.size()<<endl;
        // for (int i = 0; i < clustered_clouds_ptr.size() - 1; i++){
        //     cout << "Point Cloud " << i << " has got " << clustered_clouds_ptr[i]->size() << " Points" << endl;
        //     clustered_clouds_ptr[i]->header.frame_id = _base_frame_id;
        //     // pc_pub.publish(clustered_clouds_ptr[i]); //publishing the aligned pointcloud

        // }

        // icp registration for visual odometry
/*        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setTransformationEpsilon(_transformation_epsilon);
        icp.setMaximumIterations(_max_iters);
        icp.setMaxCorrespondenceDistance(_max_correspondence_distance);
        icp.setEuclideanFitnessEpsilon(_euclidean_fitness_epsilon);
     
        icp.setInputSource(filtered_cloud_ptr);
        icp.setInputTarget(prev_cloud_ptr);

        double diff_time = msg->header.stamp.toSec() - _prev_time_stamp; //calculating time btw the matching pointclouds
        end = chrono::system_clock::now();
        
        
        chrono::duration<double> elapsed_seconds = end-start;
        cout<<"elapsed time: " << elapsed_seconds.count() << "s\n"; 
       

        //cout<<"-------Matching clouds---------"<<endl;
        //start = chrono::system_clock::now(); 
        // inti_guess <<
        tf2::Transform guess_transf;
        // tf2::fromMsg(robot_pose, guess_transf);
        cout<<robot_pose.getOrigin().x()<<endl;
        // init_guess = tf2::transformToEigen (pose_msg);//converting geometry_msg to Eigen::Matrix4f
        // icp.align(*trans_cloud_ptr, robot_pose);
        icp.align(*trans_cloud_ptr);

        
        Eigen::Matrix4f t = icp.getFinalTransformation();
        Eigen::Matrix4f curr_transformation = prev_transformation*t; //final transformation matrix

        print4x4Matrix (curr_transformation);

        Eigen::Matrix3f rot_mat; //rotation matrix
        Eigen::Vector3f trans; //translation vector        
        trans << curr_transformation(0,3), curr_transformation(1,3), curr_transformation(2,3);
        rot_mat << curr_transformation(0,0), curr_transformation(0,1), 0,
                    curr_transformation(1,0), curr_transformation(1,1), 0,
                    0, 0, 1;
        
        Eigen::Quaternionf quat(rot_mat); //rotation matrix stored as a quaternion
        
        // transform from camera frame to base frame
        tf2::Transform T;
        T.setOrigin(tf2::Vector3(curr_transformation(0,3), curr_transformation(1,3), curr_transformation(2,3)));
        T.setRotation(tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
        if (!mCamera2BaseTransfValid) getCamera2BaseTransform();
        double roll, pitch, yaw;
        tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

        tf2::Quaternion quat_2;
        quat_2.setRPY(roll, pitch, yaw);

        mCamera2BaseTransf.setRotation(quat_2);
        tf2::Transform RT = mCamera2BaseTransf *T;//*mCamera2BaseTransf.inverse();
        geometry_msgs::Transform camera2base = tf2::toMsg(T);

        nav_msgs::OdometryPtr odomMsg = boost::make_shared<nav_msgs::Odometry>();
        odomMsg->header.stamp = ros::Time::now();
        odomMsg->header.frame_id = _frame_id; //remember to change to odom frame or other custom frame mOdometryFrameId; // frame
        // odomMsg->child_frame_id = _camera_frame_id; //camera frame
        odomMsg->pose.pose.position.x =  camera2base.translation.x; 
        odomMsg->pose.pose.position.y =  camera2base.translation.y; 
        odomMsg->pose.pose.position.z =  camera2base.translation.z; 
        odomMsg->pose.pose.orientation.x = camera2base.rotation.x;
        odomMsg->pose.pose.orientation.y = camera2base.rotation.y;
        odomMsg->pose.pose.orientation.z = camera2base.rotation.z;
        odomMsg->pose.pose.orientation.w = camera2base.rotation.w;
        
        
        //----------Note: for now the covariance is left at default values-----------
        //---------later covariance values will also be used,------------------------
        //---------so this can used as input to probabilistic filter like EKF/UKF----
        // Odometry pose covariance
        Eigen::MatrixXi c(6, 6);
        c = Eigen::MatrixXi::Identity(6, 6);
        odomMsg->pose.covariance[0]  = 0.1;
        odomMsg->pose.covariance[7]  = 0.1;
        odomMsg->pose.covariance[35] = 0.05;

        odomMsg->pose.covariance[14] = 1e10; // set a non-zero covariance on unused
        odomMsg->pose.covariance[21] = 1e10; // dimensions (z, pitch and roll); this
        odomMsg->pose.covariance[28] = 1e10; // is a requirement of pose_slam
        // Publish odometry message
        odom_pub.publish(odomMsg);*/
        

        
    }

    return;
}







// /*
// IMU topic callback
// */
// void RGBD::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     // ROS_INFO("imu callback");
//     if(is_imu_start)
//     {
//         _prev_acc = msg->linear_acceleration.x;
//         _prev_imu_time = msg->header.stamp.toSec();

//         is_imu_start = false;
//     }
//     else
//     {
//         _curr_acc = msg->linear_acceleration.x;
//         _curr_imu_time = msg->header.stamp.toSec();

//         double del_time = _curr_imu_time - _prev_imu_time;
//         double avg_acc = 0.5*(_prev_acc + _curr_acc);

//         _speed = avg_acc*del_time;
//         _yaw_rate = msg->angular_velocity.z;

//         _prev_acc = _curr_acc;
//         _prev_imu_time = _curr_imu_time;
//     }

//     return;
// }






// transform from camera frame to base frame
        // tf2::Transform T;
        // T.setOrigin(tf2::Vector3(curr_transformation(0,3), curr_transformation(1,3), curr_transformation(2,3)));
        // T.setRotation(tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
        // tf2::Transform RT = T*mCamera2BaseTransf.inverse();
        // tf2::convert(RT, mTf2Transf);
        // geometry_msgs::Transform camera2base = tf2::toMsg(T);

        // geometry_msgs::PoseWithCovarianceStamped curr_pose;
        // curr_pose.header.stamp = ros::Time::now();
        // curr_pose.header.frame_id = "base_link"; //remember to change to odom frame or other custom frame
        // curr_pose.pose.pose.position.x = trans[0];
        // curr_pose.pose.pose.position.y = trans[1];        
        // // curr_pose.pose.pose.position.z = trans[2];        
        // curr_pose.pose.pose.orientation.x = quat.x();
        // curr_pose.pose.pose.orientation.y = quat.y();        
        // curr_pose.pose.pose.orientation.z = quat.z();        
        // curr_pose.pose.pose.orientation.w = quat.w();
        // pose_pub.publish(curr_pose); //publishing the current pose

        // cout<<"-------ZED handles transformation between frames for realsense transformation required---------"<<endl; 
        // tf2::TransformListener *tf_ = new tf::TransformListener;
        // tf_->waitForTransform("odom", "realsense_link", t, ros::Duration(1.0));
        // tf_->lookupTransform("odom", "realsense_link", t, transform);




        // cloud_res = sl::Resolution(1280, 1024);

        // // Allocate PCL point cloud at the resolution
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr n_pcs(new pcl::PointCloud<pcl::PointXYZRGB>);
        // n_pcs->points.resize(cloud_res.area());
        // pcl::copyPointCloud(*current_cloud_ptr, *n_pcs);

        // shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(n_pcs);
        
        // // Set Viewer initial position
        // viewer->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
        // viewer->setCameraClipDistances(0.1,1000);
        // // Loop until viewer catches the stop signal
        // while (!viewer->wasStopped()) {

        //         //Lock to use the point cloud
        //         // mutex_input.lock();
        //         // float *p_data_cloud = data_cloud.getPtr<float>();
        //         int index = 0;

        //         // Check and adjust points for PCL format
        //         for (auto &it : n_pcs->points) {
        //             // float X = p_data_cloud[index];
        //             // if (!isValidMeasure(X)) // Checking if it's a valid point
        //                 // cout<<"invalid point"<<endl;
        //                 // it.x = it.y = it.z = it.rgb = 0;
        //             // else {
        //                 it.x = trans_cloud_ptr->points[index].x;
        //                 it.y = trans_cloud_ptr->points[index].y;
        //                 it.z = trans_cloud_ptr->points[index].z;
        //                 it.rgb = convertColor(trans_cloud_ptr->points[index].x);//convertColor(p_data_cloud[index+2]); // Convert a 32bits float into a pcl .rgb format
        //             // }
        //             index += 4;
        //         }

        //         // Unlock data and update Point cloud
        //         // mutex_input.unlock();
        //         viewer->updatePointCloud(n_pcs);
        //         viewer->spinOnce(10);
        // }

        // // Close the viewer
        // viewer->close();   


        // cv::Mat result; 

        // if (n_pcs->isOrganized()) {
        //     result = cv::Mat(n_pcs->height, n_pcs->width, CV_8UC3);

        //     if (!n_pcs->empty()) {

        //         for (int h=0; h<result.rows; h++) {
        //             for (int w=0; w<result.cols; w++) {
        //                 pcl::PointXYZRGBA point = n_pcs->points.at(w, h);

        //                 Eigen::Vector3i rgb = point.getRGBVector3i();

        //                 result.at<cv::Vec3b>(h,w)[0] = rgb[2];
        //                 result.at<cv::Vec3b>(h,w)[1] = rgb[1];
        //                 result.at<cv::Vec3b>(h,w)[2] = rgb[0];
        //             }
        //         }
        //     }
        // } 