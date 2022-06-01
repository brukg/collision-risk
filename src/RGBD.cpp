#include "rgbd_risk/RGBD.h"
#include "rgbd_risk/utils.h"
// #include "rgbd_risk/correspondence.h"
#include <iostream>
#include <chrono>
#include <mutex>
#include <math.h> 
#include <cstdlib>
#include <cmath>
#include <bits/stdc++.h>


#include <geometry_msgs/PoseWithCovarianceStamped.h> //rose pose msg with covariance
#include <geometry_msgs/PoseStamped.h> //ros pose msg
#include <nav_msgs/Odometry.h> //ros odometry msg
#include "std_msgs/String.h" //signal message



#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h> //conversion between tf and eigen




// #include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h> //generalized iterative closest point algorithm from pcl
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// risk visualizer libraries
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>

// ZED includes
// #include <sl/Camera.hpp>
#include <opencv2/core.hpp>

//visulaization marker
#include <visualization_msgs/Marker.h>


using namespace std;
// using namespace sl; 

// Mat data_cloud;
std::mutex mutex_input;
// sl::Resolution cloud_res;

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
    private_nh.param<std::string>("stop_signal_topic", _stop_signal_topic, "rgbd_pose");
    
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
    signal_pub = node.advertise<std_msgs::String>(_stop_signal_topic,1);
    // vis_pub = node.advertise<geometry_msgs::PoseStamped>( "visualization_marker", 0 );
    vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    vis_pub_centroid = node.advertise<geometry_msgs::PoseStamped>( "visualization_centroid", 0 );
    // vis_pub = node.advertise<geometry_msgs::PoseStamped>( "visualization_marker", 0 );

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

    } catch (tf2::TransformException& ex) {
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
    //cout<<"xyz"<<robot_pose.getOrigin().x()<<"y"<<robot_pose.getOrigin().y()<<"z"<<robot_pose.getOrigin().z()<<endl;
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

    if (mCamera2BaseTransfValid){
    rot_1 =  tf2::Matrix3x3(mCamera2BaseTransf.getRotation());

    transform_1 << rot_1[0][0], rot_1[0][1], rot_1[0][2], mCamera2BaseTransf.getOrigin().x(),
                    rot_1[1][0], rot_1[1][1], rot_1[1][2], mCamera2BaseTransf.getOrigin().y(),
                    rot_1[2][0], rot_1[2][1], rot_1[2][2], mCamera2BaseTransf.getOrigin().z(),
                    0, 0, 0, 1;
    }
    Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

    //......................................................................................................................................
    // cout<<"transform_1 "<<endl<<transform_1<<endl;
    if(is_initial){

        //cout<<"inside initial========================="<<endl;
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

        //cout<<"coming======================"<<endl;
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
        //cout<<"xyz "<<robot_pose.getOrigin().x()<<"y "<<robot_pose.getOrigin().y()<<"z "<<robot_pose.getOrigin().z()<<endl;

        double roll, pitch, yaw;
        tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);
        robot_pose_pred <<robot_pose_2[0]+0.2*cos(yaw), robot_pose_2[1] + 0.2* sin(yaw), robot_pose_2[2]; // predict robot pose

        //cout<< "robot_pose"<<endl<<robot_pose_2<<endl;

        filterCloud(trans_curr_cloud_ptr, filtered_cloud_ptr);
        tf2::Matrix3x3 rot_2(robot_pose.getRotation());
        transform_2 <<  rot_2[0][0], rot_2[0][1],   0, robot_pose.getOrigin().x(),
                        rot_2[1][0], rot_2[1][1],   0, robot_pose.getOrigin().y(),
                        0,              0,          1, robot_pose.getOrigin().z(),
                        0,              0,          0, 1;
        pcl::transformPointCloud(*filtered_cloud_ptr, *curr_cloud_ptr_w_frame, transform_2);  //       
       
        Eigen::Vector3d robot_pose_change = robot_pose_2 - robot_pose_1; // chane of robot pose in x,y,z
        //cout<<"robot_pose_change:"<<endl<<robot_pose_change<<endl; 
        //cout<<"robot pose 1:"<<robot_pose_1<<endl;
        //cout<<"robot pose 2:"<<robot_pose_2<<endl;
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
        // cout<<"random force"<<Force<<endl;
        // cout<<"random force norm"<<Force.norm()<<endl;

        //edowardo's=================================================================================>

        // clustering point clouds using euclidean clustering 
        clusterCloud(filtered_cloud_ptr, clustered_clouds_ptr);
        // cout<<clustered_clouds_ptr.size()<<endl;

        //getting centroids
        pcl::CentroidPoint<pcl::PointXYZ>centroid;
        pcl::PointXYZRGB c1;
        //array of risk for each cluster
        float risk_array[5] = { 0.0 };
        for (int i = 0; i < clustered_clouds_ptr.size(); i++){
             //cout << "Point Cloud " << i << " has got " << clustered_clouds_ptr[i]->size() << " Points" << endl;
           for (auto &it : clustered_clouds_ptr[i]->points) {
            //cout<<"come inside..."<<endl;
                // cout <<"print"<< it.x <<endl;
                // cout <<"print"<< it.y <<endl;
                // cout <<"print"<< it.z <<endl;
                centroid.add(pcl::PointXYZ (it.x, it.y, it.z));

           }
           centroid.get(c1);

            geometry_msgs::PoseStamped pp;
            pp.header.stamp = ros::Time();
            pp.header.frame_id = _base_frame_id;
            pp.pose.position.x =  c1.x;
            pp.pose.position.y =  c1.y;
            pp.pose.position.z =  c1.z;
            vis_pub_centroid.publish( pp );
           
            //cout<<"----------------centroid:"<<c1<<endl;

           // clustered_clouds_ptr[i]->header.frame_id = _base_frame_id;
           //cout<<"come inside..."<<endl;
           pt << c1.x, c1.y, c1.z;
            // cout<<"pt:"<<pt<<endl;
           robot_pose_2_pt_diff = robot_pose_pred - pt;

            //cout<<"robot_pose_2_pt_diff :"<<robot_pose_2_pt_diff<<endl;
            //cout<<"robot_pose_change..:"<<robot_pose_change<<endl; 0,0,0 even when teleoperate

            time_to_dmin = -(robot_pose_2_pt_diff).dot(robot_pose_change)/(pow(robot_pose_change.norm(),2));

            d_min_temp = ((robot_pose_change.cross(robot_pose_2_pt_diff)).norm()) / robot_pose_change.norm();

            //cout<<"time_to_dmin :"<<time_to_dmin<<endl; ===>none
            //cout<<"_time_between_cloud_points..:"<<_time_between_cloud_points<<endl;
            
            ttc = ((time_to_dmin - 1) * _time_between_cloud_points) + Offset;
            //cout<<"ttc..:"<<ttc<<endl;==> none

            min_d_norm = d_min_temp/norm_factor;

            if(ttc < 0.01){
                ttc = 100;
            }
            sigma = k * (ttc);
            x = min_d_norm;
            //cout<<"k..:"<<k<<endl;
            //cout<<"sigma...:"<<sigma<<endl;===>none

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
            
            c1.rgb = risk_function;
            risk_array[i] = risk_function;
        
            //cout<<"risk function:"<<risk_function<<endl;===>none
            f_pt << (robot_pose_2-pt)/(robot_pose_2-pt).norm();
            //force_vector.push_back((risk_function/pow(pt.norm(),2))*f_pt);
            Force+= f_pt*(risk_function/pow((robot_pose_2-pt).norm(),2));


            
        }
        float max_risk=0.0;
        for (int i=0;i<5;i++){
            //cout<<"i<"<<i<<" risk "<<risk_array[i]<<endl;
            if (risk_array[i]>max_risk){

                max_risk = risk_array[i];
            }
        }
        cout<<"maximum risk: "<<max_risk<<endl;
        
        

        



        cout<<"Force"<<Force<<endl;
        cout<<"Force norm"<<Force.norm()<<endl;
        

        tf2::Quaternion F_quaternion;
        double r,p,y;
        y = atan2(robot_pose_2(1,0), robot_pose_2(0,0));
        
        F_quaternion.setRPY(0,0,atan2(Force(1),Force(0)) - y);


        visualization_msgs::Marker marker;


        

        marker.header.frame_id = _frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;



        marker.pose.position.x = robot_pose.getOrigin().x();
        marker.pose.position.y = robot_pose.getOrigin().y();
        marker.pose.position.z = robot_pose.getOrigin().z();
        marker.pose.orientation.x = F_quaternion.x();
        marker.pose.orientation.y = F_quaternion.y();
        marker.pose.orientation.z = F_quaternion.z();
        marker.pose.orientation.w = F_quaternion.w();
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //vis_pub.publish( marker );

        if (Force.norm() > 0.004){
            vis_pub.publish( marker );
        }

        
        if(max_risk > 0.046){
            
            std_msgs::String msg;

            std::stringstream ss;
            ss << "Stop";
            msg.data = ss.str();
            cout<<"send a signal"<<endl;
            ROS_INFO("%s", msg.data.c_str());
            signal_pub.publish(msg);

        }
        

        robot_pose_1 = robot_pose_2; //update robot pose
        _prev_time_stamp = msg->header.stamp.toSec();

        n_pcs->header.frame_id = _frame_id;

        pc_pub.publish(n_pcs); //publishing the aligned pointcloud..............................................................///////////////////
        _prev_cloud = *curr_cloud_ptr_w_frame;



        
        

        
    }

    return;
}
