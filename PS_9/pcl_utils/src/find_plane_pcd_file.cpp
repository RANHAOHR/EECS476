//find_plane_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// can select a patch; then computes a plane containing that patch, which is published on topic "planar_pts"
// illustrates use of PCL methods: computePointNormal(), transformPointCloud(), 
// pcl::PassThrough methods setInputCloud(), setFilterFieldName(), setFilterLimits, filter()
// pcl::io::loadPCDFile() 
// pcl::toROSMsg() for converting PCL pointcloud to ROS message
// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn March 2016

#include<ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs


using namespace std;
extern PclUtils *g_pcl_utils_ptr; 

//this fnc is defined in a separate module, find_indices_of_plane_from_patch.cpp
extern void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices);

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rough_stool_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to stool pts from Rvis tool    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Stool_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    vector<int> indices;

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    string fname = "/home/user/ros_ws/src/EECS476/PS_9/pcd_images/coke_can.pcd";
    //cout << "enter pcd file name: "; //prompt to enter file name
    //cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);

    sensor_msgs::PointCloud2 ros_cloud, stool_planar_cloud, downsampled_cloud; //here are ROS-compatible messages
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);

    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    cout << "done voxel filtering" << endl;

    cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    PclUtils pclUtils(&nh); //instantiate a PclUtils object--a local library w/ some handy fncs
    g_pcl_utils_ptr = &pclUtils; // make this object shared globally, so above fnc can use it too

    cout << " select a patch of points to find corresponding plane..." << endl; //prompt user action

//******************************************************************//
    sensor_msgs::PointCloud2 stoolPts; //create a ROS message
    ros::Publisher Stool = nh.advertise<sensor_msgs::PointCloud2> ("/stoolpts", 1);
    
    pclUtils.getDesPts(pclKinect_clr_ptr, rough_stool_cloud_ptr);

    sensor_msgs::PointCloud2 canPts;
    ros::Publisher Can = nh.advertise<sensor_msgs::PointCloud2> ("/canpts", 1);
    pclUtils.getCanPts(pclKinect_clr_ptr, can_pts_cloud_ptr);
    pcl::toROSMsg(*can_pts_cloud_ptr, canPts);


    pclUtils.from_RGB_to_XYZ(rough_stool_cloud_ptr, Stool_xyz_ptr);
    find_indices_of_plane_from_patch(downsampled_kinect_ptr, Stool_xyz_ptr, indices);    
    pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *plane_pts_ptr); //extract these pts into new cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_stool_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to stool pts from Rvis tool 
    pclUtils.find_final_cloud(plane_pts_ptr, final_stool_cloud_ptr);
    pcl::toROSMsg(*final_stool_cloud_ptr, stoolPts);
    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    pcl::toROSMsg(*plane_pts_ptr, stool_planar_cloud); //convert to ros message for publication and display

    //loop to test for new selected-points inputs and compute and display corresponding planar fits 
    while (ros::ok()) {

        Stool.publish(stoolPts);  
        Can.publish(canPts);

        pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
        pubPlane.publish(stool_planar_cloud); // display the set of points computed to be coplanar w/ selection
        pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.1).sleep();
    }

    return 0;
}
