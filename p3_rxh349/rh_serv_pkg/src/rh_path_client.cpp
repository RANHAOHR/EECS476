//path_client:
// illustrates how to send a request to the path_service service

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
#include <rh_serv_pkg/PathSrv.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0); 
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<rh_serv_pkg::PathSrv>("path_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    rh_serv_pkg::PathSrv path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;  
    geometry_msgs::Pose pose;
    pose.position.x = 3.0; // say desired x-coord is 3
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);  //2D vector
    
    // some more poses...
    quat = convertPlanarPhi2Quaternion(0.80); // get a quaterion corresponding to this heading
    pose_stamped.pose.orientation = quat;   
    pose_stamped.pose.position.y=5.0; // say desired y-coord is 5.0
    pose_stamped.pose.position.x = 7.9; // say desired y-coord is 4.9
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    
    quat = convertPlanarPhi2Quaternion(3.03);
    pose_stamped.pose.orientation = quat;  
    //desired position is not updated...just the desired heading 
    pose_stamped.pose.position.y=5.0; // 
    pose_stamped.pose.position.x=7.9; // 
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    quat = convertPlanarPhi2Quaternion(3.08);
    pose_stamped.pose.orientation = quat;  
    //desired position is not updated...just the desired heading 
    pose_stamped.pose.position.y=5.4; // 
    pose_stamped.pose.position.x=1.0; // 
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    quat = convertPlanarPhi2Quaternion(2.20);
    pose_stamped.pose.orientation = quat;  
    //desired position is not updated...just the desired heading 
    pose_stamped.pose.position.y=6.5; // 
    pose_stamped.pose.position.x=0.3; // 
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    quat = convertPlanarPhi2Quaternion(1.61);
    pose_stamped.pose.orientation = quat;  
    //desired position is not updated...just the desired heading 
    pose_stamped.pose.position.y=21.0; // 
    pose_stamped.pose.position.x=-0.3; // 
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    client.call(path_srv);

    return 0;
}
