// example_action_client: 
// wsn, October, 2014

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<rh_action_server/actmsgAction.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


bool g_lidar_alarm1=false; // global var for lidar alarm
bool g_lidar_alarm2=false; // global var for lidar alarm
bool g_lidar_alarm=false; // global var for lidar alarm


double delta_angle = 0.0;

bool g_goal_active = false; //some global vars for communication with callbacks

geometry_msgs::PoseStamped pose_stamped;  
geometry_msgs::Pose pose;
geometry_msgs::Quaternion quat; 

rh_action_server::actmsgGoal goal;
actionlib::SimpleActionClient<rh_action_server::actmsgAction> *action_new;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0); 
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}
//callbacks
void doneCb(const actionlib::SimpleClientGoalState& state,
        const rh_action_server::actmsgResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("THe final travel distance is %f",result->travel_dist);
    g_goal_active = false;
}

void feedbackCb(const rh_action_server::actmsgFeedbackConstPtr& feedback) {
      delta_angle += feedback->delta_angles;
      ROS_INFO("Feedback: Gazebo already turned %f", delta_angle);
}

void activeCb()
{
  ROS_INFO("Goal is active");
  g_goal_active = true;
}


void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 

  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
     ROS_INFO("LIDAR alarm received!");
      (*action_new).cancelGoal();
      goal.input = 1;
             }
      else{goal.input = 0;}

}
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

int main(int argc, char** argv) {
        ros::init(argc, argv, "rh_action_client_node"); // name this node 
        ros::NodeHandle n; //
        // stuff a goal message:
         ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm_1",1,alarmCallback);
        actionlib::SimpleActionClient<rh_action_server::actmsgAction> action_client("path_action", true);
        action_new = &action_client;
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever
        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }       
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        while(true) {

            pose.position.x = 1.0; // 
            pose.position.y = 0.0;
            pose.position.z = 0.0; // let's hope so!
            pose.orientation.x = 0.0; //always, for motion in horizontal plane
            pose.orientation.y = 0.0; // ditto
            pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
            pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
            pose_stamped.pose = pose;
            goal.nav_path.poses.push_back(pose_stamped);  //2D vector

            action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
            ros::spinOnce();
            
            goal.nav_path.poses.pop_back();
            quat = convertPlanarPhi2Quaternion(-0.50); // get a quaterion corresponding to this heading
            pose_stamped.pose.orientation = quat;   
            pose_stamped.pose.position.x = 0.0; // say desired y-coord is 4.9
            goal.nav_path.poses.push_back(pose_stamped); 

            //action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
            ros::spinOnce();
            //goal.nav_path.poses.pop_back();

    //        goal.nav_path.poses.clear();

            action_client.waitForResult(); 
/*
            if(delta_angle >= 6.28){   //if angle is more than 2pi in one spin, want it to stop 
            ROS_INFO("Cancelling goal !");  
            ROS_INFO("delta_angle = %f", delta_angle);         
             action_client.cancelGoal();
           }
         */  
         
        }
     
    return 0;
}
