

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <rh_action_server/actmsgAction.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h> // boolean message

//ros::Publisher twist_commander;

const double g_move_speed=1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed=1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt=0.01;

class RhActionServer {
private:
    ros::NodeHandle nh_;
    ros::Publisher  twist_new;
  // we'll need a node handle; get one upon instantiation
    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<rh_action_server::actmsgAction> as_;
    
    // here are some message types to communicate with our client(s)
  //  example_action_server::demoGoal goal_; // goal message, received from client
    rh_action_server::actmsgResult result_; // put results here, to be sent back to the client when done w/ goal
    rh_action_server::actmsgFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    nav_msgs::Path goal_receive;   /////received goal
    geometry_msgs::Pose pose_desired;

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);
void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading);

double total_distance;

public:
    RhActionServer(); //define the body of the constructor outside of class definition

    ~RhActionServer(void) {
    }
    // Action Interface
    void executeCallback(const actionlib::SimpleActionServer<rh_action_server::actmsgAction>::GoalConstPtr& goal); 

double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom!!!!

int alarm_sig;

};
///////////////////////////////////////////////////////////////////////////define member functions
double RhActionServer::sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

double RhActionServer::min_spin(double spin_angle) {
        while (spin_angle>M_PI) {  
            spin_angle -= 2.0*M_PI;}
        while (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
} 

double RhActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

geometry_msgs::Quaternion RhActionServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;                                   
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void RhActionServer::do_spin(double spin_ang) {
    ros::Rate loop_timer(100);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    ROS_INFO("spin final_time: %f", final_time);
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    ROS_INFO("BEFORE CIRCLE");
    while(timer<final_time) {
          ROS_INFO("IN SPIN");
          g_twist_commander.publish(g_twist_cmd);            
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    ROS_INFO("DO HALT");
    do_halt(); 
}

void RhActionServer::do_halt() {
    ros::Rate loop_timer(100);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          ROS_INFO("IN HALT");
          g_twist_commander.publish(g_twist_cmd);            
          loop_timer.sleep(); 
          }   
}

void RhActionServer::do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(100);
    double timer=0.0;
  //  double final_time = fabs(distance)/g_move_speed;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    ROS_INFO("Move final time %f", final_time);
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
           ros::spinOnce();
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}
const double g_dist_tol = 0.1;
void RhActionServer::get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 double dx = 0.0;
 double dy = 0.0;
 // dist = 0.0; //FALSE!!
 dx = goal_pose.position.x - current_pose.position.x;
 dy = goal_pose.position.y - current_pose.position.y;
 dist = sqrt((dx*dx) + (dy*dy));
 //heading = convertPlanarQuat2Phi(goal_pose.orientation);

 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
 }
 else {
 //   heading = 0.0; //FALSE!!
   heading = atan2(dy,dx);
 }

}

RhActionServer::RhActionServer() :
   as_(nh_, "path_action", boost::bind(&RhActionServer::executeCallback, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of RhActionServer...");
    // do any other desired initializations here...specific to your implementation
      //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;

    g_twist_commander = nh_.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);  ///////////////////here

    //g_twist_commander = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);  ///////////////////here

    total_distance = 0.0;

    alarm_sig = 0;
    //twist_new = twist_commander;
    as_.start(); //start the server running
}

void RhActionServer::executeCallback(const actionlib::SimpleActionServer<rh_action_server::actmsgAction>::GoalConstPtr& goal) {
 
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    ROS_INFO("in executeCallback");
 //   ROS_INFO("goal input is: %d", goal->input);  ///cannot get here
    goal_receive = goal->nav_path;
    int npts = goal_receive.poses.size();
    ROS_INFO("received %d poses",npts);   

for (int i=0;i<npts;i++) {

 // ROS_INFO("ENTER CIRCLE");
  
      alarm_sig = goal->input;

      if(alarm_sig == 1){ 
        do_halt(); ///want every iter check if alarm
        do_spin(0.8);
        ROS_INFO("Gazebo is meeting something");
      feedback_.delta_angles += 0.8;
      as_.publishFeedback(feedback_);
        alarm_sig = 0;
      }


      pose_desired = goal_receive.poses[i].pose;
     
      get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
      ROS_INFO("GET POSE");

      total_distance += travel_distance;

     // yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
      yaw_current = 0.0;
      spin_angle = yaw_desired - yaw_current; // spin this much
      //spin_angle = yaw_desired;
      ROS_INFO("Spin_angle is : %f", spin_angle);
    
      spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
      feedback_.delta_angles += spin_angle;
      as_.publishFeedback(feedback_);
      ROS_INFO("feedback sent"); 

      do_spin(spin_angle); // carry out this incremental action
 
      g_current_pose.orientation = pose_desired.orientation; 
      do_move(travel_distance); 
      ROS_INFO("next round");  

       if (as_.isPreemptRequested()){  //check if cancelled,do not want to cancel during alarm 
          ROS_WARN("goal cancelled!");
          result_.travel_dist = total_distance;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
          }
 
     }

          result_.travel_dist = total_distance;
          as_.setSucceeded(result_);  // tell the client we have given up on this goal; send the result message as well
    

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rh_action_server_node"); // name this node 

    RhActionServer as_object; // create an instance of the class "ExampleActionServer"
     
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();  //spin and spinonce use together

    return 0;
}

