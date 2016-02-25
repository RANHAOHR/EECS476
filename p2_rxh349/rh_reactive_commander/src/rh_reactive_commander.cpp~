#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h> // boolean message

bool g_lidar_alarm1=false; // global var for lidar alarm
bool g_lidar_alarm2=false; // global var for lidar alarm
bool g_lidar_alarm3=false; // global var for lidar alarm
bool g_lidar_alarm4=false; // global var for lidar alarm
bool g_lidar_alarm5=false;

void alarmCallback_1(const std_msgs::Bool& alarm_msg1) 
{ 
  g_lidar_alarm1 = alarm_msg1.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm1) {
     ROS_INFO("LIDAR alarm 1 received!"); 
  }
} 

void alarmCallback_2(const std_msgs::Bool& alarm_msg2) 
{ 
  g_lidar_alarm2 = alarm_msg2.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm2) {

     ROS_INFO("LIDAR alarm 2 received!"); 
  }
} 

void alarmCallback_3(const std_msgs::Bool& alarm_msg3) 
{ 
  g_lidar_alarm3 = alarm_msg3.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm3) {
     ROS_INFO("LIDAR alarm 3 received!"); 
  }
} 

void alarmCallback_4(const std_msgs::Bool& alarm_msg4) 
{ 
  g_lidar_alarm4 = alarm_msg4.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm4) {
     ROS_INFO("LIDAR alarm 4 received!"); 
  }
} 

void alarmCallback_5(const std_msgs::Bool& alarm_msg5) 
{ 
  g_lidar_alarm5 = alarm_msg5.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm5) {
     ROS_INFO("LIDAR alarm 5 received!"); 
  }
} 
//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel
int main(int argc, char **argv) {
    ros::init(argc, argv, "commander"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);

    ros::Subscriber alarm_subscriber_1 = n.subscribe("lidar_alarm_1",1,alarmCallback_1);
    ros::Subscriber alarm_subscriber_2 = n.subscribe("lidar_alarm_2",1,alarmCallback_2);
    ros::Subscriber alarm_subscriber_3 = n.subscribe("lidar_alarm_3",1,alarmCallback_3);
    ros::Subscriber alarm_subscriber_4 = n.subscribe("lidar_alarm_4",1,alarmCallback_4); 
    ros::Subscriber alarm_subscriber_5 = n.subscribe("lidar_alarm_5",1,alarmCallback_5); 
    //want to add a subscriber of distance

    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 1.2; // 1m/s speed command
    double low_speed = 0.3;
    double yaw_rate = 1.2; //0.5 rad/sec yaw rate command
  //  double low_yaw_rate = 1.0;
    double time_sec = 0.5; // should move 3 meters or 1.5 rad in 3 seconds   
      
    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;    
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;   

    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      ros::spinOnce();
      loop_timer.sleep();
    }
    while(ros::ok()) { // do forever
      
        twist_cmd.angular.z=0.0; // do not spin 
        twist_cmd.linear.x=speed; //command to move forward
        timer=0.0;
        while(!g_lidar_alarm1 && !g_lidar_alarm2 && !g_lidar_alarm3 && !g_lidar_alarm4 && !g_lidar_alarm5) { 
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          ros::spinOnce();
          loop_timer.sleep();
          }
 
        //here if got an alarm; turn CCW until alarm clears

        twist_cmd.linear.x=0.0; //stop moving forward
        twist_cmd.angular.z=yaw_rate; //and start spinning in place
        timer=0.0; //reset the timer
        while(g_lidar_alarm1 && !g_lidar_alarm2 && !g_lidar_alarm3 && !g_lidar_alarm4 && !g_lidar_alarm5) { // case1  
            ROS_INFO("Under Case 1 !"); 
            twist_commander.publish(twist_cmd);
            timer+=sample_dt;
            ros::spinOnce();
            loop_timer.sleep();  
                                      
          }
     
        twist_cmd.linear.x=0.0; //stop moving forward
        twist_cmd.angular.z=-yaw_rate; //and start spinning in place
        timer=0.0; //reset the timer
        while(!g_lidar_alarm1 && g_lidar_alarm2 && !g_lidar_alarm3 && !g_lidar_alarm4 && !g_lidar_alarm5) { // case2
          ROS_INFO("Under Case 2 !");
          while(timer<time_sec){
              twist_commander.publish(twist_cmd);
              timer+=sample_dt;
                    loop_timer.sleep();
               } 
          ros::spinOnce();
            
   
              
          }

        twist_cmd.linear.x=0.0; //stop moving forward
        twist_cmd.angular.z=yaw_rate; //and start spinning in place
        timer=0.0; //reset the timer
        while(!g_lidar_alarm1 && !g_lidar_alarm2 && g_lidar_alarm3 && !g_lidar_alarm4 && !g_lidar_alarm5) { // case3
          ROS_INFO("Under Case 3 !");
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          ros::spinOnce();
          loop_timer.sleep();
          }
 //special cases
        twist_cmd.linear.x=0.0; //stop moving forward
        twist_cmd.angular.z=yaw_rate; //and start spinning in place
        timer=0.0; //reset the timer
        while(!g_lidar_alarm1 && !g_lidar_alarm2 && !g_lidar_alarm3 && g_lidar_alarm4 && !g_lidar_alarm5) { // case4
          ROS_INFO("Under Case 4 !");
          while(timer<2.6){
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
                }
          ros::spinOnce();           
          }

        twist_cmd.angular.z=0.0; // do not spin 
        twist_cmd.linear.x=low_speed; //command to move forward slowly
        timer=0.0;
        while(!g_lidar_alarm1 && !g_lidar_alarm2 && !g_lidar_alarm3 && !g_lidar_alarm4 && g_lidar_alarm5) { 
          ROS_INFO("Under Case 5 !");
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          ros::spinOnce();
          loop_timer.sleep();
          }
 }   
    //done commanding the robot; node runs to completion
}

