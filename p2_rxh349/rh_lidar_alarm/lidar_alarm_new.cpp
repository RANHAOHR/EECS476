#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
//#include <vector>


const double SAFE_DISTANCE = 0.1; // set alarm if anything is within 0.5m of the front of robot
const double LEFT_SAFE_DISTANCE = 0.1;
const double RIGHT_SAFE_DISTANCE = 0.1;
// these values to be set within the laser callback
float ping_dist=3.0; // global var to hold length of a SINGLE LIDAR ping--in front

int ping_index_= -1; // NOT real; callback will have to find this

double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;

bool front = false;
bool left = false;
bool right = false;
 
bool laser_alarm_1=false;   //5 cases;
bool laser_alarm_2=false;
bool laser_alarm_3=false;
bool laser_alarm_4=false;
bool laser_alarm_5=false;

int ping_min = 0;
int ping_max = 0;
int ping_left = 0;
int ping_right = 0;



int counter = 0;

ros::Publisher lidar_alarm_publisher_1;
ros::Publisher lidar_alarm_publisher_2;
ros::Publisher lidar_alarm_publisher_3;
ros::Publisher lidar_alarm_publisher_4;
ros::Publisher lidar_alarm_publisher_5;

//ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
  // if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;

      
   //    }
        ping_min = (int) (-1.00-angle_min_)/angle_increment_;    //-90 degree ping
        ping_max = (int) (1.00 - angle_min_)/angle_increment_;    
        ping_left = (int) (0.30-angle_min_)/angle_increment_;
        ping_right = (int) (-0.10-angle_min_)/angle_increment_;

    

//front
     for(ping_index_=ping_right;ping_index_<ping_left;ping_index_++){

         ping_dist = laser_scan.ranges[ping_index_];
         if(ping_dist <SAFE_DISTANCE){
           
            front = true;    //if there is one ping gives warning then count
            break;   //obstacles in front
            }      
         else{
            front = false;
              }
        } 
//left
     for(ping_index_=ping_left;ping_index_<ping_max;ping_index_++){
         ping_dist = laser_scan.ranges[ping_index_];
         if(ping_dist<LEFT_SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>25){ 
        left = true;  
       }
     else{
        left = false;
     }
     counter = 0;  //clear counter
//right
     for(ping_index_=ping_min;ping_index_<ping_right;ping_index_++){
         ping_dist = laser_scan.ranges[ping_index_];
         if(ping_dist<RIGHT_SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>1){ 
        right = true;  
       }
     else{
        right = false;
     }
     counter = 0;  //clear counter

std_msgs::Bool lidar_alarm_msg1;
std_msgs::Bool lidar_alarm_msg2;
std_msgs::Bool lidar_alarm_msg3;
std_msgs::Bool lidar_alarm_msg4;
std_msgs::Bool lidar_alarm_msg5;


//case1
    if(front &&!left && !right){
       ROS_WARN("DANGER,SOMETHING IN FRONT,JUST TURN LEFT	");
       laser_alarm_1=true;  // notice lidar and laser
       lidar_alarm_msg1.data = laser_alarm_1;
       lidar_alarm_publisher_1.publish(lidar_alarm_msg1);
    }
     else{
     laser_alarm_1 = false;
     lidar_alarm_msg1.data = laser_alarm_1;
     lidar_alarm_publisher_1.publish(lidar_alarm_msg1);
     }


//case2
    if(left && !right){
       ROS_WARN("DANGER,TRUN RIGHT");
       laser_alarm_2=true;
       lidar_alarm_msg2.data = laser_alarm_2;
       lidar_alarm_publisher_2.publish(lidar_alarm_msg2);
    }
    else{
       laser_alarm_2 = false;
       lidar_alarm_msg2.data = laser_alarm_2;
       lidar_alarm_publisher_2.publish(lidar_alarm_msg2);
     }

//case3
    if(!left && right){
       ROS_WARN("DANGER,TRUN LEFT");
       laser_alarm_3=true;
       lidar_alarm_msg3.data = laser_alarm_3;
       lidar_alarm_publisher_3.publish(lidar_alarm_msg3);
    }
    else{
       laser_alarm_3 = false;
       lidar_alarm_msg3.data = laser_alarm_3;
       lidar_alarm_publisher_3.publish(lidar_alarm_msg3);
     }

//case4
    if(front && left && right){
       ROS_WARN("DANGER,TRUN AROUND");
       laser_alarm_4=true;
       lidar_alarm_msg4.data = laser_alarm_4;
       lidar_alarm_publisher_4.publish(lidar_alarm_msg4);
    }
    else{
       laser_alarm_4 = false;
       lidar_alarm_msg4.data = laser_alarm_4;
       lidar_alarm_publisher_4.publish(lidar_alarm_msg4);
    }
//case5:false alarm
    if(!front && left && right){
       ROS_WARN("IN TUNNEL");
       laser_alarm_5=true;
       lidar_alarm_msg5.data = laser_alarm_5;
       lidar_alarm_publisher_5.publish(lidar_alarm_msg5);
    }
    else{
       laser_alarm_5 = false;
       lidar_alarm_msg5.data = laser_alarm_5;
       lidar_alarm_publisher_5.publish(lidar_alarm_msg5);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub1 = nh.advertise<std_msgs::Bool>("lidar_alarm_1", 1);
    lidar_alarm_publisher_1 = pub1; 

    ros::Publisher pub2 = nh.advertise<std_msgs::Bool>("lidar_alarm_2", 1);
    lidar_alarm_publisher_2 = pub2; 

    ros::Publisher pub3 = nh.advertise<std_msgs::Bool>("lidar_alarm_3", 1);
    lidar_alarm_publisher_3 = pub3;

    ros::Publisher pub4 = nh.advertise<std_msgs::Bool>("lidar_alarm_4", 1);
    lidar_alarm_publisher_4 = pub4; 

    ros::Publisher pub5 = nh.advertise<std_msgs::Bool>("lidar_alarm_5", 1);  
    lidar_alarm_publisher_5 = pub5;

    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);

    ros::spin();
    return 0; // should never get here, unless roscore dies
}

