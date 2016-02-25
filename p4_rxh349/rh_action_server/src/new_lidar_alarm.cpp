#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
//#include <vector>

const double SAFE_DISTANCE = 1.0; //set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist=3.0; // global var to hold length of a SINGLE LIDAR ping--in front

int ping_index_= -1; // NOT real; callback will have to find this

double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;

bool front = false;

//bool right = false;
 
bool laser_alarm_1=false;   //5 cases;



int ping_front_min = 0;
int ping_front_max = 0;




int counter = 0;

ros::Publisher lidar_alarm_publisher_1;



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
        ping_front_max = (int) (0.6-angle_min_)/angle_increment_;    //-90 degree ping
        ping_front_min = (int) (-0.6 - angle_min_)/angle_increment_;    


    

//front
     for(ping_index_=ping_front_min;ping_index_<ping_front_max;ping_index_++){
         ping_dist = laser_scan.ranges[ping_index_];
         if(ping_dist<SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>1){ 
        front = true;  
       }
     else{
        front = false;
     }
     counter = 0;  //clear counter
     

std_msgs::Bool lidar_alarm_msg1;



//case1
    if(front){
       ROS_WARN("DANGER,SOMETHING IN FRONT,JUST TURN LEFT");
       laser_alarm_1=true;  // notice lidar and laser
       lidar_alarm_msg1.data = laser_alarm_1;
       lidar_alarm_publisher_1.publish(lidar_alarm_msg1);
    }
     else{
     laser_alarm_1 = false;
     lidar_alarm_msg1.data = laser_alarm_1;
     lidar_alarm_publisher_1.publish(lidar_alarm_msg1);
     }
     
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "new_lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub1 = nh.advertise<std_msgs::Bool>("lidar_alarm_1", 1);
    lidar_alarm_publisher_1 = pub1; 


    ros::Subscriber lidar_subscriber = nh.subscribe("/robot0/laser_0", 1, laserCallback);
    //ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);

    ros::spin();
    return 0; // should never get here, unless roscore dies
}

