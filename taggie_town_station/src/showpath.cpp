#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include <cmath>

nav_msgs::Path path;
std::vector< geometry_msgs::PoseStamped> v;
int i;
int get;
#define CONTROL_LOOP_FREQ 5
void posecb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  geometry_msgs::PoseStamped data;

    data.pose.position.x= float(msg->pose.position.x);
    data.pose.position.y =float(msg->pose.position.y);

    data.pose.orientation.x=0;
    data.pose.orientation.y=0;
    data.pose.orientation.z=0;
    data.pose.orientation.w=0;
    v.push_back(data);

    //if(v.size()>200)v.clear();		// to constraint data being display
}

void loop_path(const ros::TimerEvent& event){
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    path.poses=v;
    get=1;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_listener");
  ros::NodeHandle n;
  ros::Subscriber ino_pose_sub = n.subscribe("ino_pose", 1000, posecb);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("ino_path", 10);
    i=0;get=0;
  ros::Timer timer = n.createTimer(ros::Duration((float)1/CONTROL_LOOP_FREQ),loop_path);

  while(n.ok()){
      if(get==1){

        ROS_INFO("haha");
        path_pub.publish(path);
        get=0;


      }
        ros::spinOnce();
  }



  return 0;
}

