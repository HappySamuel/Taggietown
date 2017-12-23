#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PointStamped.h>
#include <time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>	/*cos(), sin(), atan()*/

/*-------------------------------------Define Variables Here-------------------------------------*/
#define CONTROL_LOOP_FREQ 10
#define kv 0.18
#define kw 0.25
#define d_t 0.1		//frequency = 10 (Hz)
#define PI 3.14159265359

class InoCar_Controller{
public:
    InoCar_Controller(const nav_msgs::Path& path){
	mypath = path;

    //publish topic
	pub_1 = n.advertise<std_msgs::Float64>("V_cmd",1000);
    	pub_2 = n.advertise<std_msgs::Float64>("W_cmd",1000);
    	pub_3 = n.advertise<geometry_msgs::PointStamped>("samuel_goal", 1000);
    	pub_4 = n.advertise<nav_msgs::Path>("samuel_path", 4000);

    //subscribe topic
    	sub_pose = n.subscribe("/SAMUEL/ino_pose", 10000, &InoCar_Controller::posecb, this);

    	timer = n.createTimer(ros::Duration((float)1/CONTROL_LOOP_FREQ), &InoCar_Controller::loop_control, this);
    	index = 0;
    }

/*----------------------Location XY & Yaw angle Now given by AR-Track-Alvar----------------------*/
    void posecb(const geometry_msgs::PoseStamped::ConstPtr &pose){

    	pose_now = *pose;

    	x_now = pose_now.pose.position.x;
    	y_now = pose_now.pose.position.y;
    	z_now = pose_now.pose.position.z;
    	
	yaw_now = pose_now.pose.orientation.z;	//using this when you only have 1 computer
						//using with ARtag_tf_pose_TYPE_2 in .launch file

	//yaw_now = pose_now.pose.orientation.w; 	//using this when feed /ino_pose from others
						//using with ARtag_tf_pose in .launch file

        //ROS_INFO("position_x = %f \t position_y = %f \t yaw = %f \n", x_now, y_now, yaw_now);
    }

/*-------------------------Get the Desired Points from the Desired Path--------------------------*/
    void get_point(){
	x_d = mypath.poses[index].pose.position.x;
	y_d = mypath.poses[index].pose.position.y;
	
	index = index + 1;
    }
	
/*------------------Put the Function here that Wish to be continue doing in Loop-----------------*/
    void loop_control(const ros::TimerEvent& event){
    	std_msgs::Float64 Vcmd, Wcmd;

	get_point();

	TwoWD_car_control(x_d, y_d, x_now, y_now, yaw_now);	// yaw_now <---> theta_now

	Vcmd.data = V;
	Wcmd.data = W;
	
	pub_1.publish(Vcmd);
	pub_2.publish(Wcmd);

    	geometry_msgs::PointStamped inogoal;
    	inogoal.point.x = x_d;
    	inogoal.point.y = y_d;
    	inogoal.point.z = 0;
    	inogoal.header.frame_id = "/map";

    	pub_3.publish(inogoal);
    	pub_4.publish(mypath);
    }

/*--------Refer to  << Tracking Control of a Mobile Robot using Linear Interpolation >> ---------*/
/*------------by Gustavo Scaglia, Vicente Mut, Andres Rosales, Lucia Quintero Montoya------------*/
/*---------------------------------------Main Equations Here-------------------------------------*/
    void TwoWD_car_control(double x_d, double y_d, double x_now, double y_now, double theta_now){
    	
	double delta_x = x_d - x_now;		//delta_x = desired_x(k+1) - follow_x(k)
	double delta_y = y_d - y_now;		//delta_y = desired_y(k+1) - follow_y(k)	
	double W_V = 2*(delta_y*cos(theta_now) - delta_x*sin(theta_now))/(delta_x*delta_x + delta_y*delta_y);
	//double theta_d = atan((W_V*delta_x + sin(theta_now))/(-W_V*delta_y + cos(theta_now)));
	double theta_d = atan2((W_V*delta_x + sin(theta_now)),(-W_V*delta_y + cos(theta_now)));
	double delta_theta = theta_d - theta_now;
	if(delta_theta >= PI) delta_theta = delta_theta - 2*PI;
	if(delta_theta <= -PI) delta_theta = delta_theta + 2*PI;
	W = delta_theta/d_t;
	a = sin(theta_d) - sin(theta_now);
	b = cos(theta_now) - cos(theta_d);
	V = delta_theta*(delta_x*a/d_t + delta_y*b/d_t)/(a*a + b*b);

	V = kv * V;
	W = kw * W;
    }

/*---------------------Private Variables only use in this .cpp are defined Here------------------*/
private:
    ros::NodeHandle n;
    ros::Subscriber sub_pose;
    ros::Publisher pub_1, pub_2, pub_3, pub_4;
    ros::Timer timer;
    geometry_msgs::PoseStamped pose_now;
    
    double a, b;
    double V, W;
    double x_now, y_now, z_now; 
    double roll_now, pitch_now, yaw_now;
    double x_d, y_d;
    int index;
    nav_msgs::Path mypath;
    
};



void get_trajectory(bool);
nav_msgs::Path path;


int main(int argc, char **argv){
    //Initiate ROS
    ros::init(argc, argv, "ARTAG_info_to_InoCar_cmd");
    bool haspath = false;
    get_trajectory(!haspath);
    //Create an object of class Subscribe & Publish that will take care of everything
    InoCar_Controller taggie_town_station(path);

    ros::spin();

    return 0;
}

/*----------------------------------Desired Path from Excels-------------------------------------*/
void get_trajectory(bool b){
    double temx, temy;
    std::string headline;
    path.header.frame_id="map";
    std::vector<geometry_msgs::PoseStamped> plan;
    geometry_msgs::PoseStamped tempose;
	
	std::ifstream myfile;
	ROS_INFO("already open");
	myfile.open("/home/happysamuel/catkin_ws/src/taggie_town_station/trajectory/traj_loop_eight.txt", std::ios::in);
	if(myfile.is_open()){
	    ROS_INFO("open");
	    getline(myfile, headline);
	    
	    while(myfile >> temx >> temy){
		tempose.pose.position.x = temx;
		tempose.pose.position.y = temy;
		plan.push_back(tempose);
	    }
	}
	else{
	    ROS_INFO("unable to open");
	}
    path.poses = plan;
    //ROS_INFO("%f", path.poses[1570].pose.position.x);
}

