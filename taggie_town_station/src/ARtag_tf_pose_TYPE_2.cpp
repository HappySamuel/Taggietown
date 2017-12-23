//********************************************************
//This is the code for coordinate transformation
//between camera and two tags. The out put will be the relative coordinate
//between to tags.
//cam->tag1
//cam->tag2  } =>tag1->tag2
//If there is no marker as input coming it will pulbish the data with zero in every element
//*******************************************************************
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle n;

  ros::Publisher ino_pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("ino_pose", 10);

  tf::TransformListener listener;

  ros::Rate rate(15.0);

  while (n.ok()){
      tf::StampedTransform tf_cam_to_map, tf_cam_to_ino;
      try{
       ros::Time now = ros::Time::now();

        listener.lookupTransform("/samuel_station", "/SAMUEL_ar_marker_100",ros::Time(0), tf_cam_to_map);
	

        listener.lookupTransform("/samuel_station", "/SAMUEL_ar_marker_104", ros::Time(0), tf_cam_to_ino);


      }
      catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();

      }
        geometry_msgs::PoseStamped mypose;
ros::Time time_now = ros::Time::now();
if(time_now-tf_cam_to_ino.stamp_<ros::Duration(0.1)){

    mypose.header.frame_id = "map";
    double x= -tf_cam_to_map.getOrigin().x()+ tf_cam_to_ino.getOrigin().x();
    double y= -tf_cam_to_map.getOrigin().y()+ tf_cam_to_ino.getOrigin().y();
    double z= -tf_cam_to_map.getOrigin().z()+ tf_cam_to_ino.getOrigin().z();


    tf::Quaternion q(x,y,z,0), Q, qq;
    Q=tf_cam_to_map.getRotation();
    

    Q = Q.inverse();
    q*=Q.inverse();
    q=Q*q;

    mypose.pose.position.x=q[0];
    mypose.pose.position.y=q[1];
    mypose.pose.position.z=q[2];
    qq=Q*tf_cam_to_ino.getRotation();
    
    tf::Matrix3x3 quat_now(qq);
    double roll_now, pitch_now, yaw_now;

    quat_now.getRPY(roll_now, pitch_now, yaw_now);
    mypose.pose.orientation.x = roll_now;		//roll_now
    mypose.pose.orientation.y = pitch_now;		//pitch_now
    mypose.pose.orientation.z = yaw_now;		//yaw_now
    mypose.pose.orientation.w = 0;
    
    //mypose.pose.orientation.x=qq[0];
    //mypose.pose.orientation.y=qq[1];
    //mypose.pose.orientation.z=qq[2];
    //mypose.pose.orientation.w=qq[3];
    
    mypose.header.stamp = ros::Time::now();


     ino_pose_pub_.publish(mypose);
     ROS_INFO("%f\t %f\t %f",q[0],q[1],q[2]);
     ROS_INFO("get pose data!");
}/*else{					//useless
    mypose.pose.position.x=0;
    mypose.pose.position.y=0;
    mypose.pose.position.z=0;

    mypose.pose.orientation.x=0;
    mypose.pose.orientation.y=0;
    mypose.pose.orientation.z=0;

    //mypose.pose.orientation.x=0;
    //mypose.pose.orientation.y=0;
    //mypose.pose.orientation.z=0;
    //mypose.pose.orientation.w=0;
    //mypose.header.stamp = ros::Time::now();

    ino_pose_pub_.publish(mypose);
    ROS_INFO("!no pose data");
  }*/
        rate.sleep();

}

     return 0;
}
