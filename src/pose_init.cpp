#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h> 
#include <iostream>
#include <sstream>

using namespace std;

geometry_msgs::PoseWithCovarianceStamped ini_pose;

class PoseInit
{
private:
  ros::NodeHandle nh;
  ros::Subscriber amcl_sub;
  ros::Publisher init_pub;
  ros::ServiceServer init_srv;


public:
  PoseInit()
  {
    amcl_sub 
      = nh.subscribe("amcl_pose", 1, &PoseInit::callback, this);
    init_pub 
      = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    init_srv 
      = nh.advertiseService("pose_init_srv", &PoseInit::sendData, this); 
  }

  void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
  {
    //ini_pose.header = pose->header;
    //ini_pose.pose.pose = pose->pose.pose;
    ini_pose = *pose;
  }  

  bool sendData(std_srvs::Empty::Request& req,
		std_srvs::Empty::Response& res)
  {
    ros::ServiceClient clear_unknown_space_clt 
      = nh.serviceClient<std_srvs::Empty>("clear_unknown_space");
    ros::ServiceClient clear_costmaps_clt 
      = nh.serviceClient<std_srvs::Empty>("clear_costmaps");

    std_srvs::Empty emp;

    clear_unknown_space_clt.call( emp );
    clear_costmaps_clt.call( emp );
    /*
    double cov[36] =  {20*20, 0, 0, 0, 0, 0, 
		       0, 20*20, 0, 0, 0, 0, 
		       0, 0, 0, 0, 0, 0,
		       0, 0, 0, 0, 0, 0,
		       0, 0, 0, 0, 0, 0,
		       0, 0, 0, 0, 0, 2}; 
    */
    
    ini_pose.pose.covariance = {0.2*0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 
				0.0, 0.2*0.2, 0.0, 0.0, 0.0, 0.0, 
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 2.0};


    sleep(1);
    init_pub.publish( ini_pose );
    ROS_INFO("amcl init!");
    cout << "now_pose:( " << ini_pose.pose.pose.position.x << "," << ini_pose.pose.pose.position.y << " )" <<endl;
    for(int i = 0; i<36; ++i) 
      cout<< "covariance[ "<< i << " ]: " << ini_pose.pose.covariance[i] << endl;
    return true;
  }  


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_init");
  PoseInit PI;
  ros::spin();
  return 0;
}
