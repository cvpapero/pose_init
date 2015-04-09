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
  ros::NodeHandle nh_p;
  ros::Subscriber amcl_sub;
  ros::Publisher init_pub;
  ros::ServiceServer init_srv;
  ros::ServiceClient clear_costmap_local_clt;

  double sig_param;
  double sig_deg_param;

public:
  PoseInit():
    nh_p("~"),
    sig_param(0.2),
    sig_deg_param(90)
  {
    nh_p.param("sigma", sig_param, sig_param);
    nh_p.param("sigma_deg", sig_deg_param, sig_deg_param);

    amcl_sub 
      = nh.subscribe("amcl_pose", 1, &PoseInit::callback, this);
    init_pub 
      = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    init_srv 
      = nh.advertiseService("pose_init_srv", &PoseInit::sendData, this); 

    clear_costmap_local_clt 
      = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmap_local");
  }

  void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
  {
    ini_pose = *pose;
  }  

  bool sendData(std_srvs::Empty::Request& req,
		std_srvs::Empty::Response& res)
  {
    std_srvs::Empty emp;

    double sigma = sig_param;
    double rad_sigma = (double)sig_deg_param*(M_PI/180.);

    ini_pose.pose.covariance = {sigma * sigma, 0.0, 0.0, 0.0, 0.0, 0.0, 
				0.0, sigma * sigma, 0.0, 0.0, 0.0, 0.0, 
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, rad_sigma * rad_sigma};

   
    sleep(1);
    init_pub.publish( ini_pose );
    ROS_INFO("amcl init!");
    cout << "now_sigma:( "
	 << sigma
	 << ","
	 << sig_deg_param
	 << "<rad: "
	 << rad_sigma
	 << "> ), now_pose:( " 
	 << ini_pose.pose.pose.position.x 
	 << "," 
	 << ini_pose.pose.pose.position.y 
	 << " )" 
	 << endl;

    clear_costmap_local_clt.call( emp );

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
