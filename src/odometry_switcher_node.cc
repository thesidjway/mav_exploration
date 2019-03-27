#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>


Eigen::Matrix4d odomMsgToEigenMatrix(const nav_msgs::Odometry& odom)
{
	Eigen::Affine3d eigenTr;
	tf::poseMsgToEigen(odom.pose.pose, eigenTr);
	return eigenTr.matrix().cast<double>();
}

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    odom_pub_ = n_.advertise<nav_msgs::Odometry>("/odometry_switched", 1000);

    //Topic you want to subscribe
    vins_sub_ = n_.subscribe("/firefly/vins_node/odometry", 1, &SubscribeAndPublish::vins_callback, this);
    gt_sub_ = n_.subscribe("/firefly/odometry_sensor1/odometry", 1, &SubscribeAndPublish::gt_callback, this);
  }

  void checkReadyForVins(const nav_msgs::Odometry& input) {
  	//if (input.pose.pose.position.z >= 0.7) {
  	//	ready_for_vins_ = true;
  	//}
  	ready_for_vins_ = false;
  }

  void vins_callback(const nav_msgs::Odometry& input)
  {
  	if (ready_for_vins_) {
  		if (!callback_unreg) {
  			Eigen::Matrix4d a = odomMsgToEigenMatrix(last_gt_msg_);
  			Eigen::Matrix4d b = odomMsgToEigenMatrix(input);
  			gt_sub_.shutdown();
  			callback_unreg = true;
  		}
    	odom_pub_.publish(input);
    }
  }

  void gt_callback(const nav_msgs::Odometry& input)
  {
  	checkReadyForVins(input);
  	ros::Duration time_diff = ros::Time::now() - last_pub_time_;

  	if (!ready_for_vins_ && time_diff.toSec() >=  0.07) {
    	odom_pub_.publish(input);
    	last_gt_msg_ = input;
    	last_pub_time_ = ros::Time::now();
    }
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher odom_pub_;
  ros::Subscriber vins_sub_;
  ros::Subscriber gt_sub_;
  bool ready_for_vins_ = false;
  bool callback_unreg = false;
  ros::Time last_pub_time_ = ros::Time::now();
  nav_msgs::Odometry last_gt_msg_;

};


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_switcher");
  SubscribeAndPublish s_p;
  ros::spin();
}


