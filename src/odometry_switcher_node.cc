#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>



// From https://github.com/ANYbotics/pointmatcher-ros/blob/master/pointmatcher_ros/src/transform.cpp

Eigen::Matrix4d odomMsgToEigenMatrix(const nav_msgs::Odometry& odom) {
	Eigen::Affine3d eigenTr;
	tf::poseMsgToEigen(odom.pose.pose, eigenTr);
	return eigenTr.matrix().cast<double>();
}

Eigen::Matrix4d eigenMatrixToDim(const Eigen::Matrix4d& matrix, int dimp1) {
    assert(matrix.rows() == matrix.cols());
    assert((matrix.rows() == 3) || (matrix.rows() == 4));
    assert((dimp1 == 3) || (dimp1 == 4));
    
    if (matrix.rows() == dimp1)
      return matrix;
    
    Eigen::Matrix4d out(Eigen::Matrix4d::Identity(dimp1,dimp1));
    out.topLeftCorner(2,2) = matrix.topLeftCorner(2,2);
    out.topRightCorner(2,1) = matrix.topRightCorner(2,1);
    return out;
}

nav_msgs::Odometry eigenMatrixToOdomMsg(const Eigen::Matrix4d& inTr, const std::string& frame_id, const ros::Time& stamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id;
    
    // Fill pose
    const Eigen::Affine3d eigenTr(Eigen::Matrix4d(eigenMatrixToDim(inTr, 4)));
    tf::poseEigenToMsg(eigenTr, odom.pose.pose);

    // Fill velocity, TODO: find proper computation from delta poses to twist
    //odom.child_frame_id = cloudMsgIn.header.frame_id;
    odom.twist.covariance[0+0*6] = 0;
    odom.twist.covariance[1+1*6] = 0;
    odom.twist.covariance[2+2*6] = 0;
    odom.twist.covariance[3+3*6] = 0;
    odom.twist.covariance[4+4*6] = 0;
    odom.twist.covariance[5+5*6] = 0;

    return odom;
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
    imu_sub_ = n_.subscribe("/firefly/imu", 1, &SubscribeAndPublish::imu_callback, this);
  }

  void checkReadyForVins(const nav_msgs::Odometry& input) {
  	if (input.pose.pose.position.z >= 0.7) {
  		ready_for_vins_ = true;
  	}
  	//ready_for_vins_ = false;
  }

  void vins_callback(const nav_msgs::Odometry& vins_odom) {
  	if (ready_for_vins_) {
        Eigen::Matrix4d vins_in_world = odomMsgToEigenMatrix(vins_odom);
  		if (!callback_unreg) {
  			Eigen::Matrix4d gt_in_world = odomMsgToEigenMatrix(last_gt_msg_);
        gt_in_vins = vins_in_world.inverse() * gt_in_world;
  			gt_sub_.shutdown();
  			callback_unreg = true;
  		}
        Eigen::Matrix4d gt_in_world_transformed = vins_in_world * gt_in_vins;
        nav_msgs::Odometry modified_odom = eigenMatrixToOdomMsg(gt_in_world_transformed, vins_odom.header.frame_id, vins_odom.header.stamp);
        odom_pub_.publish(modified_odom);
    }
  }

  void imu_callback(const sensor_msgs::Imu& imu_msg) {
  	last_angular_vel_ = Eigen::Vector3d(imu_msg.angular_velocity.x,
  		imu_msg.angular_velocity.y,
  		imu_msg.angular_velocity.z);
  }

  void gt_callback(const nav_msgs::Odometry& gt_msg) {
  	checkReadyForVins(gt_msg);
  	ros::Duration time_diff = ros::Time::now() - last_pub_time_;
    last_gt_msg_ = gt_msg;
  	if (!ready_for_vins_ && time_diff.toSec() >=  0.07) {
    	odom_pub_.publish(gt_msg);
    	last_pub_time_ = ros::Time::now();
    }
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher odom_pub_;
  ros::Subscriber vins_sub_;
  ros::Subscriber gt_sub_;
  ros::Subscriber imu_sub_;
  Eigen::Vector3d last_angular_vel_;
  bool ready_for_vins_ = false;
  bool callback_unreg = false;
  ros::Time last_pub_time_ = ros::Time::now();
  nav_msgs::Odometry last_gt_msg_;
  Eigen::Matrix4d gt_in_vins;

};


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_switcher");
  SubscribeAndPublish s_p;
  ros::spin();
}


