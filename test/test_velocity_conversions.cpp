
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <hubero_local_planner/hubero_planner_ros.h>
#include <hubero_common/typedefs.h>
#include <hubero_local_planner/utils/converter.h>

#include "gtest_cout.h"

using namespace hubero_local_planner;

TEST(HuberoVelocityConversions, computeVelocityGlobal) {
	// 1st case, robot orientation Euler Z 0 deg
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.w = 1.0;
	tf::Stamped<tf::Pose> pose_tf;
	tf::poseStampedMsgToTF(pose, pose_tf);

	tfScalar yaw, pitch, roll;
	tf::Matrix3x3 orientation1(pose_tf.getRotation());
	orientation1.getEulerYPR(yaw, pitch, roll);

	geometry_msgs::Twist vel_local;
	vel_local.linear.x = 0.15;
	vel_local.linear.y = 0.00;
	vel_local.angular.z = 0.25;

	geometry_msgs::Twist vel_global;
	HuberoPlannerROS::computeVelocityGlobal(vel_local, pose_tf, vel_global);
	GTEST_COUT << "1)          Pose2D: x " << pose_tf.getOrigin().x()  << " y " << pose_tf.getOrigin().y() << " yaw " << yaw << " pitch " << pitch << " roll " << roll << std::endl;
	GTEST_COUT << "1) Local  Velocity: x " << vel_local.linear.x  << " y " << vel_local.linear.y  << " z " << vel_local.angular.z  << std::endl;
	GTEST_COUT << "1) Global Velocity: x " << vel_global.linear.x << " y " << vel_global.linear.y << " z " << vel_global.angular.z << std::endl;
	ASSERT_NE(vel_local.angular.z, 0.0); // provides the next ASSERT to make any sense
	ASSERT_NE(yaw, vel_global.angular.z);

	// 2nd case: robot orientation Euler, Z 45 deg
	pose.pose.orientation.z = 0.3826834;
	pose.pose.orientation.w = 0.9238795;
	tf::poseStampedMsgToTF(pose, pose_tf);
	tf::Matrix3x3 orientation2(pose_tf.getRotation());
	orientation2.getEulerYPR(yaw, pitch, roll);
	HuberoPlannerROS::computeVelocityGlobal(vel_local, pose_tf, vel_global);
	GTEST_COUT << "2)          Pose2D: x " << pose_tf.getOrigin().x()  << " y " << pose_tf.getOrigin().y() << " yaw " << yaw << " pitch " << pitch << " roll " << roll << std::endl;
	GTEST_COUT << "2) Local  Velocity: x " << vel_local.linear.x  << " y " << vel_local.linear.y  << " z " << vel_local.angular.z  << std::endl;
	GTEST_COUT << "2) Global Velocity: x " << vel_global.linear.x << " y " << vel_global.linear.y << " z " << vel_global.angular.z << std::endl;
	ASSERT_NE(vel_local.angular.z, 0.0); // provides the next ASSERT to make any sense
	ASSERT_NE(yaw, vel_global.angular.z);

	// 3rd case: robot orientation Euler, Z -90 deg
	pose.pose.orientation.z = -0.7071068;
	pose.pose.orientation.w = +0.7071068;
	tf::poseStampedMsgToTF(pose, pose_tf);
	tf::Matrix3x3 orientation3(pose_tf.getRotation());
	orientation3.getEulerYPR(yaw, pitch, roll);
	HuberoPlannerROS::computeVelocityGlobal(vel_local, pose_tf, vel_global);
	GTEST_COUT << "3)          Pose2D: x " << pose_tf.getOrigin().x()  << " y " << pose_tf.getOrigin().y() << " yaw " << yaw << " pitch " << pitch << " roll " << roll << std::endl;
	GTEST_COUT << "3) Local  Velocity: x " << vel_local.linear.x  << " y " << vel_local.linear.y  << " z " << vel_local.angular.z  << std::endl;
	GTEST_COUT << "3) Global Velocity: x " << vel_global.linear.x << " y " << vel_global.linear.y << " z " << vel_global.angular.z << std::endl;
	ASSERT_NE(vel_local.angular.z, 0.0); // provides the next ASSERT to make any sense
	ASSERT_NE(yaw, vel_global.angular.z);

	// 4th case: robot orientation Euler, Z +90, local.angular.z negative
	vel_local.angular.z = -vel_local.angular.z;
	pose.pose.orientation.z = +0.7071068;
	pose.pose.orientation.w = +0.7071068;
	tf::poseStampedMsgToTF(pose, pose_tf);
	tf::Matrix3x3 orientation4(pose_tf.getRotation());
	orientation4.getEulerYPR(yaw, pitch, roll);
	HuberoPlannerROS::computeVelocityGlobal(vel_local, pose_tf, vel_global);
	GTEST_COUT << "4)          Pose2D: x " << pose_tf.getOrigin().x()  << " y " << pose_tf.getOrigin().y() << " yaw " << yaw << " pitch " << pitch << " roll " << roll << std::endl;
	GTEST_COUT << "4) Local  Velocity: x " << vel_local.linear.x  << " y " << vel_local.linear.y  << " z " << vel_local.angular.z  << std::endl;
	GTEST_COUT << "4) Global Velocity: x " << vel_global.linear.x << " y " << vel_global.linear.y << " z " << vel_global.angular.z << std::endl;
	ASSERT_NE(vel_local.angular.z, 0.0); // provides the next ASSERT to make any sense
	ASSERT_NE(yaw, vel_global.angular.z);
}

TEST(HuberoVelocityConversions, computeTwist) {
	// parameters
	const double SIM_PERIOD = 1.0; 	// time delta between acceleration calculation
	const double ROBOT_MASS = 1.0; 	// kg
	const double MIN_VEL_X = 0.0; 	// m/s
	const double MAX_VEL_X = 1.50; 	// m/s
	const double MAX_ROT_VEL = 2.0;	// rad/sec
	const double TWIST_ROT_COMPENSATION = 0.0;

	tf::Stamped<tf::Pose> pose_tf; // input
	Eigen::Vector3f force; // input
	geometry_msgs::Twist vel_local; // helper
	geometry_msgs::Twist vel_global; // input
	geometry_msgs::Twist cmd_vel; // output

	// local velocity
	vel_local.linear.x = 1.00;
	vel_local.linear.y = 0.00;
	vel_local.angular.z = 0.00; // 0.707;

	// 1st case, robot orientation Euler Z 0 deg, no force
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.w = 1.0;
	tf::poseStampedMsgToTF(pose, pose_tf);

	// no force - no movement (note that force does not sum up throught simulation)
	force[0] = 0.0;
	force[1] = 0.0;
	force[2] = 0.0;

	HuberoPlannerROS::computeVelocityGlobal(vel_local, pose_tf, vel_global);
	HuberoPlannerROS::computeTwist(pose_tf, force, vel_global, SIM_PERIOD, ROBOT_MASS, MIN_VEL_X, MAX_VEL_X, MAX_ROT_VEL, TWIST_ROT_COMPENSATION, cmd_vel);
	GTEST_COUT << "1) Global Velocity: x " << vel_global.linear.x << " y " << vel_global.linear.y << " z " << vel_global.angular.z << std::endl;
	GTEST_COUT << "1) Local  Velocity: x " << cmd_vel.linear.x  << " y " << cmd_vel.linear.y  << " z " << cmd_vel.angular.z  << std::endl;
	// force made robot maintain its speed
	EXPECT_FLOAT_EQ(cmd_vel.linear.x, 0.0);//vel_local.linear.x);
	EXPECT_FLOAT_EQ(cmd_vel.linear.y, 0.0);//vel_local.linear.y);
	EXPECT_FLOAT_EQ(cmd_vel.angular.z, 0.0);//vel_local.angular.z);

	// 2nd case, robot orientation Euler Z 0 deg, force along robot's front
	force[0] = 1.0;
	force[1] = 0.0;
	force[2] = 0.0;
	HuberoPlannerROS::computeVelocityGlobal(vel_local, pose_tf, vel_global);
	HuberoPlannerROS::computeTwist(pose_tf, force, vel_global, SIM_PERIOD, ROBOT_MASS, MIN_VEL_X, MAX_VEL_X, MAX_ROT_VEL, TWIST_ROT_COMPENSATION, cmd_vel);
	GTEST_COUT << "2) Global Velocity: x " << vel_global.linear.x << " y " << vel_global.linear.y << " z " << vel_global.angular.z << std::endl;
	GTEST_COUT << "2) Local  Velocity: x " << cmd_vel.linear.x  << " y " << cmd_vel.linear.y  << " z " << cmd_vel.angular.z  << std::endl;
	// force made robot maintain its previous speed (it's not complaint with physical laws, but we simplify things there)
	EXPECT_EQ(cmd_vel.linear.x, vel_local.linear.x);

	// 3rd case, robot orientation Euler Z 0 deg, force pointing to robot's front-left
	force[0] = 1.0;
	force[1] = 0.1;
	force[2] = 0.0;
	HuberoPlannerROS::computeVelocityGlobal(vel_local, pose_tf, vel_global);
	HuberoPlannerROS::computeTwist(pose_tf, force, vel_global, SIM_PERIOD, ROBOT_MASS, MIN_VEL_X, MAX_VEL_X, MAX_ROT_VEL, TWIST_ROT_COMPENSATION, cmd_vel);
	GTEST_COUT << "3) Global Velocity: x " << vel_global.linear.x << " y " << vel_global.linear.y << " z " << vel_global.angular.z << std::endl;
	GTEST_COUT << "3) Local  Velocity: x " << cmd_vel.linear.x  << " y " << cmd_vel.linear.y  << " z " << cmd_vel.angular.z  << std::endl;
	// force made robot maintain its speed
	EXPECT_EQ(cmd_vel.linear.x, vel_local.linear.x);
	// force made robot turn left a bit
	EXPECT_GT(cmd_vel.angular.z, vel_local.angular.z);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "HuberoVelocityConversions");
	ros::NodeHandle nh;
	ROSCONSOLE_AUTOINIT;
	ros::start();
	return RUN_ALL_TESTS();
}

