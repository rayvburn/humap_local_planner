#pragma once

#include <hubero_common/typedefs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Core>
#include <teb_local_planner/pose_se2.h>
#include <string>

namespace hubero_local_planner {

class Converter {
public:
    Converter() = default;

    static geometry_msgs::PoseStamped toPoseStamped(
			const Vector3 &pos,
			const std::string &frame = "world",
			bool zero_height = false
	) {
		geometry_msgs::PoseStamped pose_stamped;

		pose_stamped.header.frame_id = frame;
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.pose.position.x = pos.X();
		pose_stamped.pose.position.y = pos.Y();
		if ( !zero_height ) {
			pose_stamped.pose.position.z = pos.Z();
		} else {
			pose_stamped.pose.position.z = 0.0;
		}
		pose_stamped.pose.orientation.x = 0.0f;
		pose_stamped.pose.orientation.y = 0.0f;
		pose_stamped.pose.orientation.z = 0.0f;
		pose_stamped.pose.orientation.w = 1.0f;

		return (pose_stamped);
	}

    static geometry_msgs::PoseStamped toPoseStamped(
			const Pose3 &pose,
			const std::string &frame = "world",
			bool zero_height = false
	) {
		geometry_msgs::PoseStamped pose_stamped;

		pose_stamped.header.frame_id = frame;
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.pose.position.x = pose.Pos().X();
		pose_stamped.pose.position.y = pose.Pos().Y();
		if ( !zero_height ) {
			pose_stamped.pose.position.z = pose.Pos().Z();
		} else {
			pose_stamped.pose.position.z = 0.0;
		}
		pose_stamped.pose.orientation.x = pose.Rot().X();
		pose_stamped.pose.orientation.y = pose.Rot().Y();
		pose_stamped.pose.orientation.z = pose.Rot().Z();
		pose_stamped.pose.orientation.w = pose.Rot().W();

		return (pose_stamped);
	}

    static Vector3 toIgnVector(const geometry_msgs::PoseStamped &pose) {
		Vector3 pos;

		pos.X(pose.pose.position.x);
		pos.Y(pose.pose.position.y);
		pos.Z(pose.pose.position.z);

		return (pos);
	}

	static Pose3 toIgnPose(const geometry_msgs::PoseStamped &pose) {
		Pose3 pose_ign;

		pose_ign.Pos().X(pose.pose.position.x);
		pose_ign.Pos().Y(pose.pose.position.y);
		pose_ign.Pos().Z(pose.pose.position.z);

		pose_ign.Rot().X(pose.pose.orientation.x);
		pose_ign.Rot().Y(pose.pose.orientation.y);
		pose_ign.Rot().Z(pose.pose.orientation.z);
		pose_ign.Rot().W(pose.pose.orientation.w);

		return (pose_ign);
	}

	/// \brief Helper function for conversion from ignition's Pose to geometry_msgs' TransformStamped
	static geometry_msgs::TransformStamped toTfStamped(
			const Pose3 &pose,
			bool zero_height = false
	) {
		geometry_msgs::TransformStamped tf_stamp;

		tf_stamp.transform.translation.x = pose.Pos().X();
		tf_stamp.transform.translation.y = pose.Pos().Y();
		if ( zero_height ) {
			tf_stamp.transform.translation.z = 0.0;
		} else {
			tf_stamp.transform.translation.z = pose.Pos().Z();
		}

		tf_stamp.transform.rotation.x = pose.Rot().X();
		tf_stamp.transform.rotation.y = pose.Rot().Y();
		tf_stamp.transform.rotation.z = pose.Rot().Z();
		tf_stamp.transform.rotation.w = pose.Rot().W();

		return (tf_stamp);
	}

	static Pose3 toIgnPose(const Vector3 &pos, const Quaternion &quat = Quaternion(0.0, 0.0, 0.0)) {
		Pose3 pose;
		pose.Pos() = pos;
		pose.Rot() = quat;
		return (pose);
	}

	static Pose3 toIgnPose(const tf::Stamped<tf::Pose>& tf_pose) {
		Pose3 pose_ign;

		pose_ign.Pos().X(tf_pose.getOrigin().x());
		pose_ign.Pos().Y(tf_pose.getOrigin().y());
		pose_ign.Pos().Z(tf_pose.getOrigin().z());

		pose_ign.Rot().X(tf_pose.getRotation().x());
		pose_ign.Rot().Y(tf_pose.getRotation().y());
		pose_ign.Rot().Z(tf_pose.getRotation().z());
		pose_ign.Rot().W(tf_pose.getRotation().w());

		return (pose_ign);
	}

	static Vector3 toIgnVector(const tf::Stamped<tf::Pose>& tf_pose) {
		return Vector3(tf_pose.getOrigin().x(), tf_pose.getOrigin().y(), tf_pose.getOrigin().z());
	}

	static Vector3 toIgnVector(const geometry_msgs::Twist& vel) {
		return Vector3(vel.linear.x, vel.linear.y, vel.angular.z);
	}

	static Vector3 toIgnVector(const Eigen::Vector3f& v) {
		return Vector3(v[0], v[1], v[2]);
	}

    template <typename T>
	static T toEigenV3(const Vector3& v) {
		T v_out;
		v_out[0] = v.X();
		v_out[1] = v.Y();
		v_out[2] = v.Z();
		return v_out;
	}

	static teb_local_planner::PoseSE2 toTebPose(const tf::Stamped<tf::Pose>& tf_pose) {
		teb_local_planner::PoseSE2 pose_se2(
				tf_pose.getOrigin().x(),
				tf_pose.getOrigin().y(),
				tf_pose.getRotation().z() // TODO:: fixme
		);
		return pose_se2;
	}

	static teb_local_planner::PoseSE2 toTebPose(const Pose3& pose) {
		return teb_local_planner::PoseSE2(pose.Pos().X(), pose.Pos().Y(), pose.Rot().Yaw());
	}

}; // class Converter
} // namespace hubero_local_planner


/*
geometry_msgs::PoseStamped ignVectorToPoseStamped(
			const Vector3 &pos,
			const std::string &frame = "world",
			bool zero_height = false
	) {
		geometry_msgs::PoseStamped pose_stamped;

		pose_stamped.header.frame_id = frame;
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.pose.position.x = pos.X();
		pose_stamped.pose.position.y = pos.Y();
		if ( !zero_height ) {
			pose_stamped.pose.position.z = pos.Z();
		} else {
			pose_stamped.pose.position.z = 0.0;
		}
		pose_stamped.pose.orientation.x = 0.0f;
		pose_stamped.pose.orientation.y = 0.0f;
		pose_stamped.pose.orientation.z = 0.0f;
		pose_stamped.pose.orientation.w = 1.0f;

		return (pose_stamped);
	}

	geometry_msgs::PoseStamped ignPoseToPoseStamped(
			const Pose3 &pose,
			const std::string &frame = "world",
			bool zero_height = false
	) {
		geometry_msgs::PoseStamped pose_stamped;

		pose_stamped.header.frame_id = frame;
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.pose.position.x = pose.Pos().X();
		pose_stamped.pose.position.y = pose.Pos().Y();
		if ( !zero_height ) {
			pose_stamped.pose.position.z = pose.Pos().Z();
		} else {
			pose_stamped.pose.position.z = 0.0;
		}
		pose_stamped.pose.orientation.x = pose.Rot().X();
		pose_stamped.pose.orientation.y = pose.Rot().Y();
		pose_stamped.pose.orientation.z = pose.Rot().Z();
		pose_stamped.pose.orientation.w = pose.Rot().W();

		return (pose_stamped);
	}

	Vector3 poseStampedToIgnVector(const geometry_msgs::PoseStamped &pose) {
		Vector3 pos;

		pos.X(pose.pose.position.x);
		pos.Y(pose.pose.position.y);
		pos.Z(pose.pose.position.z);

		return (pos);
	}

	Pose3 poseStampedToIgnPose(const geometry_msgs::PoseStamped &pose) {
		Pose3 pose_ign;

		pose_ign.Pos().X(pose.pose.position.x);
		pose_ign.Pos().Y(pose.pose.position.y);
		pose_ign.Pos().Z(pose.pose.position.z);

		pose_ign.Rot().X(pose.pose.orientation.x);
		pose_ign.Rot().Y(pose.pose.orientation.y);
		pose_ign.Rot().Z(pose.pose.orientation.z);
		pose_ign.Rot().W(pose.pose.orientation.w);

		return (pose_ign);
	}

	/// \brief Helper function for conversion from ignition's Pose to geometry_msgs' TransformStamped
	geometry_msgs::TransformStamped ignPoseToTfStamped(
			const Pose3 &pose,
			bool zero_height = false
	) {
		geometry_msgs::TransformStamped tf_stamp;

		tf_stamp.transform.translation.x = pose.Pos().X();
		tf_stamp.transform.translation.y = pose.Pos().Y();
		if ( zero_height ) {
			tf_stamp.transform.translation.z = 0.0;
		} else {
			tf_stamp.transform.translation.z = pose.Pos().Z();
		}

		tf_stamp.transform.rotation.x = pose.Rot().X();
		tf_stamp.transform.rotation.y = pose.Rot().Y();
		tf_stamp.transform.rotation.z = pose.Rot().Z();
		tf_stamp.transform.rotation.w = pose.Rot().W();

		return (tf_stamp);
	}

	Pose3 ignVectorToPose(const Vector3 &pos, const Quaternion &quat = Quaternion(0.0, 0.0, 0.0)) {
		Pose3 pose;
		pose.Pos() = pos;
		pose.Rot() = quat;
		return (pose);
	}

	Pose3 tfPoseToIgnPose(const tf::Stamped<tf::Pose>& tf_pose) {
		Pose3 pose_ign;

		pose_ign.Pos().X(tf_pose.getOrigin().x());
		pose_ign.Pos().Y(tf_pose.getOrigin().y());
		pose_ign.Pos().Z(tf_pose.getOrigin().z());

		pose_ign.Rot().X(tf_pose.getRotation().x());
		pose_ign.Rot().Y(tf_pose.getRotation().y());
		pose_ign.Rot().Z(tf_pose.getRotation().z());
		pose_ign.Rot().W(tf_pose.getRotation().w());

		return (pose_ign);
	}

	Vector3 tfPoseToIgnVector(const tf::Stamped<tf::Pose>& tf_pose) {
		return Vector3(tf_pose.getOrigin().x(), tf_pose.getOrigin().y(), tf_pose.getOrigin().z());
	}

	Vector3 twistToIgnVector3(const geometry_msgs::Twist& vel) {
		return Vector3(vel.linear.x, vel.linear.y, vel.angular.z);
	}

	Vector3 eigenV3fToIgnVector(const Eigen::Vector3f& v) {
		return Vector3(v[0], v[1], v[2]);
	}

	Eigen::Vector3f ignVectorToEigenV3f(const Vector3& v) {
		Eigen::Vector3f v_out;
		v_out[0] = v.X();
		v_out[1] = v.Y();
		v_out[2] = v.Z();
		return v_out;
	}

	teb_local_planner::PoseSE2 tfPoseToPoseSE2(const tf::Stamped<tf::Pose>& tf_pose) {
		teb_local_planner::PoseSE2 pose_se2(
				tf_pose.getOrigin().x(),
				tf_pose.getOrigin().y(),
				tf_pose.getRotation().z() // TODO:: fixme
		);
		return pose_se2;
	}

	teb_local_planner::PoseSE2 ignPoseToPoseSE2(const Pose3& pose) {
		return teb_local_planner::PoseSE2(pose.Pos().X(), pose.Pos().Y(), pose.Rot().Yaw());
	}
*/