#pragma once

#include <people_msgs/People.h>

#include <hubero_local_planner/geometry/pose.h>
#include <hubero_local_planner/geometry/vector.h>
#include <hubero_local_planner/geometry/quaternion.h>

#include <string>

namespace hubero_local_planner {

class Person {
public:
	/**
	 * @brief Basic constructor from people_msgs/Person
	 */
	Person(const people_msgs::Person& person);

	/**
	 * @brief Basic constructor from people_msgs/Person contents
	 */
	Person(
		const std::string& name,
		const geometry_msgs::Point& position,
		const geometry_msgs::Point& velocity,
		const double& reliability,
		const std::vector<std::string>& tagnames,
		const std::vector<std::string>& tags
	);

	/**
	 * @brief Transforms person pose
	 */
	void transform(const geometry::Pose transform);

	inline std::string getName() const {
		return name_;
	}

	inline unsigned int getID() const {
		return std::stoul(name_);
	}

	inline geometry::Pose getPose() const {
		return pose_;
	}

	inline double getReliability() const {
		return reliability_;
	}

	inline geometry::Vector getVelocity() const {
		return vel_;
	}

	inline bool isOccluded() const {
		return occluded_;
	}

	inline bool isMatched() const {
		return matched_;
	}

	inline unsigned int getDetectionID() const {
		return detection_id_;
	}

	inline unsigned int getTrackAge() const {
		return track_age_;
	}

	inline std::string getGroupName() const {
		return group_id_;
	}

	inline unsigned int getGroupID() const {
		return std::stoul(group_id_);
	}

	inline unsigned int getGroupAge() const {
		return group_age_;
	}

	inline std::vector<unsigned int> getGroupTrackIDs() const {
		return group_track_ids_;
	}

	inline geometry::Vector getGroupCenterOfGravity() const {
		return group_center_of_gravity_;
	}

	/**
	 * @brief Parses string containing a set of T-type values
	 *
	 * @tparam T type of values
	 */
	template <typename T>
	static std::vector<T> parseString(const std::string& str, const std::string& delimiter) {
		std::vector<T> values;
		std::string payload(str);

		if (str.empty() || delimiter.empty()) {
			return values;
		}

		// https://stackoverflow.com/a/14266139
		size_t pos = 0;
		std::string token;
		while ((pos = payload.find(delimiter)) != std::string::npos) {
			token = payload.substr(0, pos);
			// convert with the biggest possible precision, then convert to desired type
			values.push_back(static_cast<T>(std::stod(token)));
			payload.erase(0, pos + delimiter.length());
		}

		bool token_with_whitespace_only = payload.find_first_not_of("\t\n ") == std::string::npos;
		if (payload.empty() || token_with_whitespace_only) {
			return values;
		}

		// token still stores some meaningful value
		values.push_back(static_cast<T>(std::stod(payload)));
	}

	static bool parseStringBool(const std::string& str);

protected:
	/**
	 * @brief Returns true if tagnames and tags are valid, no matter if expected data were found inside
	 */
	bool parseTags(const std::vector<std::string>& tagnames, const std::vector<std::string>& tags);

	/// Person ID (number) is treated as name
	std::string name_;
	/// 2D pose (x, y, theta)
	geometry::Pose pose_;
	/// Defines accuracy of person's pose and velocity
	double reliability_;
	/// 2D velocity (x, y + theta)
	geometry::Vector vel_;

	bool occluded_;
	/// Whether person is currently matched by perception system
	bool matched_;

	/// Detection ID of the person
	unsigned int detection_id_;
	/// How long this person has been tracked
	unsigned int track_age_;

	/// ID of the group that this person was classified to
	std::string group_id_;
	/// How long person's group has been tracked
	unsigned int group_age_;
	/// IDs of other people classified to the same group
	std::vector<unsigned int> group_track_ids_;
	/// Position of the group's center of gravity
	geometry::Vector group_center_of_gravity_;

}; // class Person

//! Abbrev. for shared pointers
typedef std::shared_ptr<Person> PersonPtr;
//! Abbrev. for shared const pointers
typedef std::shared_ptr<const Person> PersonConstPtr;
//! Abbrev. for containers storing multiple pointers to objects
typedef std::vector<PersonPtr> PeoplePtrContainer;
//! Abbrev. for containers storing multiple objects
typedef std::vector<Person> PeopleContainer;

// Container
typedef std::shared_ptr<PeopleContainer> PeopleContainerPtr;
typedef std::shared_ptr<const PeopleContainer> PeopleContainerConstPtr;

} // namespace hubero_local_planner
