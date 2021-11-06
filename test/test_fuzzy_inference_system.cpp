#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <hubero_local_planner/geometry/geometry.h>
#include <hubero_local_planner/fuzz/processor.h>

#include <chrono>
#include <thread>

using namespace hubero_local_planner::geometry;
using namespace hubero_local_planner::fuzz;

// Helpers
static void calculateDynamicObjectData(
    const Pose& robot,
    const Pose& object,
    Angle& dir_object,
    Angle& rel_loc_object,
    Angle& dist_angle,
    double& dist
);
static void trimTermNameMarker(std::string& term_name);
static std::vector<std::string> createVectorOfBestTerms(const std::vector<std::tuple<std::string, double>>& mem);
static bool existsInVector(const std::vector<std::string>& v_str, const std::string& name);

struct FuzzyficationOutput {
    std::vector<std::string> highest_membership_terms;
    double highest_membership;
};
static std::tuple<FuzzyficationOutput, FuzzyficationOutput> fuzzyfy(
    Processor& fis,
    const Pose& robot,
    const Pose& object,
    bool log = false
);

// Test cases
TEST(FuzzyInferenceSystem, fuzzyfiedInputsMembership) {
    Processor fis;

    // Case 1
    Pose robot1(0.0, 0.0, 0.0);
    Pose object1(2.0, 2.0, IGN_PI_2);

    FuzzyficationOutput rel_loc1 {};
    FuzzyficationOutput dir_cross1 {};
    std::tie(rel_loc1, dir_cross1) = fuzzyfy(fis, robot1, object1);

    ASSERT_TRUE(existsInVector(rel_loc1.highest_membership_terms, "front_left"));
    ASSERT_TRUE(existsInVector(dir_cross1.highest_membership_terms, "outwards"));


    // Case 2
    Pose robot2(0.0, 0.0, 0.0);
    Pose object2(2.0, -2.0, IGN_PI_2);

    FuzzyficationOutput rel_loc2 {};
    FuzzyficationOutput dir_cross2 {};
    std::tie(rel_loc2, dir_cross2) = fuzzyfy(fis, robot2, object2);

    ASSERT_TRUE(existsInVector(rel_loc2.highest_membership_terms, "front_right"));
    ASSERT_TRUE(existsInVector(dir_cross2.highest_membership_terms, "cross_front"));


    // Case 3
    Pose robot3(0.0, 0.0, 0.0);
    Pose object3(-2.0, -2.0, IGN_PI_2);

    FuzzyficationOutput rel_loc3 {};
    FuzzyficationOutput dir_cross3 {};
    std::tie(rel_loc3, dir_cross3) = fuzzyfy(fis, robot3, object3);

    ASSERT_TRUE(existsInVector(rel_loc3.highest_membership_terms, "back_right"));
    ASSERT_TRUE(existsInVector(dir_cross3.highest_membership_terms, "cross_behind"));


    // Case 4
    Pose robot4(0.0, 0.0, 0.0);
    Pose object4(-2.0, +2.0, 0.0);

    FuzzyficationOutput rel_loc4 {};
    FuzzyficationOutput dir_cross4 {};
    std::tie(rel_loc4, dir_cross4) = fuzzyfy(fis, robot4, object4);

    ASSERT_TRUE(existsInVector(rel_loc4.highest_membership_terms, "back_left"));
    ASSERT_TRUE(existsInVector(dir_cross4.highest_membership_terms, "equal"));


    // Case 5
    Pose robot5 = Pose(0.0, 0.0, 0.0);
    Pose object5 = Pose(+2.0, 0.0, +IGN_PI);

    FuzzyficationOutput rel_loc5 {};
    FuzzyficationOutput dir_cross5 {};
    std::tie(rel_loc5, dir_cross5) = fuzzyfy(fis, robot5, object5);

    ASSERT_TRUE(existsInVector(rel_loc5.highest_membership_terms, "front"));
    ASSERT_TRUE(existsInVector(dir_cross5.highest_membership_terms, "opposite"));


    // Case 6
    Pose robot6 = Pose(0.0, 0.0, 0.0);
    Pose object6 = Pose(-2.0, 0.0, IGN_PI_4);

    FuzzyficationOutput rel_loc6 {};
    FuzzyficationOutput dir_cross6 {};
    std::tie(rel_loc6, dir_cross6) = fuzzyfy(fis, robot6, object6);

    ASSERT_TRUE(existsInVector(rel_loc6.highest_membership_terms, "back"));
    ASSERT_TRUE(existsInVector(dir_cross6.highest_membership_terms, "outwards"));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

// helpers
static void calculateDynamicObjectData(
    const Pose& robot,
    const Pose& object,
    Angle& dir_object,
    Angle& rel_loc_object,
    Angle& dist_angle,
    double& dist
) {
    // orientation is used as direction determinant
    dir_object = Angle(object.getYaw());

    // vector connecting positions
    Vector dist_v = Vector(object.getRawPosition() - robot.getRawPosition());
    dist_angle = Angle(dist_v);
    dist = dist_v.calculateLength();

    // relative location of object, compared to robot
    rel_loc_object = Angle(dist_angle.getRadian() - robot.getYaw());
}

static void trimTermNameMarker(std::string& term_name) {
    if (term_name.back() == 'A' || term_name.back() == 'B') {
        term_name.pop_back();
    }
}

static std::vector<std::string> createVectorOfBestTerms(const std::vector<std::tuple<std::string, double>>& mem) {
    std::vector<std::string> best_terms;
    for (const auto& term_mem : mem) {
        std::string term_name = std::get<0>(term_mem);
        trimTermNameMarker(term_name);
        best_terms.push_back(term_name);
    }
    return best_terms;
}

static bool existsInVector(const std::vector<std::string>& v_str, const std::string& name) {
    return std::find(std::begin(v_str), std::end(v_str), name) != std::end(v_str);
}

static std::tuple<FuzzyficationOutput, FuzzyficationOutput> fuzzyfy(Processor& fis, const Pose& robot, const Pose& object, bool log) {
    // outputs
    Angle dir_beta_dynamic;
    Angle rel_loc_dynamic;
    Angle dist_angle_dynamic;
    double dist_dynamic;
    calculateDynamicObjectData(robot, object, dir_beta_dynamic, rel_loc_dynamic, dist_angle_dynamic, dist_dynamic);

    fis.load(
        robot.getYaw(),
        std::vector<double>({dir_beta_dynamic.getRadian()}),
        std::vector<double>({rel_loc_dynamic.getRadian()}),
        std::vector<double>({dist_angle_dynamic.getRadian()})
    );
    fis.process();

    std::vector<std::tuple<std::string, double>> highest_mem_rel_loc;
    std::vector<std::tuple<std::string, double>> highest_mem_dir_cross;
    highest_mem_rel_loc = fis.membershipInputRelLoc();
    highest_mem_dir_cross = fis.membershipInputDirCross();
    std::vector<std::string> best_terms_rel_loc = createVectorOfBestTerms(highest_mem_rel_loc);
    std::vector<std::string> best_terms_dir_cross = createVectorOfBestTerms(highest_mem_dir_cross);

    if (log) {
        std::cout << "REL LOC   best: " << std::endl;
        for (const auto& entry : best_terms_rel_loc) {
            std::cout << "\t" << entry << std::endl;
        }
        std::cout << "DIR_CROSS best: " << std::endl;
        for (const auto& entry : best_terms_dir_cross) {
            std::cout << "\t" << entry << std::endl;
        }
    }

    FuzzyficationOutput output_rel_loc {};
    // arbitrary
    output_rel_loc.highest_membership = std::get<1>(highest_mem_rel_loc.at(0));
    output_rel_loc.highest_membership_terms = best_terms_rel_loc;

    FuzzyficationOutput output_dir_cross {};
    // arbitrary
    output_dir_cross.highest_membership = std::get<1>(highest_mem_dir_cross.at(0));
    output_dir_cross.highest_membership_terms = best_terms_dir_cross;
    return std::make_tuple(output_rel_loc, output_dir_cross);
}
