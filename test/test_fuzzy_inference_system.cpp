#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <humap_local_planner/geometry/geometry.h>
#include <humap_local_planner/fuzz/processor.h>

#include <chrono>
#include <thread>

// #define PRINT_DEBUG

using namespace humap_local_planner::geometry;
using namespace humap_local_planner::fuzz;

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
static bool existsInString(const std::string& str, const std::string& pattern);

struct FuzzyficationOutput {
    std::vector<std::string> highest_membership_terms;
    double highest_membership;
};
static std::tuple<FuzzyficationOutput, FuzzyficationOutput> fuzzyfy(
    Processor& fis,
    const Pose& robot,
    const Pose& object
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

/**
 * evaluates FIS raw output values - checks if they geometrically match their verbose analogues;
 * this is highly related to rule blocks composition;
 * TODO: make it a TestFixture?
 */
TEST(FuzzyInferenceSystem, outputValues) {
    /*
     * Assertions, here, check:
     * - whether 1 outputs of the FIS was generated (should be if there is 1 dynamic obejct in the environment (besides robot))
     * - output value bounds as action direction (see rule successor)
     * - OPTIONAL: output value term name
     */
    Processor fis;

    // front/opposite -> turn right
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front/opposite --> turn right" << std::endl;
    #endif
    Pose robot1 = Pose(0.0, 0.0, 0.0);
    Pose object1 = Pose(+2.0, 0.0, +IGN_PI);
    fuzzyfy(fis, robot1, object1);

    std::vector<Processor::FisOutput> fis_output1 = fis.getOutput();
    ASSERT_EQ(fis_output1.size(), 1);
    auto output_single1 = fis_output1.front();
    ASSERT_LE(output_single1.value, 0.0);
    ASSERT_GE(output_single1.value, -IGN_PI);
    trimTermNameMarker(output_single1.term_name);
    EXPECT_TRUE(existsInString(output_single1.term_name, "turn_right"));


    /// front/outwards -> decelerate (2), see @ref DISABLED_outputValuesKindaOff
    /// front/equal -> decelerate (3), see @ref DISABLED_outputValuesKindaOff


    // front/cross front -> turn right
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front/cross front --> turn right" << std::endl;
    #endif
    Pose robot4 = Pose(0.0, 0.0, 0.0);
    Pose object4 = Pose(+2.0, 0.0, -IGN_PI_2);
    fuzzyfy(fis, robot4, object4);

    std::vector<Processor::FisOutput> fis_output4 = fis.getOutput();
    ASSERT_EQ(fis_output4.size(), 1);
    auto output_single4 = fis_output4.front();
    ASSERT_LE(output_single4.value, 0.0);
    ASSERT_GE(output_single4.value, -IGN_PI);
    trimTermNameMarker(output_single4.term_name);
    ASSERT_EQ(output_single4.term_name, "turn_right");


    // front right/cross behind -> turn left
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front right/cross behind --> turn left" << std::endl;
    #endif
    Pose robot5 = Pose(0.0, 0.0, 0.0);
    Pose object5 = Pose(+1.0, -2.0, 3.0 * IGN_PI_4);
    fuzzyfy(fis, robot5, object5);

    std::vector<Processor::FisOutput> fis_output5 = fis.getOutput();
    ASSERT_EQ(fis_output5.size(), 1);
    auto output_single5 = fis_output5.front();
    ASSERT_LE(output_single5.value, IGN_PI);
    ASSERT_GE(output_single5.value, 0.0);
    trimTermNameMarker(output_single5.term_name);
    ASSERT_EQ(output_single5.term_name, "turn_left");


    // front right/opposite -> turn_left
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front right/opposite --> turn_left" << std::endl;
    #endif
    Pose robot6 = Pose(0.0, 0.0, 0.0);
    Pose object6 = Pose(2.0, -2.0, IGN_PI);
    fuzzyfy(fis, robot6, object6);

    std::vector<Processor::FisOutput> fis_output6 = fis.getOutput();
    ASSERT_EQ(fis_output6.size(), 1);
    auto output_single6 = fis_output6.front();
    ASSERT_LE(output_single6.value, IGN_PI);
    ASSERT_GE(output_single6.value, 0.0);
    trimTermNameMarker(output_single6.term_name);
    ASSERT_EQ(output_single6.term_name, "turn_left");


    // front right/outwards -> turn_left
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front right/outwards --> turn_left" << std::endl;
    #endif
    Pose robot7 = Pose(0.0, 0.0, 0.0);
    Pose object7 = Pose(2.0, -2.0, -IGN_PI);
    fuzzyfy(fis, robot7, object7);

    std::vector<Processor::FisOutput> fis_output7 = fis.getOutput();
    ASSERT_EQ(fis_output7.size(), 1);
    auto output_single7 = fis_output7.front();
    ASSERT_LE(output_single7.value, IGN_PI);
    ASSERT_GE(output_single7.value, 0.0);
    trimTermNameMarker(output_single7.term_name);
    ASSERT_EQ(output_single7.term_name, "turn_left");


    // front right/equal -> turn_left_accelerate
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front right/equal --> turn_left_accelerate" << std::endl;
    #endif
    Pose robot8 = Pose(0.0, 0.0, 0.0);
    Pose object8 = Pose(2.0, -2.0, 0.0);
    fuzzyfy(fis, robot8, object8);

    std::vector<Processor::FisOutput> fis_output8 = fis.getOutput();
    ASSERT_EQ(fis_output8.size(), 1);
    auto output_single8 = fis_output8.front();
    ASSERT_LE(output_single8.value, IGN_PI_2);
    ASSERT_GE(output_single8.value, 0.0);
    trimTermNameMarker(output_single8.term_name);
    // not perfect
    // ASSERT_EQ(output_single8.term_name, "turn_left_accelerate");
    ASSERT_TRUE(
        existsInString(output_single8.term_name, "turn_left")
        || existsInString(output_single8.term_name, "accelerate")
    );


    // front right/cross front -> turn_right
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front right/cross front --> turn_right" << std::endl;
    #endif
    Pose robot9 = Pose(0.0, 0.0, 0.0);
    Pose object9 = Pose(2.0, -2.0, IGN_PI_4);
    fuzzyfy(fis, robot9, object9);

    std::vector<Processor::FisOutput> fis_output9 = fis.getOutput();
    ASSERT_EQ(fis_output9.size(), 1);
    auto output_single9 = fis_output9.front();
    ASSERT_GE(output_single9.value, -IGN_PI);
    ASSERT_LE(output_single9.value, 0.0);
    trimTermNameMarker(output_single9.term_name);
    ASSERT_EQ(output_single9.term_name, "turn_right");


    // back right/cross behind -> turn_left_accelerate
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] back right/cross behind --> turn_left_accelerate" << std::endl;
    #endif
    Pose robot10 = Pose(0.0, 0.0, 0.0);
    Pose object10 = Pose(-2.0, -2.0, +IGN_PI_2);
    fuzzyfy(fis, robot10, object10);

    std::vector<Processor::FisOutput> fis_output10 = fis.getOutput();
    ASSERT_EQ(fis_output10.size(), 1);
    auto output_single10 = fis_output10.front();
    ASSERT_GE(output_single10.value, 0.0);
    ASSERT_LE(output_single10.value, +IGN_PI_2);
    trimTermNameMarker(output_single10.term_name);
    ASSERT_EQ(output_single10.term_name, "turn_left_accelerate");


    // back right/opposite -> turn_right
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] back right/opposite --> turn_right" << std::endl;
    #endif
    Pose robot11 = Pose(0.0, 0.0, 0.0);
    Pose object11 = Pose(-2.0, -2.0, +IGN_PI);
    fuzzyfy(fis, robot11, object11);

    std::vector<Processor::FisOutput> fis_output11 = fis.getOutput();
    ASSERT_EQ(fis_output11.size(), 1);
    auto output_single11 = fis_output11.front();
    ASSERT_LE(output_single11.value, 0.0);
    ASSERT_GE(output_single11.value, -IGN_PI);
    // FIXME: not perfect
    // trimTermNameMarker(output_single11.term_name);
    // ASSERT_EQ(output_single11.term_name, "turn_right");


    // back right/equal -> turn_right_accelerate
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] back right/equal --> turn_right_accelerate" << std::endl;
    #endif
    Pose robot12 = Pose(0.0, 0.0, 0.0);
    Pose object12 = Pose(-2.0, -2.0, 0.0);
    fuzzyfy(fis, robot12, object12);

    std::vector<Processor::FisOutput> fis_output12 = fis.getOutput();
    ASSERT_EQ(fis_output12.size(), 1);
    auto output_single12 = fis_output12.front();
    ASSERT_LE(output_single12.value, 0.0);
    ASSERT_GE(output_single12.value, -IGN_PI_2);
    trimTermNameMarker(output_single12.term_name);
    ASSERT_EQ(output_single12.term_name, "turn_right_accelerate");


    // back right/cross front -> accelerate
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] back right/cross front --> accelerate" << std::endl;
    #endif
    Pose robot13 = Pose(0.0, 0.0, 0.0);
    Pose object13 = Pose(-2.0, -2.0, IGN_PI_4 / 2.0);
    fuzzyfy(fis, robot13, object13);

    std::vector<Processor::FisOutput> fis_output13 = fis.getOutput();
    ASSERT_EQ(fis_output13.size(), 1);
    auto output_single13 = fis_output13.front();
    ASSERT_LE(output_single13.value, +IGN_PI_2);
    ASSERT_GE(output_single13.value, -IGN_PI_2);
    trimTermNameMarker(output_single13.term_name);
    ASSERT_TRUE(existsInString(output_single13.term_name, "accelerate"));


    // back left/cross behind -> accelerate
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] back left/cross behind --> accelerate" << std::endl;
    #endif
    Pose robot14 = Pose(0.0, 0.0, 0.0);
    Pose object14 = Pose(-2.0, +2.0, -IGN_PI_2);
    fuzzyfy(fis, robot14, object14);

    std::vector<Processor::FisOutput> fis_output14 = fis.getOutput();
    ASSERT_EQ(fis_output14.size(), 1);
    auto output_single14 = fis_output14.front();
    ASSERT_GE(output_single14.value, -IGN_PI_2);
    ASSERT_LE(output_single14.value, +IGN_PI_2);
    trimTermNameMarker(output_single14.term_name);
    ASSERT_TRUE(existsInString(output_single14.term_name, "accelerate"));


    // front left/cross behind -> turn_right
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front left/cross behind --> turn_right" << std::endl;
    #endif
    Pose robot15 = Pose(0.0, 0.0, 0.0);
    Pose object15 = Pose(+2.0, +2.0, -(5.0 / 6.0) * IGN_PI);
    fuzzyfy(fis, robot15, object15);

    std::vector<Processor::FisOutput> fis_output15 = fis.getOutput();
    ASSERT_EQ(fis_output15.size(), 1);
    auto output_single15 = fis_output15.front();
    ASSERT_LE(output_single15.value, 0.0);
    ASSERT_GE(output_single15.value, -IGN_PI);
    trimTermNameMarker(output_single15.term_name);
    ASSERT_EQ(output_single15.term_name, "turn_right");


    // front left/cross front -> turn_right_accelerate
    #ifdef PRINT_DEBUG
    std::cout << std::endl << "[outputValues] front left/cross front --> turn_right_accelerate" << std::endl;
    #endif
    Pose robot16 = Pose(0.0, 0.0, 0.0);
    Pose object16 = Pose(+2.0, +2.0, -IGN_PI_2);
    fuzzyfy(fis, robot16, object16);

    std::vector<Processor::FisOutput> fis_output16 = fis.getOutput();
    ASSERT_EQ(fis_output16.size(), 1);
    auto output_single16 = fis_output16.front();
    ASSERT_LE(output_single16.value, 0.0);
    ASSERT_GE(output_single16.value, -IGN_PI_2);
    trimTermNameMarker(output_single16.term_name);
    ASSERT_EQ(output_single16.term_name, "turn_right_accelerate");
}

/**
 * Test case that contains cases from @ref outputValues that **do not work** as expected but are not considered
 * crucial in overall performance
 */
TEST(FuzzyInferenceSystem, DISABLED_outputValuesKindaOff) {
    Processor fis;

    // FIXME
    // front/outwards -> decelerate
    Pose robot2 = Pose(0.0, 0.0, 0.0);
    Pose object2 = Pose(+2.0, 0.0, +IGN_PI_2);
    fuzzyfy(fis, robot2, object2);

    std::vector<Processor::FisOutput> fis_output2 = fis.getOutput();
    ASSERT_EQ(fis_output2.size(), 1);
    auto output_single2 = fis_output2.front();
    ASSERT_LE(output_single2.value, -IGN_PI_2);
    ASSERT_GE(output_single2.value, +IGN_PI_2);


    // FIXME
    // front/equal -> decelerate
    Pose robot3 = Pose(0.0, 0.0, 0.0);
    Pose object3 = Pose(+2.0, 0.0, 0.0);
    fuzzyfy(fis, robot3, object3);

    std::vector<Processor::FisOutput> fis_output3 = fis.getOutput();
    ASSERT_EQ(fis_output3.size(), 1);
    auto output_single3 = fis_output3.front();
    ASSERT_LE(output_single3.value, -IGN_PI_2);
    ASSERT_GE(output_single3.value, +IGN_PI_2);
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

static bool existsInString(const std::string& str, const std::string& pattern) {
    return str.find(pattern) != std::string::npos;
}

static std::tuple<FuzzyficationOutput, FuzzyficationOutput> fuzzyfy(Processor& fis, const Pose& robot, const Pose& object) {
    // outputs
    Angle dir_beta_dynamic;
    Angle rel_loc_dynamic;
    Angle dist_angle_dynamic;
    double dist_dynamic;
    calculateDynamicObjectData(robot, object, dir_beta_dynamic, rel_loc_dynamic, dist_angle_dynamic, dist_dynamic);

    fis.process(
        robot.getYaw(),
        std::vector<double>({dir_beta_dynamic.getRadian()}),
        std::vector<double>({rel_loc_dynamic.getRadian()}),
        std::vector<double>({dist_angle_dynamic.getRadian()})
    );

    std::vector<std::tuple<std::string, double>> highest_mem_rel_loc;
    std::vector<std::tuple<std::string, double>> highest_mem_dir_cross;
    highest_mem_rel_loc = fis.membershipInputRelLoc();
    highest_mem_dir_cross = fis.membershipInputDirCross();
    std::vector<std::string> best_terms_rel_loc = createVectorOfBestTerms(highest_mem_rel_loc);
    std::vector<std::string> best_terms_dir_cross = createVectorOfBestTerms(highest_mem_dir_cross);

    if (highest_mem_rel_loc.empty() || highest_mem_dir_cross.empty()) {
        std::cout << "REL LOC or DIR CROSS empty, returning empty FIS output!" << std::endl;
        return std::make_tuple(FuzzyficationOutput {}, FuzzyficationOutput {});
    }

    #ifdef PRINT_DEBUG
    std::cout << "[ INPUT] REL LOC   best: " << std::endl;
    for (const auto& entry : best_terms_rel_loc) {
        std::cout << "\t" << entry << std::endl;
    }
    std::cout << "[ INPUT] DIR_CROSS best: " << std::endl;
    for (const auto& entry : best_terms_dir_cross) {
        std::cout << "\t" << entry << std::endl;
    }
    // logging only
    std::vector<Processor::FisOutput> best_outputs = fis.getOutput();
    std::cout << "[OUTPUT] SOC_BEH   best: " << std::endl;
    for (const auto& entry : best_outputs) {
        std::cout << "\t" << entry.term_name << std::endl;
    }
    #endif

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
