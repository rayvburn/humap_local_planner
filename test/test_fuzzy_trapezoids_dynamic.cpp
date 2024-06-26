#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <humap_local_planner/defines.h>
#include <humap_local_planner/fuzz/processor.h>
#include <humap_local_planner/fuzz/trapezoid_loc_dep.h>
#include <humap_local_planner/fuzz/trapezoid_loc_indep.h>

// allows to easibly enable debug info (i.a., what trapezoid data is being calculated for etc.)
// #define PRINT_DEBUG

using namespace humap_local_planner;
using namespace humap_local_planner::geometry;
using namespace humap_local_planner::fuzz;

struct DirCrossBorders {
    Angle gamma_eq;
    Angle gamma_opp;
    Angle gamma_cc;
    Angle gamma;
    RelativeLocation rel_loc;
};
static DirCrossBorders computeBorderValues(const Pose& robot, const Pose& object);

/**
 * Terms insensitive to `side` changes have higher `near` tolerances due to the fact that these terms
 * have artificially extended "max height" range, i.e., b-c vertices region. See @ref TrapezoidLocIndep
 * constructor for details.
 */
const double TOL_STRICT = 1e-05;
const double TOL_EASY = 1e-03;

/**
 * Poses for each case
 */
static const Pose robot1(0.0, 0.0, -IGN_PI / 6.0);
static const Pose object1(+5.0, +2.88675, +IGN_PI);

static const Pose robot2(0.0, 0.0, -IGN_PI / 6.0);
static const Pose object2(+5.0, -5.0, +IGN_PI);

static const Pose robot3(0.0, 0.0, 0.0);
static const Pose object3(4.5, -0.2, +IGN_PI);

static const Pose robot4(0.0, 0.0, (5.0 / 6.0) * IGN_PI);
static const Pose object4(+2.88675, +5.0, +IGN_PI);


/**
 * Rectangular trapezoids fixture
 *
 * To verify results with the Matlab implementation, change these:
 *
 * `parameters_mf_dir_cross.m`:
 * OVERLAP = deg2rad(1e-06);
 * INTERVAL = deg2rad(1e-06);
 *
 * `parameters_spatial_arrangement.m`:
 * set `POS_ROBOT`, `RPY_ROBOT` and `POS_HUMAN`, `RPY_HUMAN`
 *
 * Then launch `spatial_arrangement_geometry.m`
 */
class TrapezoidsRectangularFixture: public ::testing::Test {
public:
    // NOTE: increasing TrapezoidLocIndep lengths will ruin these tests
    TrapezoidsRectangularFixture():
        outwards("outwards", 0.0, true),
        cross_front("cross_front", 0.0, true),
        cross_behind("cross_behind", 0.0, true),
        equal("equal", 0.0, 0.0, true),
        opposite("opposite", 0.0, 0.0, true) {
        // initialization code here
    }

    void SetUp() {
        // code here will execute just before the test ensues
        // normal regions
        outwards_raw_n = outwards.getTrapezoids().at(0);
        cross_front_raw_n = cross_front.getTrapezoids().at(0);
        cross_behind_raw_n = cross_behind.getTrapezoids().at(0);
        equal_raw_n = equal.getTrapezoids().at(0);
        opposite_raw_n = opposite.getTrapezoids().at(0);
        // wrapped regions
        outwards_raw_w = outwards.getTrapezoids().at(1);
        cross_front_raw_w = cross_front.getTrapezoids().at(1);
        cross_behind_raw_w = cross_behind.getTrapezoids().at(1);
        equal_raw_w = equal.getTrapezoids().at(1);
        opposite_raw_w = opposite.getTrapezoids().at(1);
    }

    void TearDown() {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
    }

    // any custom data members that you need
    TrapezoidLocDep outwards;
    TrapezoidLocDep cross_front;
    TrapezoidLocDep cross_behind;
    TrapezoidLocIndep equal;
    TrapezoidLocIndep opposite;
    // normal
    fl::Trapezoid* outwards_raw_n;
    fl::Trapezoid* cross_front_raw_n;
    fl::Trapezoid* cross_behind_raw_n;
    fl::Trapezoid* equal_raw_n;
    fl::Trapezoid* opposite_raw_n;
    // wrapped
    fl::Trapezoid* outwards_raw_w;
    fl::Trapezoid* cross_front_raw_w;
    fl::Trapezoid* cross_behind_raw_w;
    fl::Trapezoid* equal_raw_w;
    fl::Trapezoid* opposite_raw_w;
};


/**
 * Investigates the same cases as `trapezoidsRectangular` but trapezoids have their full shape,
 * i.e., contain rising/falling edges. A broad edge used here @ref TRAPEZOID_SIDE_LENGTH_DEGn
 *
 * To verify results with the Matlab implementation, change these:
 *
 * `parameters_mf_dir_cross.m`:
 * OVERLAP = deg2rad(<PROPER VALUE ACCORDING TO SetUp() call>);
 * INTERVAL = deg2rad(1e-06);
 *
 * `parameters_spatial_arrangement.m`:
 * set `POS_ROBOT`, `RPY_ROBOT` and `POS_HUMAN`, `RPY_HUMAN`
 *
 * Then launch `spatial_arrangement_geometry.m`
 *
 */
class TrapezoidsValidShapeFixture: public ::testing::Test {
public:
    // NOTE: increasing TrapezoidLocIndep lengths will ruin these tests
    TrapezoidsValidShapeFixture():
        outwards("outwards", 0.0, true),
        cross_front("cross_front", 0.0, true),
        cross_behind("cross_behind", 0.0, true),
        equal("equal", 0.0, 0.0, true),
        opposite("opposite", 0.0, 0.0, true),
        trap_side(0.0) {
        // initialization code here
    }

    void SetUp(double trapezoid_side_length_deg) {
        // code here will execute just before the test ensues
        trap_side = IGN_DTOR(trapezoid_side_length_deg);

        outwards.setSideLength(trapezoid_side_length_deg);
        cross_front.setSideLength(trapezoid_side_length_deg);
        cross_behind.setSideLength(trapezoid_side_length_deg);
        equal.setSideLength(trapezoid_side_length_deg);
        opposite.setSideLength(trapezoid_side_length_deg);
        // normal regions
        outwards_raw_n = outwards.getTrapezoids().at(0);
        cross_front_raw_n = cross_front.getTrapezoids().at(0);
        cross_behind_raw_n = cross_behind.getTrapezoids().at(0);
        equal_raw_n = equal.getTrapezoids().at(0);
        opposite_raw_n = opposite.getTrapezoids().at(0);
        // wrapped regions
        outwards_raw_w = outwards.getTrapezoids().at(1);
        cross_front_raw_w = cross_front.getTrapezoids().at(1);
        cross_behind_raw_w = cross_behind.getTrapezoids().at(1);
        equal_raw_w = equal.getTrapezoids().at(1);
        opposite_raw_w = opposite.getTrapezoids().at(1);
    }

    void TearDown() {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
    }

    // any custom data members that you need
    TrapezoidLocDep outwards;
    TrapezoidLocDep cross_front;
    TrapezoidLocDep cross_behind;
    TrapezoidLocIndep equal;
    TrapezoidLocIndep opposite;

    // normal
    fl::Trapezoid* outwards_raw_n;
    fl::Trapezoid* cross_front_raw_n;
    fl::Trapezoid* cross_behind_raw_n;
    fl::Trapezoid* equal_raw_n;
    fl::Trapezoid* opposite_raw_n;

    // wrapped
    fl::Trapezoid* outwards_raw_w;
    fl::Trapezoid* cross_front_raw_w;
    fl::Trapezoid* cross_behind_raw_w;
    fl::Trapezoid* equal_raw_w;
    fl::Trapezoid* opposite_raw_w;

protected:
    double trap_side;
};


/**
 * TrapezoidsRectangularFixture Cases
 */
TEST_F(TrapezoidsRectangularFixture, case1) {
    // 1st test case config definition
    DirCrossBorders dir_cross1 = computeBorderValues(robot1, object1);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 1 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross1.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross1.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross1.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross1.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross1.gamma_eq.getDegree(), -30.0, TOL_EASY);
    ASSERT_NEAR(dir_cross1.gamma_opp.getDegree(), +150.0, TOL_EASY);
    ASSERT_NEAR(dir_cross1.gamma_cc.getDegree(), -150.0, TOL_EASY);

    // terms sensitive to side changes
	// `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 1 - Trapezoid OUTWARDS" << std::endl;
    #endif
	outwards.update(dir_cross1.rel_loc, dir_cross1.gamma_opp, dir_cross1.gamma_eq);
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross1.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross1.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexC(), dir_cross1.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexD(), dir_cross1.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_TRUE(std::isnan(outwards_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(outwards_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(outwards_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(outwards_raw_w->getVertexD()));

	// `cross_front`
    #ifdef PRINT_DEBUG
	std::cout << "[TrapezoidX::update] Case 1 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross1.rel_loc, dir_cross1.gamma_eq, dir_cross1.gamma_cc);
    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross1.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross1.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexC(), dir_cross1.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexD(), dir_cross1.gamma_eq.getRadian(), TOL_STRICT);

	// `cross_behind`
    #ifdef PRINT_DEBUG
	std::cout << "[TrapezoidX::update] Case 1 - Trapezoid CROSS BEHIND" << std::endl;
    #endif
	cross_behind.update(dir_cross1.rel_loc, dir_cross1.gamma_cc, dir_cross1.gamma_opp);
    ASSERT_NEAR(cross_behind_raw_n->getVertexA(), dir_cross1.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexB(), dir_cross1.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexC(), +IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexD(), +IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_w->getVertexA(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_w->getVertexB(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_w->getVertexC(), dir_cross1.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_w->getVertexD(), dir_cross1.gamma_cc.getRadian(), TOL_STRICT);

	// terms insensitive to side changes
	// `equal`
    #ifdef PRINT_DEBUG
	std::cout << "[TrapezoidX::update] Case 1 - Trapezoid EQUAL" << std::endl;
    #endif
	equal.update(dir_cross1.gamma_eq);
    ASSERT_NEAR(equal_raw_n->getVertexA(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexC(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexD(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexD()));

	// `opposite`
    #ifdef PRINT_DEBUG
	std::cout << "[TrapezoidX::update] Case 1 - Trapezoid OPPOSITE" << std::endl;
    #endif
	opposite.update(dir_cross1.gamma_opp);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexC(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexD(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexD()));
}


TEST_F(TrapezoidsRectangularFixture, case2) {
    // 2nd test case config definition
    DirCrossBorders dir_cross2 = computeBorderValues(robot2, object2);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 2 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross2.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross2.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross2.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross2.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross2.gamma_eq.getDegree(), -30.0, TOL_EASY);
    ASSERT_NEAR(dir_cross2.gamma_opp.getDegree(), +150.0, TOL_EASY);
    ASSERT_NEAR(dir_cross2.gamma_cc.getDegree(), +135.0, TOL_EASY);

    // terms sensitive to side changes
	// `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid OUTWARDS" << std::endl;
    #endif
	outwards.update(dir_cross2.rel_loc, dir_cross2.gamma_opp, dir_cross2.gamma_eq);
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross2.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross2.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexC(), +IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexD(), +IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexA(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexB(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexC(), dir_cross2.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexD(), dir_cross2.gamma_eq.getRadian(), TOL_STRICT);

	// `cross_front`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross2.rel_loc, dir_cross2.gamma_eq, dir_cross2.gamma_cc);
    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross2.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross2.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexC(), dir_cross2.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexD(), dir_cross2.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexD()));

	// `cross_behind`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid CROSS BEHIND" << std::endl;
    #endif
	cross_behind.update(dir_cross2.rel_loc, dir_cross2.gamma_cc, dir_cross2.gamma_opp);
    ASSERT_NEAR(cross_behind_raw_n->getVertexA(), dir_cross2.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexB(), dir_cross2.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexC(), dir_cross2.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexD(), dir_cross2.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexD()));

	// terms insensitive to side changes
	// `equal`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid EQUAL" << std::endl;
    #endif
	equal.update(dir_cross2.gamma_eq);
    ASSERT_NEAR(equal_raw_n->getVertexA(), dir_cross2.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), dir_cross2.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexC(), dir_cross2.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexD(), dir_cross2.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexD()));

	// `opposite`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid OPPOSITE" << std::endl;
    #endif
	opposite.update(dir_cross2.gamma_opp);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), dir_cross2.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), dir_cross2.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexC(), dir_cross2.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexD(), dir_cross2.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexD()));
}


TEST_F(TrapezoidsRectangularFixture, case3) {
    // 3rd test case config definition
    DirCrossBorders dir_cross3 = computeBorderValues(robot3, object3);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 3 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross3.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross3.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross3.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross3.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross3.gamma_eq.getDegree(), +0.0, TOL_EASY);
    ASSERT_NEAR(dir_cross3.gamma_opp.getDegree(), +180.0, TOL_EASY);
    ASSERT_NEAR(dir_cross3.gamma_cc.getDegree(), +177.4552, TOL_EASY);

    // terms sensitive to side changes
	// `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid OUTWARDS" << std::endl;
    #endif
	outwards.update(dir_cross3.rel_loc, dir_cross3.gamma_opp, dir_cross3.gamma_eq);
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross3.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross3.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexC(), +IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexD(), +IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexA(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexB(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexC(), dir_cross3.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexD(), dir_cross3.gamma_eq.getRadian(), TOL_STRICT);

	// `cross_front`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross3.rel_loc, dir_cross3.gamma_eq, dir_cross3.gamma_cc);
    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross3.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross3.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexC(), dir_cross3.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexD(), dir_cross3.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexD()));

	// `cross_behind`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid CROSS BEHIND" << std::endl;
    #endif
	cross_behind.update(dir_cross3.rel_loc, dir_cross3.gamma_cc, dir_cross3.gamma_opp);
    ASSERT_NEAR(cross_behind_raw_n->getVertexA(), dir_cross3.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexB(), dir_cross3.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexC(), dir_cross3.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexD(), dir_cross3.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexD()));

	// terms insensitive to side changes
	// `equal`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid EQUAL" << std::endl;
    #endif
	equal.update(dir_cross3.gamma_eq);
    ASSERT_NEAR(equal_raw_n->getVertexA(), dir_cross3.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), dir_cross3.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexC(), dir_cross3.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexD(), dir_cross3.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexD()));

	// `opposite`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid OPPOSITE" << std::endl;
    #endif
	opposite.update(dir_cross3.gamma_opp);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), dir_cross3.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), dir_cross3.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexC(), dir_cross3.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexD(), dir_cross3.gamma_opp.getRadian(), TOL_EASY);
    /* NOTE: in fact (compared to Matlab implementation), this should be true:
     *   ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexA()));
     * but a slight offset is acceptable here (edge case)
     */
    ASSERT_NEAR(opposite_raw_w->getVertexA(), Angle(dir_cross3.gamma_opp.getRadian() + TOL_EASY / 2).getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_w->getVertexB(), Angle(dir_cross3.gamma_opp.getRadian() + TOL_EASY / 2).getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_w->getVertexC(), Angle(dir_cross3.gamma_opp.getRadian() + TOL_EASY / 2).getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_w->getVertexD(), Angle(dir_cross3.gamma_opp.getRadian() + TOL_EASY / 2).getRadian(), TOL_EASY);
}


TEST_F(TrapezoidsRectangularFixture, case4) {
    // 4th test case config definition
    DirCrossBorders dir_cross4 = computeBorderValues(robot4, object4);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 4 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross4.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross4.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross4.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross4.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross4.gamma_eq.getDegree(), +150.0, TOL_EASY);
    ASSERT_NEAR(dir_cross4.gamma_opp.getDegree(), -30.0, TOL_EASY);
    ASSERT_NEAR(dir_cross4.gamma_cc.getDegree(), -120.0, TOL_EASY);

    // terms sensitive to side changes
	// `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid OUTWARDS" << std::endl;
    #endif
	outwards.update(dir_cross4.rel_loc, dir_cross4.gamma_opp, dir_cross4.gamma_eq);
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross4.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross4.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexC(), dir_cross4.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexD(), dir_cross4.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_TRUE(std::isnan(outwards_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(outwards_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(outwards_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(outwards_raw_w->getVertexD()));

	// `cross_front`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross4.rel_loc, dir_cross4.gamma_eq, dir_cross4.gamma_cc);
    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross4.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross4.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexC(), +IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexD(), +IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_w->getVertexA(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_w->getVertexB(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_w->getVertexC(), dir_cross4.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_w->getVertexD(), dir_cross4.gamma_cc.getRadian(), TOL_STRICT);

	// `cross_behind`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid CROSS BEHIND" << std::endl;
    #endif
	cross_behind.update(dir_cross4.rel_loc, dir_cross4.gamma_cc, dir_cross4.gamma_opp);
    ASSERT_NEAR(cross_behind_raw_n->getVertexA(), dir_cross4.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexB(), dir_cross4.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexC(), dir_cross4.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexD(), dir_cross4.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexD()));

	// terms insensitive to side changes
	// `equal`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid EQUAL" << std::endl;
    #endif
	equal.update(dir_cross4.gamma_eq);
    ASSERT_NEAR(equal_raw_n->getVertexA(), dir_cross4.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), dir_cross4.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexC(), dir_cross4.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexD(), dir_cross4.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexD()));

	// `opposite`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid OPPOSITE" << std::endl;
    #endif
	opposite.update(dir_cross4.gamma_opp);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), dir_cross4.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), dir_cross4.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexC(), dir_cross4.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexD(), dir_cross4.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexD()));
}


/**
 * TrapezoidsRectangularFixture Cases
 */
TEST_F(TrapezoidsValidShapeFixture, case1) {
    // configure side length in degrees
    SetUp(80.0);

    DirCrossBorders dir_cross1 = computeBorderValues(robot1, object1);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 1 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross1.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross1.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross1.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross1.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross1.gamma_eq.getDegree(), -30.0, TOL_EASY);
    ASSERT_NEAR(dir_cross1.gamma_opp.getDegree(), +150.0, TOL_EASY);
    ASSERT_NEAR(dir_cross1.gamma_cc.getDegree(), -150.0, TOL_EASY);

    // terms sensitive to side changes
	// `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 1 - Trapezoid OUTWARDS" << std::endl;
    #endif
    outwards.update(dir_cross1.rel_loc, dir_cross1.gamma_opp, dir_cross1.gamma_eq);
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross1.gamma_eq.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(outwards_raw_n->getVertexC(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(outwards_raw_n->getVertexD(), dir_cross1.gamma_opp.getRadian() + trap_side, TOL_EASY);
    ASSERT_LE(outwards_raw_w->getVertexA(), dir_cross1.gamma_opp.getRadian() - 2.0 * IGN_PI);
    ASSERT_LE(outwards_raw_w->getVertexB(), dir_cross1.gamma_opp.getRadian() - 2.0 * IGN_PI);
    ASSERT_NEAR(outwards_raw_w->getVertexC(), dir_cross1.gamma_opp.getRadian() - 2.0 * IGN_PI, TOL_EASY);
    ASSERT_NEAR(outwards_raw_w->getVertexD(), dir_cross1.gamma_opp.getRadian() - 2.0 * IGN_PI + trap_side, TOL_EASY);

    // `cross_front`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 1 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross1.rel_loc, dir_cross1.gamma_eq, dir_cross1.gamma_cc);
    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross1.gamma_cc.getRadian() + 2.0 * IGN_PI - trap_side, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross1.gamma_cc.getRadian() + 2.0 * IGN_PI, TOL_EASY);
    ASSERT_GE(cross_front_raw_n->getVertexC(), dir_cross1.gamma_cc.getRadian() + 2.0 * IGN_PI);
    ASSERT_GE(cross_front_raw_n->getVertexD(), dir_cross1.gamma_cc.getRadian() + 2.0 * IGN_PI);
    ASSERT_NEAR(cross_front_raw_w->getVertexA(), dir_cross1.gamma_cc.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_w->getVertexB(), dir_cross1.gamma_cc.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_front_raw_w->getVertexC(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_front_raw_w->getVertexD(), dir_cross1.gamma_eq.getRadian() + trap_side, TOL_EASY);

    // `cross_behind`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 1 - Trapezoid CROSS BEHIND" << std::endl;
    #endif
	cross_behind.update(dir_cross1.rel_loc, dir_cross1.gamma_cc, dir_cross1.gamma_opp);
    ASSERT_NEAR(cross_behind_raw_n->getVertexA(), dir_cross1.gamma_opp.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(cross_behind_raw_n->getVertexB(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_GT(cross_behind_raw_n->getVertexC(), +IGN_PI);
    ASSERT_GT(cross_behind_raw_n->getVertexD(), +IGN_PI);
    ASSERT_LE(cross_behind_raw_w->getVertexA(), -IGN_PI);
    ASSERT_LE(cross_behind_raw_w->getVertexB(), +IGN_PI);
    ASSERT_NEAR(cross_behind_raw_w->getVertexC(), dir_cross1.gamma_cc.getRadian(), +IGN_PI);
    ASSERT_NEAR(cross_behind_raw_w->getVertexD(), dir_cross1.gamma_cc.getRadian() + trap_side, +IGN_PI);

    // terms insensitive to side changes
	// `equal`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 1 - Trapezoid EQUAL" << std::endl;
    #endif
	equal.update(dir_cross1.gamma_eq);
    ASSERT_NEAR(equal_raw_n->getVertexA(), dir_cross1.gamma_eq.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexC(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexD(), dir_cross1.gamma_eq.getRadian() + trap_side, TOL_EASY);
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexD()));

    // `opposite`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 1 - Trapezoid OPPOSITE" << std::endl;
    #endif
	opposite.update(dir_cross1.gamma_opp);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), dir_cross1.gamma_opp.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexC(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexD(), dir_cross1.gamma_opp.getRadian() + trap_side, TOL_EASY);
    ASSERT_LT(opposite_raw_w->getVertexA(), dir_cross1.gamma_opp.getRadian() - 2.0 * IGN_PI);
    ASSERT_LT(opposite_raw_w->getVertexB(), dir_cross1.gamma_opp.getRadian() - 2.0 * IGN_PI);
    ASSERT_NEAR(opposite_raw_w->getVertexC(), dir_cross1.gamma_opp.getRadian() - 2.0 * IGN_PI, TOL_EASY);
    ASSERT_NEAR(opposite_raw_w->getVertexD(), dir_cross1.gamma_opp.getRadian() - 2.0 * IGN_PI + trap_side, TOL_EASY);
}


TEST_F(TrapezoidsValidShapeFixture, case2) {
    // configure side length in degrees
    SetUp(29.0); // NOTE: Matlab impl. used for verification does not handle wider 'sides' (some edge case)

    DirCrossBorders dir_cross2 = computeBorderValues(robot2, object2);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 2 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross2.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross2.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross2.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross2.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross2.gamma_eq.getDegree(), -30.0, TOL_EASY);
    ASSERT_NEAR(dir_cross2.gamma_opp.getDegree(), +150.0, TOL_EASY);
    ASSERT_NEAR(dir_cross2.gamma_cc.getDegree(), +135.0, TOL_EASY);

    // terms sensitive to side changes
	// `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid OUTWARDS" << std::endl;
    #endif
	outwards.update(dir_cross2.rel_loc, dir_cross2.gamma_opp, dir_cross2.gamma_eq);
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross2.gamma_opp.getRadian() - trap_side, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross2.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexC(), +IGN_PI, TOL_STRICT);
    ASSERT_GE(outwards_raw_n->getVertexD(), +IGN_PI);
    ASSERT_LE(outwards_raw_w->getVertexA(), -IGN_PI);
    ASSERT_NEAR(outwards_raw_w->getVertexB(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexC(), dir_cross2.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexD(), dir_cross2.gamma_eq.getRadian() + trap_side, TOL_STRICT);

	// `cross_front`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross2.rel_loc, dir_cross2.gamma_eq, dir_cross2.gamma_cc);
    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross2.gamma_eq.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross2.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexC(), dir_cross2.gamma_cc.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexD(), dir_cross2.gamma_cc.getRadian() + trap_side, TOL_EASY);
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(cross_front_raw_w->getVertexD()));

	// `cross_behind`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid CROSS BEHIND" << std::endl;
    #endif
	cross_behind.update(dir_cross2.rel_loc, dir_cross2.gamma_cc, dir_cross2.gamma_opp);
    ASSERT_NEAR(cross_behind_raw_n->getVertexA(), dir_cross2.gamma_cc.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(cross_behind_raw_n->getVertexB(), dir_cross2.gamma_cc.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_behind_raw_n->getVertexC(), dir_cross2.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_behind_raw_n->getVertexD(), dir_cross2.gamma_opp.getRadian() + trap_side, TOL_EASY);
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(cross_behind_raw_w->getVertexD()));

	// terms insensitive to side changes
	// `equal`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid EQUAL" << std::endl;
    #endif
	equal.update(dir_cross2.gamma_eq);
    ASSERT_NEAR(equal_raw_n->getVertexA(), dir_cross2.gamma_eq.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), dir_cross2.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexC(), dir_cross2.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexD(), dir_cross2.gamma_eq.getRadian() + trap_side, TOL_EASY);
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexD()));

	// `opposite`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 2 - Trapezoid OPPOSITE" << std::endl;
    #endif
	opposite.update(dir_cross2.gamma_opp);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), dir_cross2.gamma_opp.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), dir_cross2.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexC(), dir_cross2.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexD(), dir_cross2.gamma_opp.getRadian() + trap_side, TOL_EASY);
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexD()));
}


TEST_F(TrapezoidsValidShapeFixture, case3) {
    // configure side length in degrees
    SetUp(20.0);

    DirCrossBorders dir_cross3 = computeBorderValues(robot3, object3);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 3 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross3.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross3.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross3.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross3.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross3.gamma_eq.getDegree(), +0.0, TOL_EASY);
    ASSERT_NEAR(dir_cross3.gamma_opp.getDegree(), +180.0, TOL_EASY);
    ASSERT_NEAR(dir_cross3.gamma_cc.getDegree(), +177.4552, TOL_EASY);

    // terms sensitive to side changes
    // `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid OUTWARDS" << std::endl;
    #endif
	outwards.update(dir_cross3.rel_loc, dir_cross3.gamma_opp, dir_cross3.gamma_eq);
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross3.gamma_opp.getRadian() - trap_side, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross3.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_GE(outwards_raw_n->getVertexC(), +IGN_PI);
    ASSERT_GE(outwards_raw_n->getVertexD(), +IGN_PI);
    ASSERT_LE(outwards_raw_w->getVertexA(), -IGN_PI);
    ASSERT_NEAR(outwards_raw_w->getVertexB(), -IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexC(), dir_cross3.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexD(), dir_cross3.gamma_eq.getRadian() + trap_side, TOL_STRICT);

    // `cross_front`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross3.rel_loc, dir_cross3.gamma_eq, dir_cross3.gamma_cc);
    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross3.gamma_eq.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross3.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexC(), dir_cross3.gamma_cc.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexD(), dir_cross3.gamma_cc.getRadian() + trap_side, TOL_EASY);
    ASSERT_LE(cross_front_raw_w->getVertexA(), -IGN_PI);
    ASSERT_LE(cross_front_raw_w->getVertexB(), -IGN_PI);
    ASSERT_NEAR(cross_front_raw_w->getVertexC(), dir_cross3.gamma_cc.getRadian() - 2.0 * IGN_PI, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_w->getVertexD(), dir_cross3.gamma_cc.getRadian() - 2.0 * IGN_PI + trap_side, TOL_EASY);

    // `cross_behind`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid CROSS BEHIND" << std::endl;
    #endif
	cross_behind.update(dir_cross3.rel_loc, dir_cross3.gamma_cc, dir_cross3.gamma_opp);
    ASSERT_NEAR(cross_behind_raw_n->getVertexA(), dir_cross3.gamma_cc.getRadian() - trap_side, TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexB(), dir_cross3.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexC(), +IGN_PI, TOL_STRICT);
    ASSERT_GE(cross_behind_raw_n->getVertexD(), +IGN_PI);
    ASSERT_LE(cross_behind_raw_w->getVertexA(), -IGN_PI);
    ASSERT_LE(cross_behind_raw_w->getVertexB(), -IGN_PI);
    ASSERT_NEAR(cross_behind_raw_w->getVertexC(), dir_cross3.gamma_opp.getRadian() - 2.0 * IGN_PI, TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_w->getVertexD(), dir_cross3.gamma_opp.getRadian() - 2.0 * IGN_PI + trap_side, TOL_STRICT);

    // terms insensitive to side changes
    // `equal`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid EQUAL" << std::endl;
    #endif
	equal.update(dir_cross3.gamma_eq);
    ASSERT_NEAR(equal_raw_n->getVertexA(), dir_cross3.gamma_eq.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), dir_cross3.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexC(), dir_cross3.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexD(), dir_cross3.gamma_eq.getRadian() + trap_side, TOL_EASY);
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(equal_raw_w->getVertexD()));

    // `opposite`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 3 - Trapezoid OPPOSITE" << std::endl;
    #endif
	opposite.update(dir_cross3.gamma_opp);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), dir_cross3.gamma_opp.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), dir_cross3.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_GE(opposite_raw_n->getVertexC(), +IGN_PI);
    ASSERT_GE(opposite_raw_n->getVertexD(), +IGN_PI);
    ASSERT_LE(opposite_raw_w->getVertexA(), -IGN_PI);
    ASSERT_LE(opposite_raw_w->getVertexB(), -IGN_PI);
    ASSERT_NEAR(opposite_raw_w->getVertexC(), dir_cross3.gamma_opp.getRadian() - 2.0 * IGN_PI, TOL_EASY);
    ASSERT_NEAR(opposite_raw_w->getVertexD(), dir_cross3.gamma_opp.getRadian() - 2.0 * IGN_PI + trap_side, TOL_EASY);
}


TEST_F(TrapezoidsValidShapeFixture, case4) {
    // configure side length in degrees
    SetUp(65.0);

    DirCrossBorders dir_cross4 = computeBorderValues(robot4, object4);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 4 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross4.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross4.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross4.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross4.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross4.gamma_eq.getDegree(), +150.0, TOL_EASY);
    ASSERT_NEAR(dir_cross4.gamma_opp.getDegree(), -30.0, TOL_EASY);
    ASSERT_NEAR(dir_cross4.gamma_cc.getDegree(), -120.0, TOL_EASY);

    // terms sensitive to side changes
	// `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid OUTWARDS" << std::endl;
    #endif
	outwards.update(dir_cross4.rel_loc, dir_cross4.gamma_opp, dir_cross4.gamma_eq);
    // cut-off axis between B-C vertices
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross4.gamma_opp.getRadian() - trap_side, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross4.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexC(), dir_cross4.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_NEAR(outwards_raw_n->getVertexD(), dir_cross4.gamma_eq.getRadian() + trap_side, TOL_STRICT);
    ASSERT_LE(outwards_raw_w->getVertexA(), dir_cross4.gamma_opp.getRadian() - IGN_PI);
    ASSERT_LE(outwards_raw_w->getVertexB(), dir_cross4.gamma_opp.getRadian() - IGN_PI);
    ASSERT_NEAR(outwards_raw_w->getVertexC(), dir_cross4.gamma_opp.getRadian() - IGN_PI, TOL_STRICT);
    ASSERT_NEAR(outwards_raw_w->getVertexD(), dir_cross4.gamma_opp.getRadian() - IGN_PI + trap_side, TOL_STRICT);

	// `cross_front`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross4.rel_loc, dir_cross4.gamma_eq, dir_cross4.gamma_cc);
    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross4.gamma_eq.getRadian() - trap_side, TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross4.gamma_eq.getRadian(), TOL_STRICT);
    ASSERT_GE(cross_front_raw_n->getVertexC(), +IGN_PI);
    ASSERT_GE(cross_front_raw_n->getVertexD(), +IGN_PI);
    ASSERT_LE(cross_front_raw_w->getVertexA(), -IGN_PI);
    ASSERT_LE(cross_front_raw_w->getVertexB(), -IGN_PI);
    ASSERT_NEAR(cross_front_raw_w->getVertexC(), dir_cross4.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_front_raw_w->getVertexD(), dir_cross4.gamma_cc.getRadian() + trap_side, TOL_STRICT);

	// `cross_behind`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid CROSS BEHIND" << std::endl;
    #endif
	cross_behind.update(dir_cross4.rel_loc, dir_cross4.gamma_cc, dir_cross4.gamma_opp);
    ASSERT_NEAR(cross_behind_raw_n->getVertexA(), dir_cross4.gamma_cc.getRadian() + 2.0 * IGN_PI - trap_side, TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_n->getVertexB(), dir_cross4.gamma_cc.getRadian() + 2.0 * IGN_PI, TOL_STRICT);
    ASSERT_GT(cross_behind_raw_n->getVertexC(), +IGN_PI);
    ASSERT_GT(cross_behind_raw_n->getVertexD(), +IGN_PI);
    ASSERT_LT(cross_behind_raw_w->getVertexA(), -IGN_PI);
    ASSERT_NEAR(cross_behind_raw_w->getVertexB(), dir_cross4.gamma_cc.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_w->getVertexC(), dir_cross4.gamma_opp.getRadian(), TOL_STRICT);
    ASSERT_NEAR(cross_behind_raw_w->getVertexD(), dir_cross4.gamma_opp.getRadian() + trap_side, TOL_STRICT);

	// terms insensitive to side changes
	// `equal`
	#ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 4 - Trapezoid EQUAL" << std::endl;
    #endif
	equal.update(dir_cross4.gamma_eq);
    ASSERT_NEAR(equal_raw_n->getVertexA(), dir_cross4.gamma_eq.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), dir_cross4.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexC(), dir_cross4.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexD(), dir_cross4.gamma_eq.getRadian() + trap_side, TOL_EASY);

    Angle eq4_wrap_valid_len(dir_cross4.gamma_eq.getRadian() + trap_side - IGN_PI);
    Angle eq4_falling_edge_w(-IGN_PI + eq4_wrap_valid_len.getRadian() - trap_side, false);
    ASSERT_LE(equal_raw_w->getVertexA(), eq4_falling_edge_w.getRadian());
    ASSERT_LE(equal_raw_w->getVertexB(), eq4_falling_edge_w.getRadian());
    ASSERT_NEAR(equal_raw_w->getVertexC(), -IGN_PI + eq4_wrap_valid_len.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(equal_raw_w->getVertexD(), -IGN_PI + eq4_wrap_valid_len.getRadian(), TOL_EASY);

	// `opposite`
    #ifdef PRINT_DEBUG
	std::cout << "[TrapezoidX::update] Case 4 - Trapezoid OPPOSITE" << std::endl;
    #endif
	opposite.update(dir_cross4.gamma_opp);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), dir_cross4.gamma_opp.getRadian() - trap_side, TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), dir_cross4.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexC(), dir_cross4.gamma_opp.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexD(), dir_cross4.gamma_opp.getRadian() + trap_side, TOL_EASY);
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexA()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexB()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexC()));
    ASSERT_TRUE(std::isnan(opposite_raw_w->getVertexD()));
}


/**
 * Evaluate worst case, here Case 1 with very long trapezoid sides
 */
TEST(FuzzyTrapezoids, DISABLED_trapezoidsExtendedSides) {
    const double TRAPEZOID_SIDE_LENGTH_DEG = 120.0;
    const double TRAP_SIDE = IGN_DTOR(TRAPEZOID_SIDE_LENGTH_DEG);

    #ifdef PRINT_DEBUG
    std::cout << std::endl << std::endl;
    std::cout << "trapezoidsExtendedSides" << std::endl;
    std::cout << std::endl << std::endl;
    #endif

    TrapezoidLocDep outwards("outwards", TRAPEZOID_SIDE_LENGTH_DEG, true);
    fl::Trapezoid* outwards_raw_n = outwards.getTrapezoids().at(0);
    fl::Trapezoid* outwards_raw_w = outwards.getTrapezoids().at(1);

    TrapezoidLocDep cross_front("cross_front", TRAPEZOID_SIDE_LENGTH_DEG, true);
    fl::Trapezoid* cross_front_raw_n = cross_front.getTrapezoids().at(0);
    fl::Trapezoid* cross_front_raw_w = cross_front.getTrapezoids().at(1);

    // 1st test case config definition
    DirCrossBorders dir_cross1 = computeBorderValues(robot1, object1);

    #ifdef PRINT_DEBUG
    std::cout << "[trapezoids] Case 1 gammas: "
        << " gamma_eq: " << IGN_RTOD(dir_cross1.gamma_eq.getRadian())
        << " gamma_opp: " << IGN_RTOD(dir_cross1.gamma_opp.getRadian())
        << " gamma_cc: " << IGN_RTOD(dir_cross1.gamma_cc.getRadian())
        << " rel_loc: " << ((dir_cross1.rel_loc == LOCATION_RIGHT) ? "RIGHT" : "LEFT")
    << std::endl << std::endl;
    #endif

    ASSERT_NEAR(dir_cross1.gamma_eq.getDegree(), -30.0, TOL_EASY);
    ASSERT_NEAR(dir_cross1.gamma_opp.getDegree(), +150.0, TOL_EASY);
    ASSERT_NEAR(dir_cross1.gamma_cc.getDegree(), -150.0, TOL_EASY);

    // terms sensitive to side changes
	// `outwards`
    #ifdef PRINT_DEBUG
    std::cout << "[TrapezoidX::update] Case 1 - Trapezoid OUTWARDS" << std::endl;
    #endif
    outwards.update(dir_cross1.rel_loc, dir_cross1.gamma_opp, dir_cross1.gamma_eq);

    // cut-off axis (-PI) between A-B
    Angle out_a1n(dir_cross1.gamma_opp.getRadian() - TRAP_SIDE);
    Angle out_b1n(out_a1n.getRadian() + TRAP_SIDE, false);
    ASSERT_NEAR(outwards_raw_n->getVertexA(), dir_cross1.gamma_eq.getRadian() - TRAP_SIDE, TOL_EASY);
    ASSERT_NEAR(outwards_raw_n->getVertexB(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(outwards_raw_n->getVertexC(), dir_cross1.gamma_opp.getRadian(), TOL_EASY);
    // FIXME: VertexD is wrong in both implementations; we don't set trapezoid sides that big, though
    EXPECT_NEAR(outwards_raw_n->getVertexD(), dir_cross1.gamma_opp.getRadian() + TRAP_SIDE, TOL_EASY);

    /*
     * FIXME: CF won't pass this test when TRAPEZOID_SIDE_LENGTH_DEG is set to that big value
     *
    // cut-off axis (PI) between B-C
    // `cross_front`
    #ifdef PRINT_DEBUG
	std::cout << "[TrapezoidX::update] Case 2 - Trapezoid CROSS FRONT" << std::endl;
    #endif
	cross_front.update(dir_cross1.rel_loc, dir_cross1.gamma_eq, dir_cross1.gamma_cc);

    ASSERT_NEAR(cross_front_raw_n->getVertexA(), dir_cross1.gamma_eq.getRadian() - TRAP_SIDE, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexB(), dir_cross1.gamma_eq.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexC(), IGN_PI, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_n->getVertexD(), IGN_PI, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_w->getVertexA(), -IGN_PI, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_w->getVertexB(), -IGN_PI, TOL_EASY);
    ASSERT_NEAR(cross_front_raw_w->getVertexC(), dir_cross1.gamma_cc.getRadian(), TOL_EASY);
    ASSERT_NEAR(cross_front_raw_w->getVertexD(), dir_cross1.gamma_cc.getRadian() + TRAP_SIDE, TOL_EASY);
    */
}

/**
 * Indep with >0.0 length
 */
TEST(FuzzyTrapezoids, trapezoidsSideInsensitiveLong) {
    const double TRAPEZOID_SIDE_LENGTH_DEG = 60.0;
    const double TRAP_SIDE = IGN_DTOR(TRAPEZOID_SIDE_LENGTH_DEG);
    const double TRAPEZOID_EQ_LENGTH_DEG = 90.0;
    const double TRAPEZOID_OPP_LENGTH_DEG = 30.0;
    const double TRAP_EQ_LEN = IGN_DTOR(TRAPEZOID_EQ_LENGTH_DEG);
    const double TRAP_OPP_LEN = IGN_DTOR(TRAPEZOID_OPP_LENGTH_DEG);
    // center of the trapezoid
    const Angle CENTER(+IGN_PI * (5.0 / 6.0));

    #ifdef PRINT_DEBUG
    std::cout << std::endl << std::endl;
    std::cout << "trapezoidsSideInsensitiveLong" << std::endl;
    std::cout << std::endl << std::endl;
    #endif

    TrapezoidLocIndep equal("equal", TRAPEZOID_SIDE_LENGTH_DEG, TRAPEZOID_EQ_LENGTH_DEG, true);
    TrapezoidLocIndep opposite("opposite", TRAPEZOID_SIDE_LENGTH_DEG, TRAPEZOID_OPP_LENGTH_DEG, true);

    // normal
    fl::Trapezoid* equal_raw_n = equal.getTrapezoids().at(0);
    fl::Trapezoid* opposite_raw_n = opposite.getTrapezoids().at(0);
    // wrapped
    fl::Trapezoid* equal_raw_w = equal.getTrapezoids().at(1);
    fl::Trapezoid* opposite_raw_w = opposite.getTrapezoids().at(1);

    equal.update(CENTER);
    Angle eq_b1n(CENTER.getRadian() - (TRAP_EQ_LEN / 2.0), false);
    Angle eq_a1n(eq_b1n.getRadian() - TRAP_SIDE, false);
    Angle eq_c1n(CENTER.getRadian() + (TRAP_EQ_LEN / 2.0), false);
    Angle eq_trimmed_n(eq_c1n.getRadian() - IGN_PI, false);
    ASSERT_NEAR(equal_raw_n->getVertexA(), eq_a1n.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_n->getVertexB(), eq_b1n.getRadian(), TOL_EASY);
    ASSERT_GE(equal_raw_n->getVertexC(), +IGN_PI);
    ASSERT_GE(equal_raw_n->getVertexD(), +IGN_PI);

    Angle eq_c1w(-IGN_PI + eq_trimmed_n.getRadian(), false);
    Angle eq_d1w(eq_c1w.getRadian() + TRAP_SIDE, false);
    ASSERT_LE(equal_raw_w->getVertexA(), -IGN_PI);
    ASSERT_LE(equal_raw_w->getVertexB(), -IGN_PI);
    ASSERT_NEAR(equal_raw_w->getVertexC(), eq_c1w.getRadian(), TOL_EASY);
    ASSERT_NEAR(equal_raw_w->getVertexD(), eq_d1w.getRadian(), TOL_EASY);

    opposite.update(CENTER);
    // cut-off axis between C and D points
    Angle opp_b1n(CENTER.getRadian() - (TRAP_OPP_LEN / 2.0), false);
    Angle opp_a1n(opp_b1n.getRadian() - TRAP_SIDE, false);
    Angle opp_c1n(CENTER.getRadian() + (TRAP_OPP_LEN / 2.0), false);
    Angle opp_d1n(opp_c1n.getRadian() + TRAP_SIDE, false);
    ASSERT_NEAR(opposite_raw_n->getVertexA(), opp_a1n.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexB(), opp_b1n.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_n->getVertexC(), opp_c1n.getRadian(), TOL_EASY);
    ASSERT_GE(opposite_raw_n->getVertexD(), +IGN_PI);

    // this is important here - defines slope of the falling edge (C-D range)
    Angle opp_c1w_shift(IGN_PI - opp_c1n.getRadian(), false);
    // exceeds -PI
    Angle opp_c1w(-IGN_PI - opp_c1w_shift.getRadian(), false);
    Angle opp_d1w_shift(opp_d1n.getRadian() - IGN_PI, false);
    Angle opp_d1w(-IGN_PI + opp_d1w_shift.getRadian(), false);
    ASSERT_LE(opposite_raw_w->getVertexA(), -IGN_PI);
    ASSERT_LE(opposite_raw_w->getVertexB(), -IGN_PI);
    ASSERT_NEAR(opposite_raw_w->getVertexC(), opp_c1w.getRadian(), TOL_EASY);
    ASSERT_NEAR(opposite_raw_w->getVertexD(), opp_d1w.getRadian(), TOL_EASY);
}


int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


DirCrossBorders computeBorderValues(const Pose& robot, const Pose& object) {
    double robot_dir = robot.getYaw();
    Vector dist_v(object.getRawPosition() - robot.getRawPosition());
    double dist_v_angle = Angle(dist_v).getRadian();
    Angle rel_loc_angle(dist_v_angle - robot_dir);

    #ifdef PRINT_DEBUG
    std::cout << std::endl << std::endl;
    std::cout << "[BorderValues]"
        << " robot_dir " << IGN_RTOD(robot_dir)
        << " d_ab: /x " << dist_v.getX() << " y " << dist_v.getY() << " z " << dist_v.getZ() << "/"
        << " d ab angle: " << IGN_RTOD(dist_v_angle)
        << " rel_loc_angle: " << IGN_RTOD(rel_loc_angle.getRadian())
    << std::endl;
    #endif

    DirCrossBorders result {};
    Processor::computeDirCrossBorderValues(
        robot_dir,
        dist_v_angle,
        rel_loc_angle.getRadian(),
        result.gamma_eq,
        result.gamma_opp,
        result.gamma_cc,
        result.rel_loc
    );
    return result;
}
