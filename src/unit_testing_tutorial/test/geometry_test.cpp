// Bring in my package's API, which is what I'm testing
// Bring in gtest
#include <gtest/gtest.h>

#include "geometry/geometry.h"
#include "ros/ros.h"

class GeometryTest : public ::testing::Test {
 public:
    GeometryTest() {}
    ~GeometryTest() {}
};

// Declare a test
TEST(GeometryTest, lessThan100) {
    EXPECT_EQ(geometry::aFunction(0), 1);
    EXPECT_EQ(geometry::aFunction(10), 11);
    EXPECT_EQ(geometry::aFunction(99), 100);
}

TEST(GeometryTest, moreThan100) {
    EXPECT_EQ(geometry::aFunction(100), -1);
    EXPECT_EQ(geometry::aFunction(1000), -1);
    EXPECT_EQ(geometry::aFunction(990), -1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "geometry_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
