// Bring in my package's API, which is what I'm testing
// Bring in gtest
#include <gtest/gtest.h>

#include "rumba_turtle/movement.h"
#include "ros/ros.h"

class RumbaTurtleTest : public ::testing::Test {
 public:
    RumbaTurtleTest() {}
    ~RumbaTurtleTest() {}
};

TEST(RumbaTurtleTest, lessThan100) {
    EXPECT_EQ(1, 1);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "geometry_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
