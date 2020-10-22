// Bring in my package's API, which is what I'm testing
// Bring in gtest
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "rumba_turtle/movement.h"
#include "turtlesim/Pose.h"

ros::NodeHandle* node = nullptr;
turtlesim::Pose real_turtle_pose;

void updatePose(const turtlesim::Pose::ConstPtr& pose_message) {
    real_turtle_pose.x = pose_message->x;
    real_turtle_pose.y = pose_message->y;
    real_turtle_pose.theta = pose_message->theta;
}

class RumbaTurtleTest : public ::testing::Test {
 public:
    RumbaTurtleTest() {}
    ~RumbaTurtleTest() {}
};

TEST_F(RumbaTurtleTest, GoToGoal) {
    const int loop_frequency = 100;
    turtlesim::Pose goal_pose;
    goal_pose.x = 1;
    goal_pose.y = 1;
    goal_pose.theta = 0;
    const double distance_tolerance = 0.3;
    const char* const kSubscriberTopic = "/turtlesim_test/turtle1/pose";
    const char* const kPublisherTopic = "/turtlesim_test/turtle1/cmd_vel";
    constexpr int kQueueSize = 1000;

    ros::Subscriber sub = node->subscribe(kSubscriberTopic, kQueueSize, movement::poseCallback);
    ros::Subscriber spy_sub = node->subscribe(kSubscriberTopic, kQueueSize, updatePose);
    ros::Publisher pub = node->advertise<geometry_msgs::Twist>(kPublisherTopic, kQueueSize);

    movement::goToGoal(pub, goal_pose, distance_tolerance, loop_frequency);
    EXPECT_THAT(real_turtle_pose.x,
                ::testing::AllOf(::testing::Ge(goal_pose.x - distance_tolerance),
                                 ::testing::Le(goal_pose.x + distance_tolerance)));
    EXPECT_THAT(real_turtle_pose.y,
                ::testing::AllOf(::testing::Ge(goal_pose.y - distance_tolerance),
                                 ::testing::Le(goal_pose.y + distance_tolerance)));


    goal_pose.x = 9;
    goal_pose.y = 9;
    goal_pose.theta = 0;
    movement::goToGoal(pub, goal_pose, distance_tolerance, loop_frequency);
    EXPECT_THAT(real_turtle_pose.x,
                ::testing::AllOf(::testing::Ge(goal_pose.x - distance_tolerance),
                                 ::testing::Le(goal_pose.x + distance_tolerance)));
    EXPECT_THAT(real_turtle_pose.y,
                ::testing::AllOf(::testing::Ge(goal_pose.y - distance_tolerance),
                                 ::testing::Le(goal_pose.y + distance_tolerance)));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "geometry_test");
    ros::NodeHandle nh;
    node = &nh;
    return RUN_ALL_TESTS();
}
