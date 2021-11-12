#include <gtest/gtest.h>
#include <string>
#include <ros/ros.h>
#include <robot_behavior_engine.h>
#include <robot_engine_observer.h>
#include <bp_robot_ctrl.h>

using namespace std;
using namespace bp;

class TestEngineObserver : public IRobotEngineObserver
{
public:
    ERESULT OnNaviDone(const std::string& task_id, int32 state, const std::string& description) {
        return E_OK;
    }
    ERESULT OnNaviActive(const std::string& task_id, bool active) {
        return E_OK;
    }
    ERESULT OnNaviProgress(const std::string& task_id, const Pose& curr_pose) {
        return E_OK;
    }
};

BpRobotCtrl robotCtrl_;
TestEngineObserver observer;

// Declare a test
TEST(TestSuite, testCase1)
{
  RobotBehaviorEngine& engine = RobotBehaviorEngine::GetInstance();
  ERESULT ret;
  ret = engine.Init(robotCtrl_, observer);
  EXPECT_EQ(ret, E_OK);
  ret = engine.Init(robotCtrl_, observer);
  EXPECT_EQ(ret, E_FALSE);
}

TEST(TestSuite, testCase2)
{
  RobotBehaviorEngine& engine = RobotBehaviorEngine::GetInstance();
  ERESULT ret;
  ret = engine.Init(robotCtrl_, observer);
  EXPECT_EQ(ret, E_FALSE);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
