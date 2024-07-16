#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
using namespace std;

class JointTrajectoryActionServer
{
public:
JointTrajectoryActionServer(std::string name):
		as_(nh_, name, false), action_name_(name)
{
	// register callback for goal
	as_.registerGoalCallback(boost::bind(&JointTrajectoryActionServer::goalCallback, this));
	as_.start();
}
~JointTrajectoryActionServer(void){}

// when a trajectory command comes, this function will be called.
void goalCallback()
{
	boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> goal;
	goal=as_.acceptNewGoal();
	cout<<"trajectory point size:"<< goal->trajectory.points.size()<<endl;
	// tell motion control hardware to execute
    // do something
	// when finished, return result
	ROS_INFO("!!!!!!");
	as_.setSucceeded(result_);
}
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::Result result_;
	std::string action_name_;
};

int main(int argc, char** argv)
{
	ros::init(argc,argv, "arm_controller");
	JointTrajectoryActionServer srv("execute_trajectory");
	ros::spin();
	return 0;
}