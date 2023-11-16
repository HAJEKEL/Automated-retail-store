#include "RPPick.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <retail_store_planning/PickAction.h>


/* The implementation of RPPick.h */
namespace KCL_rosplan {

	/* constructor */
	RPPickInterface::RPPickInterface(ros::NodeHandle &nh) : _nh(nh) {
		// perform setup
	}

	/* action dispatch callback */
	bool RPPickInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.
        typedef actionlib::SimpleActionClient<retail_store_planning::PickAction> PickClient;
		PickClient ac("pick_server", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the pick action server to come up");
        }

		retail_store_planning::PickGoal goal;
		
		/* The new picking action always uses aruco, so this
		   is not needed anymore
		// Get parameter to use or not use aruco markers
		bool use_aruco;
		_nh.getParam("/aruco/use_aruco", use_aruco);
		goal.use_aruco = use_aruco;
		*/

		std::map<std::string, double> april_tag;
		_nh.getParam("/objects/" + msg->parameters[2].value + "/april_tag", april_tag);
		goal.goal_id = april_tag["goal_id"];		

		ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the object is picked up!");
        else
            ROS_INFO("Albert failed to pick up the object for some reason...");

		// complete the action
		ROS_INFO("KCL: (%s) Pick Action completing.", msg->name.c_str());
		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_pick_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPPickInterface rpti(nh);

		rpti.runActionInterface();

		return 0;
	}
