/*
 * MoveBaseController.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: blackpc
 */



#include "../include/MoveBaseController.h"

MoveBaseController::MoveBaseController()
	: moveBaseClient_("move_base", true){

}

MoveBaseController::~MoveBaseController() {
	// TODO Auto-generated destructor stub
}

void MoveBaseController::navigate(const geometry_msgs::PoseStamped& goal) {
	move_base_msgs::MoveBaseGoal message = createGoalMessage(goal);
	moveBaseClient_.sendGoal(
			message,
			boost::bind(&MoveBaseController::moveBaseDoneCallback, this, _1, _2),
			boost::bind(&MoveBaseController::moveBaseActivateCallback, this),
			boost::bind(&MoveBaseController::moveBaseFeedbackCallback, this, _1));


}

move_base_msgs::MoveBaseGoal MoveBaseController::createGoalMessage(
		const geometry_msgs::PoseStamped& goal) const {
	move_base_msgs::MoveBaseGoal message;

	message.target_pose = goal;
	message.target_pose.header.stamp = ros::Time::now();

	return message;
}

void MoveBaseController::moveBaseActivateCallback() {
	ROS_DEBUG("MoveBase action activated");
}

void MoveBaseController::moveBaseFeedbackCallback(
		const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback) {
	ROS_DEBUG("MoveBase action feedback");
}

void MoveBaseController::moveBaseDoneCallback(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResult::ConstPtr& result) {
	ROS_DEBUG("MoveBase action done");
}

bool MoveBaseController::wait() {
	ROS_DEBUG("Waiting for move_base result...");
	moveBaseClient_.waitForResult();
	ROS_DEBUG("Finished waiting");

	return moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool MoveBaseController::waitForServer(const ros::Duration& timeout) {
	return moveBaseClient_.waitForServer(timeout);
}

void MoveBaseController::cancelNavigation()
{
	moveBaseClient_.cancelAllGoals();
}
