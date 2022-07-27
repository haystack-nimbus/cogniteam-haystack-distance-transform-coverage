/*
 * MoveBaseController.h
 *
 *  Created on: Mar 7, 2016
 *      Author: blackpc
 */

#ifndef INCLUDE_PENGO_SCANNING_MOVEBASECONTROLLER_H_
#define INCLUDE_PENGO_SCANNING_MOVEBASECONTROLLER_H_


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


/**
 * Wrapper of move_base's actionlib client for convenient way to send goals and get results
 */
class MoveBaseController {

public:

	MoveBaseController();

	virtual ~MoveBaseController();

public:

	/**
	 * Sending a goal to move_base
	 * @note This method doesn't wait until navigation is finished, use @see MoveBaseController::wait for that purpose
	 * @param goal
	 */
	void navigate(const geometry_msgs::PoseStamped& goal);

	/**
	 * Waits for move_base to finish
	 * @return True if navigation succeeded, false otherwise
	 */
	bool wait();

	/**
	 * Waits for the ActionServer to connect to this client
	 * @param timeout
	 * @return
	 */
	bool waitForServer(const ros::Duration& timeout = ros::Duration(0, 0));

	/**
	 * Cancels navigation process
	 */
	void cancelNavigation();

public:

	MoveBaseClient moveBaseClient_;

private:

	/**
	 * Creates actionlib's move_base goal message
	 * @param goal Goal pose
	 * @return
	 */
	move_base_msgs::MoveBaseGoal createGoalMessage(
			const geometry_msgs::PoseStamped& goal) const;

	/**
	 * Actionlib activate callback
	 */
	void moveBaseActivateCallback();

	/**
	 * Actionlib feedback callback
	 * @param feedback
	 */
	void moveBaseFeedbackCallback(
			const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);

	/**
	 * Actionlib done callback
	 * @param state
	 * @param result
	 */
	void moveBaseDoneCallback(
			const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResult::ConstPtr& result);

};

#endif /* INCLUDE_PENGO_SCANNING_MOVEBASECONTROLLER_H_ */
