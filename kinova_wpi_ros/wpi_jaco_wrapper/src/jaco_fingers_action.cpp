/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: jaco_fingers_action.cpp
 *  Desc: Class for moving/querying jaco arm fingers.
 *  Auth: Jeff Schmidt
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */
#include "wpi_jaco_wrapper/jaco_arm_trajectory_node.h"

namespace jaco
{

void JacoArmTrajectoryController::execute_fingers(const wpi_jaco_msgs::SetFingersPositionGoalConstPtr &goal)
{
    wpi_jaco_msgs::SetFingersPositionFeedback feedback;
    wpi_jaco_msgs::SetFingersPositionResult result;
    FingerAngles current_finger_positions;
    ros::Time current_time = ros::Time::now();

    try
    {
        getFingerPositions(current_finger_positions);

        finger_last_nonstall_time_ = current_time;
        finger_last_nonstall_positions_ = current_finger_positions;

        FingerAngles target(goal->fingers);
        target.Finger1 = std::min(gripper_closed_, target.Finger1);
        target.Finger2 = std::min(gripper_closed_, target.Finger2);
        target.Finger3 = std::min(gripper_closed_, target.Finger3);
        setFingerPositions(target);

        // Loop until the action completed, is preempted, or fails in some way.
        // timeout is left to the caller since the timeout may greatly depend on
        // the context of the movement.
        while (true)
        {
            ros::spinOnce();

            if (fingers_server_->isPreemptRequested() || !ros::ok())
            {
                result.fingers = current_finger_positions.constructFingersMsg();
                fingers_server_->setPreempted(result);
                return;
            }

            getFingerPositions(current_finger_positions);
            current_time = ros::Time::now();
            feedback.fingers = current_finger_positions.constructFingersMsg();
            fingers_server_->publishFeedback(feedback);

            if (target.isCloseToOther(current_finger_positions, finger_error_threshold_))
            {
                // Check if the action has succeeeded
                result.fingers = current_finger_positions.constructFingersMsg();
                fingers_server_->setSucceeded(result);
                return;
            }
            else if (!finger_last_nonstall_positions_.isCloseToOther(current_finger_positions, finger_stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                finger_last_nonstall_time_ = current_time;
                finger_last_nonstall_positions_ = current_finger_positions;
            }
            else if ((current_time - finger_last_nonstall_time_).toSec() > finger_stall_interval_seconds_)
            {
                // Check if the full stall condition has been meet
                result.fingers = current_finger_positions.constructFingersMsg();
                fingers_server_->setPreempted(result);
                return;
            }

            ros::Rate(finger_update_rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result.fingers = current_finger_positions.constructFingersMsg();
        ROS_ERROR_STREAM(e.what());
        fingers_server_->setAborted(result);
    }
}

/*!
 * \brief Sets the finger positions
 */
void JacoArmTrajectoryController::setFingerPositions(const FingerAngles &fingers, int timeout, bool push)
{
    double f1 = fingers.Finger1/finger_scale_;
    double f2 = fingers.Finger2/finger_scale_;
    double f3 = fingers.Finger3/finger_scale_;
    boost::recursive_mutex::scoped_lock lock(api_mutex);

    if (eStopEnabled)
    {
        ROS_INFO("The fingers could not be set because the arm is stopped");
        return;
    }

    TrajectoryPoint jaco_position;
    jaco_position.InitStruct();
    memset(&jaco_position, 0, sizeof(jaco_position));  // zero structure

    //startAPI();

    SetAngularControl();

    // Initialize Cartesian control of the fingers
    jaco_position.Position.HandMode = POSITION_MODE;
    jaco_position.Position.Type = ANGULAR_POSITION;
    jaco_position.Position.Fingers.Finger1 = f1;
    jaco_position.Position.Fingers.Finger2 = f2;
    jaco_position.Position.Fingers.Finger3 = f3;
    jaco_position.Position.Delay = 0.0;
    jaco_position.LimitationsActive = 0;

    AngularPosition jaco_angles;
    memset(&jaco_angles, 0, sizeof(jaco_angles));  // zero structure

    GetAngularPosition(jaco_angles);

    jaco_position.Position.Actuators = jaco_angles.Actuators;
/*
    // When loading a cartesian position for the fingers, values are required for the arm joints
    // as well or the arm goes nuts.  Grab the current position and feed it back to the arm.
    wpi_jaco_msgs::GetCartesianPosition gp;
    getCartesianPosition(gp.request, gp.response);
    jaco_position.Position.CartesianPosition.X = gp.response.pos.linear.x;
    jaco_position.Position.CartesianPosition.Y = gp.response.pos.linear.y;
    jaco_position.Position.CartesianPosition.Z = gp.response.pos.linear.z;
    jaco_position.Position.CartesianPosition.ThetaX = gp.response.pos.angular.x;
    jaco_position.Position.CartesianPosition.ThetaY = gp.response.pos.angular.y;
    jaco_position.Position.CartesianPosition.ThetaZ = gp.response.pos.angular.z;
*/
    SendAdvanceTrajectory(jaco_position);
}

/*!
 * \brief API call to obtain the current finger positions.
 */
void JacoArmTrajectoryController::getFingerPositions(FingerAngles &fingers)
{

    AngularPosition position_data;
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetAngularPosition(position_data);
    fingers = FingerAngles(position_data.Fingers.Finger1*finger_scale_,
                           position_data.Fingers.Finger2*finger_scale_,
                           position_data.Fingers.Finger3*finger_scale_);
}


}  // namespace jaco
