/*********************************************************************
Copyright (c) <2018>, <Shawn Zhang>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the <organization>.
4. Neither the name of the <organization> nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY <Shawn Zhang> ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <Shawn Zhang> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *********************************************************************/
// Created on: Dec. 29th 2016
// Last change: Dec. 29th, 2016


#ifndef rob_control_H
#define rob_control_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>

#include <list>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <ubt_msgs/angles_set.h>


#include <actionlib/server/simple_action_server.h>//header is the standard action library to implement an action server node

#include <control_msgs/FollowJointTrajectoryAction.h>////The header is generated from the stored action files

#include "rob_comm.h"

struct robotState
{
	float j[3];		// joint position
	float duration;	// duration for motion; needed for actionServer
};

struct legrobotState
{
	float j[5];		// joint position
	float duration;	// duration for motion; needed for actionServer
};

namespace rob_robots
{

	// the struct stores information about the current robots state:
	// joint positions, cartesian position and error code

	class rob_control
	{

		private:

			bool flag_stop_requested;

			robotState r_setPointState;
			robotState r_targetState;
			robotState l_setPointState;
			robotState l_targetState;

			//add double-leg
			legrobotState r_leg_setPointState;
			legrobotState r_leg_targetState;
			legrobotState l_leg_setPointState;
			legrobotState l_leg_targetState;			

			int nrOfJoints; /* 5 */ //defined by arms

			int leg_nrOfJoints; /* 5 */   			

			double cycleTime; /* in ms */

			//control data def	
			float r_controlData[3];	/* final control data */
			float r_executeData[3];
			float l_controlData[3];	/* final control data */
			float l_executeData[3];
			float r_executeData_old[3];
			float l_executeData_old[3];
			float neck_controlData[2];

			float r_leg_controlData[5];	/* final control data */
			float r_leg_executeData[5];
			float l_leg_controlData[5];	/* final control data */
			float l_leg_executeData[5];
			float r_leg_executeData_old[5];
			float l_leg_executeData_old[5];


			RobComm Comm;

			ros::NodeHandle n;
			sensor_msgs::JointState r_msgJointsCurrent;	/* the current joints */
			sensor_msgs::JointState l_msgJointsCurrent;	/* the current joints */
			sensor_msgs::JointState DoubleArmSentData;	/* the current joints */

			ubt_msgs::angles_set servo_data; //send the servo data to Raspberry Pi

			ros::Publisher  pubJoints; /* publishes the current joint positions  */
			ros::Publisher  pubJointsValue; /* publishes the current joint positions  */

			void MotionGeneration();
			void CommunicationHW();
			void L_CommunicationROS();
			void R_CommunicationROS();

			void L_leg_CommunicationROS();
			void R_leg_CommunicationROS();
			
			
		public:
			~rob_control();
			void init();
			void mainLoop();				
	};

}

#endif
