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
// editing by qibiao huang : Nov.22th,2019
// Author qibiao huang

#include <rob_control.h>

#include <list>

//#include "armplaning_client.h"

using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> R_TrajectoryServer;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> L_TrajectoryServer;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> R_Leg_TrajectoryServer;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> L_Leg_TrajectoryServer;


std::list<robotState> r_targetPointList;		// list of right arm points to move to
std::list<robotState> l_targetPointList;		// list of left arm points to move to

std::list<legrobotState> r_leg_targetPointList;		// list of right leg points to move to
std::list<legrobotState> l_leg_targetPointList;		// list of right leg points to move to

//define the constant value
double deg2rad = 3.14159/180.0;
double rad2deg = 180.0/3.14159;


//***************************************************************************
// Processing and JointTrajectoryAction of left leg
void l_leg_executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& l_leg_goal, L_Leg_TrajectoryServer* l_leg_as)
{
  //double rad2deg = 180.0/3.14159; transform rad to degree
  legrobotState llegs;

  float lastDuration = 0.0;

  int leg_nrOfJoints = l_leg_goal->trajectory.points.size();		// Number of points to add
  for(int i=0; i<leg_nrOfJoints; i++)
  {
	  llegs.j[0] = l_leg_goal->trajectory.points[i].positions[0]*rad2deg;	// ros values come in rad, internally we work in degree
	  llegs.j[1] = l_leg_goal->trajectory.points[i].positions[1]*rad2deg;
	  llegs.j[2] = l_leg_goal->trajectory.points[i].positions[2]*rad2deg;
	  llegs.j[3] = l_leg_goal->trajectory.points[i].positions[3]*rad2deg;
	  llegs.j[4] = l_leg_goal->trajectory.points[i].positions[4]*rad2deg;
	  
	  float dtmp = l_leg_goal->trajectory.points[i].time_from_start.toSec();
	  llegs.duration = dtmp - lastDuration;// time_from_start is adding up, these values are only for the single motion
	  lastDuration = dtmp;

	  l_leg_targetPointList.push_back(llegs);//push_back() 在list的末尾添加一个元素 
  }
  l_leg_as->setSucceeded();

  //debug msg
 ROS_INFO("left leg receive data: %f %f %f %f %f ,duration: %f", llegs.j[0],llegs.j[1],llegs.j[2],llegs.j[3],llegs.j[4],llegs.duration);
  
}//***************************************************************************
// Processing and JointTrajectoryAction
void r_leg_executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& r_leg_goal, R_Leg_TrajectoryServer* r_leg_as)
{
  //double rad2deg = 180.0/3.14159; transform rad to degree
  legrobotState rlegs;

  float lastDuration = 0.0;

  int leg_nrOfJoints = r_leg_goal->trajectory.points.size();		// Number of points to add
  for(int i=0; i<leg_nrOfJoints; i++)
  {
	  rlegs.j[0] = r_leg_goal->trajectory.points[i].positions[0]*rad2deg;	// ros values come in rad, internally we work in degree
	  rlegs.j[1] = r_leg_goal->trajectory.points[i].positions[1]*rad2deg;
	  rlegs.j[2] = r_leg_goal->trajectory.points[i].positions[2]*rad2deg;
	  rlegs.j[3] = r_leg_goal->trajectory.points[i].positions[3]*rad2deg;
	  rlegs.j[4] = r_leg_goal->trajectory.points[i].positions[4]*rad2deg;
	  
	  float dtmp = r_leg_goal->trajectory.points[i].time_from_start.toSec();
	  rlegs.duration = dtmp - lastDuration;// time_from_start is adding up, these values are only for the single motion
	  lastDuration = dtmp;

	  r_leg_targetPointList.push_back(rlegs);//push_back() 在list的末尾添加一个元素 
	  ROS_INFO("receive the goal sucess!");
  }
  r_leg_as->setSucceeded();

  //debug msg
 ROS_INFO("right leg receive data: %f %f %f %f %f ,duration: %f", rlegs.j[0],rlegs.j[1],rlegs.j[2],rlegs.j[3],rlegs.j[4],rlegs.duration);
  
}

//***************************************************************************
// Processing and JointTrajectoryAction
void r_executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& r_goal, R_TrajectoryServer* r_as)
{
  //double rad2deg = 180.0/3.14159; transform rad to degree
  robotState rs;

  float lastDuration = 0.0;

  int nrOfPoints = r_goal->trajectory.points.size();		// Number of points to add
  for(int i=0; i<nrOfPoints; i++)
  {
	  rs.j[0] = r_goal->trajectory.points[i].positions[0]*rad2deg;	// ros values come in rad, internally we work in degree
	  rs.j[1] = r_goal->trajectory.points[i].positions[1]*rad2deg;
	  rs.j[2] = r_goal->trajectory.points[i].positions[2]*rad2deg;
//	  rs.j[3] = r_goal->trajectory.points[i].positions[3]*rad2deg;
//	  rs.j[4] = r_goal->trajectory.points[i].positions[4]*rad2deg;
	  
	  float dtmp = r_goal->trajectory.points[i].time_from_start.toSec();
	  rs.duration = dtmp - lastDuration;// time_from_start is adding up, these values are only for the single motion
	  lastDuration = dtmp;

	  r_targetPointList.push_back(rs);//push_back() 在list的末尾添加一个元素 
	  ROS_INFO("receive the goal sucess!");
  }
  r_as->setSucceeded();

  //debug msg
 ROS_INFO("right arm receive data: %f %f %f  ,duration: %f", rs.j[0],rs.j[1],rs.j[2],rs.duration);
  
}
//***************************************************************************
// Processing and JointTrajectoryAction
void l_executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& l_goal, L_TrajectoryServer* l_as)
{
//double rad2deg = 180.0 / 3.141;
 robotState ls;

 float lastDuration = 0.0;

  int nrOfPoints = l_goal->trajectory.points.size();		// Number of trajectory points to add
  for(int i=0; i<nrOfPoints; i++){
	  ls.j[0] = l_goal->trajectory.points[i].positions[0]*rad2deg;	// followjointtrajectory values defined in rad, internally we work in degree
// we should convert rad to degree 
	  ls.j[1] = l_goal->trajectory.points[i].positions[1]*rad2deg;
	  ls.j[2] = l_goal->trajectory.points[i].positions[2]*rad2deg;
//	  ls.j[3] = l_goal->trajectory.points[i].positions[3]*rad2deg;
//	  ls.j[4] = l_goal->trajectory.points[i].positions[4]*rad2deg;
	  
	  float dtmp = l_goal->trajectory.points[i].time_from_start.toSec();
	  ls.duration = dtmp - lastDuration;// time_from_start is adding up, these values are only for the single motion
	  lastDuration = dtmp;

	  l_targetPointList.push_back(ls);//push_back() 在list的末尾添加一个元素 
  }
  l_as->setSucceeded();

  //debug msg
 ROS_INFO("left arm receive: %f %f %f ,duration: %f", ls.j[0],ls.j[1],ls.j[2],ls.duration);
//*/
}

//*************************************************************************
void quit(int sig)
{
  ros::shutdown();
  exit(0);
}

//******************** MAIN ************************************************
int main(int argc, char** argv)
{
	ros::init(argc, argv, "rob_mover");
	ros::NodeHandle n2;

	//Start  rigth_arm ActionServer for JointTrajectoryActions from MoveIT
	R_TrajectoryServer r_tserver(n2, "yanshee/right_arm_controller/follow_joint_trajectory", boost::bind(&r_executeTrajectory, _1, &r_tserver), false);
  	ROS_INFO("right_arm_controller: TrajectoryActionServer: Starting");
  	r_tserver.start();

	//Start  left_arm ActionServer for JointTrajectoryActions from MoveIT
	L_TrajectoryServer l_tserver(n2, "yanshee/left_arm_controller/follow_joint_trajectory", boost::bind(&l_executeTrajectory, _1, &l_tserver), false);
  	ROS_INFO("left_arm_controller:TrajectoryActionServer: Starting");
  	l_tserver.start();

	//Start  left_leg ActionServer for JointTrajectoryActions from MoveIT
	L_Leg_TrajectoryServer l_leg_tserver(n2, "yanshee/left_leg_controller/follow_joint_trajectory", boost::bind(&l_leg_executeTrajectory, _1, &l_leg_tserver), false);
  	ROS_INFO("left_leg_controller:TrajectoryActionServer: Starting");
  	l_leg_tserver.start();

	//Start  right_leg ActionServer for JointTrajectoryActions from MoveIT
	R_Leg_TrajectoryServer r_leg_tserver(n2, "yanshee/right_leg_controller/follow_joint_trajectory", boost::bind(&r_leg_executeTrajectory, _1, &r_leg_tserver), false);
  	ROS_INFO("right_leg_controller:TrajectoryActionServer: Starting");
  	r_leg_tserver.start();

	// Start the robot
	rob_robots::rob_control robot;
	robot.init();	
	robot.mainLoop();		//spinning is done inside the main loop			

  	signal(SIGINT,quit);	
	return(0);
}


namespace rob_robots{
	//*************************************************************************************
	rob_control::~rob_control(){

	}
	//*************************************************************************************
	void rob_control::init(){
		ROS_INFO("...initing...");

		flag_stop_requested = false;
		//this deside the control msg send frequency 
        	cycleTime = 10.0;// in ms
		nrOfJoints = 3;
		// define the number of leg joint
		

		r_setPointState.j[0] =  0.0;	// values are initialized with 6 field to be usable for rob right arm 
		r_setPointState.j[1] =  0.0;
		r_setPointState.j[2] =  0.0;
	//	r_setPointState.j[3] =  0.0;
       	// 	r_setPointState.j[4] =  0.0;
		r_setPointState.duration = 0;


		l_setPointState.j[0] =  0.0;	// values are initialized with 6 field to be usable for rob left arm 
		l_setPointState.j[1] =  0.0;
		l_setPointState.j[2] =  0.0;
	//	l_setPointState.j[3] =  0.0;
        //	l_setPointState.j[4] =  0.0;
		l_setPointState.duration = 0;

		leg_nrOfJoints=5;


		l_leg_setPointState.j[0] =  0.0;	// values are initialized with 5 field to be usable for rob left leg 
		l_leg_setPointState.j[1] =  0.0;
		l_leg_setPointState.j[2] =  0.0;
		l_leg_setPointState.j[3] =  0.0;
        	l_leg_setPointState.j[4] =  0.0;
		l_leg_setPointState.duration = 0;

		r_leg_setPointState.j[0] =  0.0;	// values are initialized with 5 field to be usable for rob right leg 
		r_leg_setPointState.j[1] =  0.0;
		r_leg_setPointState.j[2] =  0.0;
		r_leg_setPointState.j[3] =  0.0;
        	r_leg_setPointState.j[4] =  0.0;
		r_leg_setPointState.duration = 0;


		// when starting up (or when reading the HW joint values) the target position has to be aligned with the setPoint position
		//subscribe the topic /joint_state and set the r_msgJointsCurrent.position to the current position
		for(int i=0; i<nrOfJoints; i++)
		{
			r_targetState.j[i] = r_setPointState.j[i];
			l_targetState.j[i] = l_setPointState.j[i];
		}
			
       		r_targetState.duration = 0.0;//init the timeStamp
		l_targetState.duration = 0.0;//init the timeStamp

		for(int j=0;j<3;j++)
		{
			r_controlData[j] = 0;
			l_controlData[j] = 0;
		}


		for(int i=0; i<leg_nrOfJoints; i++)
		{
			r_leg_targetState.j[i] = r_leg_setPointState.j[i];
			l_leg_targetState.j[i] = l_leg_setPointState.j[i];
		}
			
        	r_leg_targetState.duration = 0.0;//init the timeStamp
		l_leg_targetState.duration = 0.0;//init the timeStamp

		for(int j=0;j<5;j++)
		{
			r_leg_controlData[j] = 0;
			l_leg_controlData[j] = 0;
		}

		// Publish the current joint states

		// Publish the current　double arm joint states
		//when connect to the true robot, the initial position must be current position
		DoubleArmSentData.header.stamp = ros::Time::now();
		DoubleArmSentData.name.resize(16);
		DoubleArmSentData.position.resize(16);
		DoubleArmSentData.name[0] ="left_arm1_joint";
		DoubleArmSentData.position[0] = 0.0;
		DoubleArmSentData.name[1] ="left_arm2_joint";
		DoubleArmSentData.position[1] = 0.0;
        	DoubleArmSentData.name[2] ="left_arm3_joint";
		DoubleArmSentData.position[2] = 0.0;
		DoubleArmSentData.name[3] ="right_arm1_joint";
		DoubleArmSentData.position[3] = 0.0;
		DoubleArmSentData.name[4] ="right_arm2_joint";
		DoubleArmSentData.position[4] = 0.0;
        	DoubleArmSentData.name[5] ="right_arm3_joint";
		DoubleArmSentData.position[5] = 0.0;
		DoubleArmSentData.name[6] ="left_leg1_joint";
		DoubleArmSentData.position[6] = 0.0;
		DoubleArmSentData.name[7] ="left_leg2_joint";
		DoubleArmSentData.position[7] = 0.0;
        	DoubleArmSentData.name[8] ="left_leg3_joint";
		DoubleArmSentData.position[8] = 0.0;
		DoubleArmSentData.name[9] ="left_leg4_joint";
		DoubleArmSentData.position[9] = 0.0;
		DoubleArmSentData.name[10] ="left_leg5_joint";
		DoubleArmSentData.position[10] = 0.0;
        	DoubleArmSentData.name[11] ="right_leg1_joint";
		DoubleArmSentData.position[11] = 0.0;
		DoubleArmSentData.name[12] ="right_leg2_joint";
		DoubleArmSentData.position[12] = 0.0;
		DoubleArmSentData.name[13] ="right_leg3_joint";
		DoubleArmSentData.position[13] = 0.0;
        	DoubleArmSentData.name[14] ="right_leg4_joint";
		DoubleArmSentData.position[14] = 0.0;
		DoubleArmSentData.name[15] ="right_leg5_joint";
		DoubleArmSentData.position[15] = 0.0;

		servo_data.angles.resize(17);
		//right arm
		servo_data.angles[0]=int(90*2048/180);
		servo_data.angles[1]=int(130*2048/180);
		servo_data.angles[2]=int(179*2048/180);
		//left arm
		servo_data.angles[3]=int(90*2048/180);
		servo_data.angles[4]=int(40*2048/180);
		servo_data.angles[5]=int(15*2048/180);

		servo_data.angles[6]=int(90*2048/180);
		servo_data.angles[7]=int(60*2048/180);
		servo_data.angles[8]=int(76*2048/180);
		servo_data.angles[9]=int(110*2048/180);
		servo_data.angles[10]=int(90*2048/180);
		servo_data.angles[11]=int(90*2048/180);
		servo_data.angles[12]=int(120*2048/180);
		servo_data.angles[13]=int(104*2048/180);
		servo_data.angles[14]=int(70*2048/180);
		servo_data.angles[15]=int(90*2048/180);
		servo_data.angles[16]=int(90*2048/180);

		pubJoints = n.advertise<sensor_msgs::JointState>("/yanshee/joint_states", 1);
		pubJointsValue = n.advertise<ubt_msgs::angles_set>("/hal_angles_set", 1000);

	}
	//*************************************************************************************


	void rob_control::mainLoop()
	{
  		ROS_INFO("Starting Mover Main Loop");
		
		//output angle data to a txt file
		int i = 0;
  		
 	 	for(;;)
  		{
			MotionGeneration();			// Generate the joint motion 
			CommunicationHW();			// Forward the new setpoints to the hardware

			if(flag_stop_requested)
				break;

			ros::spinOnce();
			ros::Duration(cycleTime/1000.0).sleep();		// main loop with 20 Hz.(50/1000=0.05s=50ms)
//loop_rate();發佈頻率
						
  		}

		ROS_INFO("Closing Mover Main Loop");

	} 
	//************************************************************************************
	//
	void rob_control::MotionGeneration()
	{
		int i=0;
		int j=0;

		//printf("r_targetPointList.size() = %ld \n",r_targetPointList.size());   
		if(r_targetPointList.size() > 0)
		{
			ROS_INFO("testing4");
			r_targetState = r_targetPointList.front();
			r_targetPointList.pop_front();

			for(int i=0; i<nrOfJoints; i++)
			{
				r_setPointState.j[i] = r_targetState.j[i];
			}
			r_setPointState.duration = r_targetState.duration;
			
			for(j=0;j<3;j++)
			{
				r_controlData[j] = r_setPointState.j[j];
			}
			r_controlData[3] = r_setPointState.duration;
		}

		if(l_targetPointList.size() > 0)
		{


			l_targetState = l_targetPointList.front();
			l_targetPointList.pop_front();

			for(int i=0; i<nrOfJoints; i++)
			{
				l_setPointState.j[i] = l_targetState.j[i];
			}
			l_setPointState.duration = l_targetState.duration;
			
			for(j=0;j<3;j++)
			{
				l_controlData[j] = l_setPointState.j[j];
			}
			l_controlData[3] = l_setPointState.duration;
		}

		if(l_leg_targetPointList.size() > 0)
		{

			l_leg_targetState = l_leg_targetPointList.front();
			l_leg_targetPointList.pop_front();

			for(int i=0; i<leg_nrOfJoints; i++)
			{
				l_leg_setPointState.j[i] = l_leg_targetState.j[i];
			}
			l_leg_setPointState.duration = l_leg_targetState.duration;
			
			for(j=0;j<5;j++)
			{
				l_leg_controlData[j] = l_leg_setPointState.j[j];
			}
			l_leg_controlData[5] = l_leg_setPointState.duration;
		}

		if(r_leg_targetPointList.size() > 0)
		{

			r_leg_targetState = r_leg_targetPointList.front();
			r_leg_targetPointList.pop_front();

			for(int i=0; i<leg_nrOfJoints; i++)
			{
				r_leg_setPointState.j[i] = r_leg_targetState.j[i];
			}
			r_leg_setPointState.duration = r_leg_targetState.duration;
			
			for(j=0;j<5;j++)
			{
				r_leg_controlData[j] = r_leg_setPointState.j[j];
			}
			r_leg_controlData[5] = r_leg_setPointState.duration;
		}



	}
	//************************************************************************************
	// Forward the new setpoints to the hardware

	void rob_control::CommunicationHW()
	{

		int i = 0;
		for(i=0;i<3;i++)//write right arm control data
		{
			r_executeData[i] = r_controlData[i];
			l_executeData[i] = l_controlData[i];
		}

		if(r_executeData[0]!=r_executeData_old[0] && r_executeData[1]!=r_executeData_old[1])
		{
			
			//add send mesage command here	
			//************************************

			Comm.SetRightJoints(r_executeData);//send new target angle 

			//************************************

			ROS_INFO("Right arm MSG [%f][%f][%f]", r_executeData[0],r_executeData[1],r_executeData[2]);

			//save the date to old
			for (int i=0;i<3;i++)
			{
				r_executeData_old[i] = r_executeData[i];
			}

			R_CommunicationROS();			// Publish the joint states and error info
		}

		if(l_executeData[0]!=l_executeData_old[0] && l_executeData[1]!=l_executeData_old[1])
		{
			//add send mesage command here		
			//************************************

			Comm.SetLeftJoints(l_executeData);//send new target angle 

			//************************************


			ROS_INFO("Left arm MSG [%f][%f][%f]", l_executeData[0],l_executeData[1],l_executeData[2]);

			//save the date to old
			for (int i=0;i<3;i++)
			{
				l_executeData_old[i] = l_executeData[i];
			}

			L_CommunicationROS();			// Publish the joint states and error info
		}

		
		for(i=0;i<5;i++)//write right & left leg control data
		{
			r_leg_executeData[i] = r_leg_controlData[i];
			l_leg_executeData[i] = l_leg_controlData[i];
		}



		if(r_leg_executeData[0]!=r_leg_executeData_old[0] && r_leg_executeData[1]!=r_leg_executeData_old[1])
		{


ROS_INFO("r_leg_executeData_old [%f][%f]", r_leg_executeData_old[0],r_leg_executeData_old[1]);						
			//add send mesage command here	
			//************************************

			Comm.SetRightlegJoints(r_leg_executeData);//send new target angle 

			//************************************

			ROS_INFO("Right leg MSG [%f][%f][%f][%f][%f]", r_leg_executeData[0],r_leg_executeData[1],r_leg_executeData[2],r_leg_executeData[3],r_leg_executeData[4]);

			//save the date to old
			for (int i=0;i<5;i++)
			{
				r_leg_executeData_old[i] = r_leg_executeData[i];
			}

			R_leg_CommunicationROS();			// Publish the joint states and error info
		}


		if(l_leg_executeData[0]!=l_leg_executeData_old[0] && l_leg_executeData[1]!=l_leg_executeData_old[1])
		{
			
			//add send mesage command here	
			//************************************

			Comm.SetLeftlegJoints(l_leg_executeData);//send new target angle 

			//************************************

			ROS_INFO("left leg MSG [%f][%f][%f][%f][%f]", l_leg_executeData[0],l_leg_executeData[1],l_leg_executeData[2],l_leg_executeData[3],l_leg_executeData[4]);

			//save the date to old
			for (int i=0;i<5;i++)
			{
				l_leg_executeData_old[i] = l_leg_executeData[i];
			}

			L_leg_CommunicationROS();			// Publish the joint states and error info
		}






	}
	//************************************************************************************
	// forward the current joints to RViz etc
	void rob_control::L_CommunicationROS()
	{

		DoubleArmSentData.header.stamp = ros::Time::now();
		DoubleArmSentData.position[0] = l_executeData[0];		// Robot SER communication works in degree
		DoubleArmSentData.position[1] = l_executeData[1];
		DoubleArmSentData.position[2] = l_executeData[2];
	//	DoubleArmSentData.position[3] = l_executeData[3]*deg2rad;
        //  	DoubleArmSentData.position[4] = l_executeData[4]*deg2rad;

//pub the servo data  +130 +179
		servo_data.angles[3]=int((l_executeData[0]+90)*2048/180);
		servo_data.angles[4]=int((l_executeData[1])*2048/180);
		servo_data.angles[5]=int((l_executeData[2]+90)*2048/180);

	//	pubJoints.publish(l_msgJointsCurrent);	// ROS communication works in Radian
		pubJoints.publish(DoubleArmSentData);	// ROS communication works in Radian
		pubJointsValue.publish(servo_data);

	}
	//************************************************************************************
	// forward the current joints to RViz etc
	void rob_control::R_CommunicationROS()
	{

		DoubleArmSentData.header.stamp = ros::Time::now();
		DoubleArmSentData.position[3] = r_executeData[0];		// Robot SER communication works in degree
		DoubleArmSentData.position[4] = r_executeData[1];
		DoubleArmSentData.position[5] = r_executeData[2];

		servo_data.angles[0]=int((r_executeData[0]+90)*2048/180);
		servo_data.angles[1]=int((r_executeData[1])*2048/180);
		servo_data.angles[2]=int((r_executeData[2]+90)*2048/180);

		pubJoints.publish(DoubleArmSentData);	// ROS communication works in Radian
		pubJointsValue.publish(servo_data);
	}


	// forward the current joints to RViz etc
	void rob_control::L_leg_CommunicationROS()
	{


		DoubleArmSentData.header.stamp = ros::Time::now();
		DoubleArmSentData.position[11] = l_leg_executeData[0];		// Robot SER communication works in degree
		DoubleArmSentData.position[12] = l_leg_executeData[1];
		DoubleArmSentData.position[13] = l_leg_executeData[2];
		DoubleArmSentData.position[14] = l_leg_executeData[3];
		DoubleArmSentData.position[15] = l_leg_executeData[4];

//data should be match to the true data
		servo_data.angles[11]=int((l_leg_executeData[0]+90)*2048/180);
		servo_data.angles[12]=int((l_leg_executeData[1])*2048/180);
		servo_data.angles[13]=int((l_leg_executeData[2]+90)*2048/180);
		servo_data.angles[14]=int((l_leg_executeData[3]+90)*2048/180);
		servo_data.angles[15]=int((l_leg_executeData[4]+90)*2048/180);

		pubJoints.publish(DoubleArmSentData);	// ROS communication works in Radian
		pubJointsValue.publish(servo_data);
	}


	void rob_control::R_leg_CommunicationROS()
	{

		DoubleArmSentData.header.stamp = ros::Time::now();
		DoubleArmSentData.position[6] = r_leg_executeData[0];		// Robot SER communication works in degree
		DoubleArmSentData.position[7] = r_leg_executeData[1];
		DoubleArmSentData.position[8] = r_leg_executeData[2];
		DoubleArmSentData.position[9] = r_leg_executeData[3];
		DoubleArmSentData.position[10] = r_leg_executeData[4];

//data should be match to the true data
		servo_data.angles[6]=int((r_leg_executeData[0]+90)*2048/180);
		servo_data.angles[7]=int((r_leg_executeData[1])*2048/180);
		servo_data.angles[8]=int((r_leg_executeData[2]+90)*2048/180);
		servo_data.angles[9]=int((r_leg_executeData[3]+90)*2048/180);
		servo_data.angles[10]=int((r_leg_executeData[4]+90)*2048/180);

		pubJoints.publish(DoubleArmSentData);	// ROS communication works in Radian
		pubJointsValue.publish(servo_data);
	}

}

