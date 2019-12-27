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
// Created on: Dec. 22th 2017
// Last change: Dec. 22th, 2017

#include <rob_comm.h>

using namespace std;

RobComm::RobComm() {
}

RobComm::~RobComm() {
}
//***************************************************************************
void RobComm::SetRightJoints(float * j)
{
ROS_INFO("send testing4");
    int i;
    for(i=0;i<3;i++)
    {
         Rx_buf_right[i] = j[i];
    }
	
	ROS_INFO("I sent [%.2lf][%.2lf][%.2lf]", 
              Rx_buf_right[0],Rx_buf_right[1],Rx_buf_right[2]);

	
}
//***************************************************************************
void RobComm::SetLeftJoints(float * j)
{
ROS_INFO("send testing3");
    int i;
    for(i=0;i<3;i++)
    {
         Rx_buf_left[i] = j[i];
    }
	
	ROS_INFO("I sent [%.2lf][%.2lf][%.2lf]", 
              Rx_buf_left[0],Rx_buf_left[1],Rx_buf_left[2]);
	
}

//***************************************************************************
void RobComm::SetLeftlegJoints(float * j)
{

    int i;
    for(i=0;i<5;i++)
    {
         Rx_buf_left_leg[i] = j[i];
    }
	
	ROS_INFO("I sent [%.2lf][%.2lf][%.2lf][%.2lf][%.2lf]", 
              Rx_buf_left_leg[0],Rx_buf_left_leg[1],Rx_buf_left_leg[2],Rx_buf_left_leg[3],Rx_buf_left_leg[4]);
	
}

//***************************************************************************
void RobComm::SetRightlegJoints(float * j)
{

    int i;
    for(i=0;i<5;i++)
    {
         Rx_buf_right_leg[i] = j[i];
    }
	
	ROS_INFO("I sent [%.2lf][%.2lf][%.2lf][%.2lf][%.2lf]", 
              Rx_buf_right_leg[0],Rx_buf_right_leg[1],Rx_buf_right_leg[2],Rx_buf_right_leg[3],Rx_buf_right_leg[4]);
	
}

//***************************************************************************
void RobComm::GetJoints(float * j)
{


}
