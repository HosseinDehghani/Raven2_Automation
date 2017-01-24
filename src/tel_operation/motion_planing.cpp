#include "motion_planing.h"
//#include "q_math.h"
#include "debug.h"
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Transform.h>
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
using namespace std;
tf::Vector3 tool_axis;
btTransform T03;
btTransform T04;
btTransform T05;


geometry_msgs::Transform set_end(btTransform tf_current)
{
	geometry_msgs::Transform msg;
	//msg = set_insertion(tf_current);
	//msg = set_roll(tf_current);
	//msg = set_wrist(tf_current);
	//msg = set_wrist2(tf_current);
	////msg = set_Z_axis(tf_current);
	return msg;
	
}
geometry_msgs::Transform set_current(btTransform tf_current)
{
	//// witing on a file
	double yaw_current, pitch_current, roll_current;
	tf_current.getBasis().getEulerYPR(yaw_current, pitch_current, roll_current);
	double temp[] = {tf_current.getBasis()[0][0], tf_current.getBasis()[0][1], tf_current.getBasis()[0][2],
			tf_current.getBasis()[1][0], tf_current.getBasis()[1][1], tf_current.getBasis()[1][2],
			tf_current.getBasis()[2][0], tf_current.getBasis()[2][1], tf_current.getBasis()[2][2],
			tf_current.getOrigin()[0], tf_current.getOrigin()[1], tf_current.getOrigin()[2],
			yaw_current, pitch_current, roll_current};
	write_file("/opt/raven_2/raven_ros/raven_2/data/actual.txt", 15, temp); //9 ori, 3 pos, 3:YPR
	//// end of writing
	
	geometry_msgs::Transform msg;
	tf::transformTFToMsg(tf_current, msg);
	return msg;
}
geometry_msgs::Transform set_increment(geometry_msgs::Transform msg_current, geometry_msgs::Transform msg_end)
{
	float p_tel = 1000;
	float q_tel = .1*M_PI/180;
	geometry_msgs::Transform increment;
	
	/// Translation
	increment.translation.x = msg_end.translation.x - msg_current.translation.x;
	increment.translation.y = msg_end.translation.y - msg_current.translation.y;
	increment.translation.z = msg_end.translation.z - msg_current.translation.z;


	/// Rotatino
	btMatrix3x3 m_end;
	tf::Quaternion q_end;
	q_end[0] = msg_end.rotation.x;
	q_end[1] = msg_end.rotation.y;
	q_end[2] = msg_end.rotation.z;
	q_end[3] = msg_end.rotation.w;
	//m_end.getRotation(q_end);
	btMatrix3x3 m_current;
	tf::Quaternion q_current;
	q_current[0] = msg_current.rotation.x;
	q_current[1] = msg_current.rotation.y;
	q_current[2] = msg_current.rotation.z;
	q_current[3] = msg_current.rotation.w;
	//m_current.getRotation(q_current);
	tf::Quaternion q_tmp = q_current.slerp(q_end,(1*3.14/180)/(2*q_current.angle(q_end)));
	tf::Quaternion dq = q_tmp *(inverse(q_current));
	increment.rotation.x = dq[0];
	increment.rotation.y = dq[1];
	increment.rotation.z = dq[2];
	increment.rotation.w = dq[3];
	
	
	/*if (abs(increment.translation.x) < p_tel and abs(increment.translation.y) < p_tel and abs(increment.translation.z) < p_tel and abs(q_end.getAngle() - q_current.getAngle()) < q_tel)
		return increment;
	else
		return msg_current;*/
	return increment;
}
geometry_msgs::Transform set_insertion(btTransform tf_current)
{

	tf::Vector3 pos_incr(0, 0, -10000);
	btTransform tf_incr;
	tf_incr.setIdentity();
	tf_incr.setOrigin(pos_incr);
	//tf_incr.setBasis(ori_incr);
	btTransform tf_end;
	tf_end = T03 * tf_incr * T03.inverse() * tf_current;		

	ROS_ERROR("\nT3\n");
	printf("\nT03\n"); print_btTransform(T03 );

	//// witing on a file
	double yaw_current, pitch_current, roll_current;
	tf_current.getBasis().getEulerYPR(yaw_current, pitch_current, roll_current);
	double temp[] = {tf_current.getBasis()[0][0], tf_current.getBasis()[0][1], tf_current.getBasis()[0][2],
			tf_current.getBasis()[1][0], tf_current.getBasis()[1][1], tf_current.getBasis()[1][2],
			tf_current.getBasis()[2][0], tf_current.getBasis()[2][1], tf_current.getBasis()[2][2],
			tf_current.getOrigin()[0], tf_current.getOrigin()[1], tf_current.getOrigin()[2],
			yaw_current, pitch_current, roll_current};
	write_file("/opt/raven_2/raven_ros/raven_2/data/desired.txt", 15, temp); //9 ori, 3 pos, 3:YPR
	//// end of writing

	geometry_msgs::Transform msg;
	tf::transformTFToMsg(tf_end, msg);
	return msg;
}



geometry_msgs::Transform set_roll(btTransform tf_current, double step)
{

	double d_theta = -step * M_PI/180;
	btMatrix3x3 ori_incr(	cos(d_theta),	-sin(d_theta), 0,
				sin(d_theta),	cos(d_theta), 0,
				0,		0,		1);
	tf::Vector3 pos_incr(0, 0, 0);
	btTransform tf_incr;
	tf_incr.setOrigin(pos_incr);
	tf_incr.setBasis(ori_incr);
	btTransform tf_end;
	tf_end = T04 * tf_incr * T04.inverse() * tf_current;		


	//// witing on a file
	double yaw_current, pitch_current, roll_current;
	tf_current.getBasis().getEulerYPR(yaw_current, pitch_current, roll_current);
	double temp[] = {tf_current.getBasis()[0][0], tf_current.getBasis()[0][1], tf_current.getBasis()[0][2],
			tf_current.getBasis()[1][0], tf_current.getBasis()[1][1], tf_current.getBasis()[1][2],
			tf_current.getBasis()[2][0], tf_current.getBasis()[2][1], tf_current.getBasis()[2][2],
			tf_current.getOrigin()[0], tf_current.getOrigin()[1], tf_current.getOrigin()[2],
			yaw_current, pitch_current, roll_current};
	write_file("/opt/raven_2/raven_ros/raven_2/data/desired.txt", 15, temp); //9 ori, 3 pos, 3:YPR
	//// end of writing
	


	geometry_msgs::Transform msg;
	tf::transformTFToMsg(tf_end, msg);
	return msg;
}
geometry_msgs::Transform set_wrist(btTransform tf_current, double step)
{
	double d_theta = -step * M_PI/180;
	btMatrix3x3 ori_incr(	cos(d_theta),	-sin(d_theta), 0,
				sin(d_theta),	cos(d_theta), 0,
				0,		0,		1);
	tf::Vector3 pos_incr(0, 0, 0);
	btTransform tf_incr;
	tf_incr.setOrigin(pos_incr);
	tf_incr.setBasis(ori_incr);
	btTransform tf_end;
	tf_end = T05 * tf_incr * T05.inverse() * tf_current;		
				
	//// witing on a file
	double yaw_current, pitch_current, roll_current;
	tf_current.getBasis().getEulerYPR(yaw_current, pitch_current, roll_current);
	double temp[] = {tf_current.getBasis()[0][0], tf_current.getBasis()[0][1], tf_current.getBasis()[0][2],
			tf_current.getBasis()[1][0], tf_current.getBasis()[1][1], tf_current.getBasis()[1][2],
			tf_current.getBasis()[2][0], tf_current.getBasis()[2][1], tf_current.getBasis()[2][2],
			tf_current.getOrigin()[0], tf_current.getOrigin()[1], tf_current.getOrigin()[2],
			yaw_current, pitch_current, roll_current};
	write_file("/opt/raven_2/raven_ros/raven_2/data/desired.txt", 15, temp); //9 ori, 3 pos, 3:YPR
	//// end of writing

	geometry_msgs::Transform msg;
	tf::transformTFToMsg(tf_end, msg);
	return msg;
}
geometry_msgs::Transform set_wrist2(btTransform tf_current, double step)
{

	double d_theta = -step * M_PI/180;
	btMatrix3x3 ori_incr(	cos(d_theta),	-sin(d_theta), 0,
				sin(d_theta),	cos(d_theta), 0,
				0,		0,		1);
	tf::Vector3 pos_incr(0, 0, 0);
	btTransform tf_incr;
	tf_incr.setOrigin(pos_incr);
	tf_incr.setBasis(ori_incr);
	btTransform tf_end;
	tf_end = tf_current * tf_incr;		
				
	//// witing on a file
	double yaw_current, pitch_current, roll_current;
	tf_current.getBasis().getEulerYPR(yaw_current, pitch_current, roll_current);
	double temp[] = {tf_current.getBasis()[0][0], tf_current.getBasis()[0][1], tf_current.getBasis()[0][2],
			tf_current.getBasis()[1][0], tf_current.getBasis()[1][1], tf_current.getBasis()[1][2],
			tf_current.getBasis()[2][0], tf_current.getBasis()[2][1], tf_current.getBasis()[2][2],
			tf_current.getOrigin()[0], tf_current.getOrigin()[1], tf_current.getOrigin()[2],
			yaw_current, pitch_current, roll_current};
	write_file("/opt/raven_2/raven_ros/raven_2/data/desired.txt", 15, temp); //9 ori, 3 pos, 3:YPR
	//// end of writing

	geometry_msgs::Transform msg;
	tf::transformTFToMsg(tf_end, msg);
	return msg;
}
geometry_msgs::Transform set_Z_axis(btTransform tf_current)// can be removed
{
	FILE *desired_YPR_file;
	tf::Vector3 origin_L;
	btMatrix3x3 basis_L;
	origin_L = tf_current.getOrigin();
	basis_L = tf_current.getBasis();
	
	tf::Quaternion q_cur , q_end, dq;
	double yaw_desired, pitch_desired, roll_desired;
	double d_theta = -1*M_PI/180;
	/*btMatrix3x3 ori_incr(1, 0, 0,
				0,	cos(d_theta),	-sin(d_theta),
				0,	sin(d_theta),	cos(d_theta));*/
	btMatrix3x3 ori_incr(	cos(d_theta),	-sin(d_theta), 0,
				sin(d_theta),	cos(d_theta), 0,
				0,		0,		1);
	/*btMatrix3x3 ori_incr(	cos(d_theta),	0,	-sin(d_theta),
				sin(d_theta),	0,	cos(d_theta),
				0,		1,		0);*/
	btMatrix3x3 m_end;
	m_end = basis_L * ori_incr;
	m_end.getRotation(q_end);
	m_end.getEulerYPR(yaw_desired, pitch_desired, roll_desired);
	desired_YPR_file = fopen("/opt/raven_2/raven_ros/raven_2/data/desired_YPR.txt" ,"a");
	fprintf(desired_YPR_file, "%f	%f	%f\n", yaw_desired, pitch_desired, roll_desired);
	fclose(desired_YPR_file);

	geometry_msgs::Transform msg;
	msg.translation.x = origin_L[0];
	msg.translation.y = origin_L[1];
	msg.translation.z = origin_L[2];
	msg.rotation.x = q_end[0];
	msg.rotation.y = q_end[1];
	msg.rotation.z = q_end[2];
	msg.rotation.w = q_end[3];
	return msg;
	
}


void set_tool_axis(tf::Vector3 tool_axis_)
{	

}
tf::Vector3 get_tool_axis()
{
	tf::Vector3 Z(0, 0, 1);
	tf::Vector3 tool_axis = T04 * Z;
	return tool_axis;

}


void set_T(btTransform T03_, btTransform T04_, btTransform T05_)
{
	T03 = T03_;
	T04 = T04_;
	T05 = T05_;
	//ROS_ERROR("\nT3\n"); print_btTransform(T03 );
}
btTransform get_T04()
{
	return T04;
}




