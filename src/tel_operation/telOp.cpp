#include "ros/ros.h"
#include "std_msgs/String.h"
#include "raven_2/raven_state.h"
#include "raven_2/raven_automove.h"
#include "raven_2/raven_input.h"
#include "raven_2/raven_tf.h"
#include "raven_2/telOp_mode.h"
#include <ros/transport_hints.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include "telOp.h"
//#include "q_math.h"
#include "motion_planing.h"
#include "debug.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h> // mkdir

using namespace std;

ros::Publisher pub_automove;
ros::Subscriber sub_ravenstate;
ros::Subscriber sub_jointstate;
ros::Subscriber sub_tf;
//ros::Subscriber sub_telopmode;
static double runcheck = true;
static double vel_safe = true;
static bool is_safe = true;
static bool init_status = false;
static bool init_IK = false;
double joint[6];


btTransform tf_current;
btTransform tf_initial;

int static ci = 0;
float theta_new=0;
float theta_old;


FILE *desired_ori_file, *fYPR, *desired_YPR_file;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "telOp");
	ros_initialization();

	mkdir("/opt/raven_2/raven_ros/raven_2/data", 0777);
	//float a[]={155,52};
//write_file("/opt/raven_2/raven_ros/raven_2/data/ali.txt", 2, a);

//exit(1);
	
	
	if (!grasp(200))
		ROS_ERROR("grasp DOne");
	if (!insertion(2))
		ROS_ERROR("Insertion DOne");
	if (!roll(45))
		ROS_ERROR("Roll DOne");
	if (!wrist(45))
		ROS_ERROR("Wrist DOne");
	if (!wrist2(90))
		ROS_ERROR("Wrist2 DOne");
	
 	return 0;
}




void ravenstateCallback(raven_2::raven_state msg)
{

	btTransform tf_msg ( btMatrix3x3 (msg.ori[9], msg.ori[10], msg.ori[11],
					  msg.ori[12], msg.ori[13], msg.ori[14],
					  msg.ori[15], msg.ori[16], msg.ori[17]),
			     btVector3 (msg.pos[3], msg.pos[4], msg.pos[5]) );
	
	set_tf_current(tf_msg);

	// initializing position and orientation
	if (ci==0)
	{

		ROS_ERROR("TF Initializing");
		tf_initial = tf_msg;
		ci++;
	}else
	{

	}


	float mvel[3];
	for (int i=0; i<3; i++)
	{
		mvel[i] = msg.mvel[i];
	}


	float vmax[7];
	float vmin[7];
	vmin[0] = -10000;
	vmin[1] = -10000;
	vmin[2] = -2000;
	vmax[0] = 10000;
	vmax[1] = 10000;
	vmax[2] = 2000;
	if (mvel[0]>vmin[0] and mvel[0]<vmax[0] and mvel[1]>vmin[1] and mvel[1]<vmax[1] and mvel[2]>vmin[2] and mvel[2]<vmax[2])
	{
		//is_safe = true;
		
	}
	else
	{
		is_safe = false;
		vel_safe=false;
		//ROS_ERROR("Speed Limits Error");
	}
	init_status = true;
	//ROS_ERROR("Start listing");
	
}




void jointstatesCallback(sensor_msgs::JointState msg)
{
	/*string position_out = "";

	float position[28];
	for (int i=0; i<28; i++)
	{

		position[i] = msg.position[i];
		std::ostringstream position_str;
		position_str << position[i];
		std::string cs8(position_str.str());
		position_out = position_out + cs8 + "	";

	}

	ofstream outfile8;
	outfile8.open("/opt/raven_2/raven_ros/raven_2/data/position.txt", std::ofstream::app);
	outfile8 << position_out << endl;
	
	//joint = {position[0], position[1], position[2], position[3], position[4], position[5]};


	//ROS_ERROR("%f", position[9]);
	float max[7];
	float min[7];
	min[0] = 0;
	min[1] = 45;
	min[2] = -8;
	max[0] = 90;
	max[1] = 135;
	max[2] = 9;

	position[0] = position[0]/3.14*180;
	position[1] = position[1]/3.14*180;

	if (position[0]>min[0] and position[0]<max[0] and position[1]>min[1] and position[1]<max[1] and position[1]>min[1] and position[1]<max[1])
	{
		//is_safe = true;
	}
	else
	{
		is_safe = false;
		//ROS_ERROR("Out of Range Error");
	}*/


}

void raventfCallback(raven_2::raven_tf msg)
{
	btTransform in_incr[3];
	tf::transformMsgToTF(msg.tf_incr[0], in_incr[0]);
	tf::transformMsgToTF(msg.tf_incr[1], in_incr[1]);
	tf::transformMsgToTF(msg.tf_incr[2], in_incr[2]);
	
	//printf("tf\n");
	//print_matrix(in_incr[2].getBasis());
	set_T(in_incr[0], in_incr[1], in_incr[2]);
	init_IK = true;
	
}


void set_tf_current(btTransform tf_msg)
{
	tf_current = tf_msg;
}
btTransform get_tf_current()
{
	return tf_current;
}
void ros_initialization()
{
	ros::NodeHandle n;
	sub_ravenstate = n.subscribe("ravenstate", 5000, ravenstateCallback);
	sub_jointstate = n.subscribe("joint_states", 5000, jointstatesCallback);
	sub_tf = n.subscribe("raven_tf", 5000, raventfCallback);
}
bool insertion(double L)
{
	ros::NodeHandle n;
	pub_automove = n.advertise<raven_2::raven_automove>("/raven_automove", 6000);
	raven_2::raven_automove msg;
	geometry_msgs::Transform tf_end;
	ros::Rate r(3000.0);
	bool new_end = true;
	while (n.ok() and runcheck){
		ros::spinOnce();
		if(init_status and init_IK)
		{
		while(new_end){
		tf_end = set_insertion(tf_current);
		printf("new end\n");
		new_end = false;
		}

		if (!insertion_step(tf_end, msg.tf_incr[1]))
		{
			ROS_ERROR("Done");
			return false;
			break;
		}
		msg.tf_incr[0].translation.x = 0;
		msg.tf_incr[0].translation.y = 0;
		msg.tf_incr[0].translation.z = 0;
		msg.tf_incr[0].rotation.x = 0;
		msg.tf_incr[0].rotation.y = 0;
		msg.tf_incr[0].rotation.z = 0;
		msg.tf_incr[0].rotation.w = 1;
		pub_automove.publish(msg);
		r.sleep();
		}
	}
	ros::spin();
	return true;
}
bool insertion_step(geometry_msgs::Transform tf_end, geometry_msgs::Transform &msg)
{
	geometry_msgs::Transform increment = set_increment(set_current(tf_current), tf_end);// (set_current, set_end)
	ROS_ERROR("incr"); print_geometry_msgs(increment);
	if (true)
	{
		increment.rotation.x = 0;
		increment.rotation.y = 0;
		increment.rotation.z = 0;
		increment.rotation.w = 1;	
	}
	tf::Quaternion dq;
	dq[0] = increment.rotation.x;
	dq[1] = increment.rotation.y;
	dq[2] = increment.rotation.z;
	dq[3] = increment.rotation.w;	
	//print_quaternion(dq);
	tf::Vector3 dp;
	dp[0] = increment.translation.x;
	dp[1] = increment.translation.y;
	dp[2] = increment.translation.z;
	//print_vector(dp);	
	if (increment.rotation.w<.98)
		is_safe = false;

	float kp_p;
	float kp_q;
	if (is_safe and vel_safe)
	{
		kp_p = 1;
		msg.translation.x = kp_p * (increment.translation.x)/1000;
		msg.translation.y = kp_p * (increment.translation.y)/1000;
		msg.translation.z = kp_p * (increment.translation.z)/1000;

		kp_q = 1;
		msg.rotation.x = kp_q * increment.rotation.x;
		msg.rotation.y = kp_q * increment.rotation.y;
		msg.rotation.z = kp_q * increment.rotation.z;
		msg.rotation.w = kp_q * increment.rotation.w;//positive after homing

	}else
	{
		msg.translation.x = 0;
		msg.translation.y = 0;
		msg.translation.z = 0;
		msg.rotation.x = 0;
		msg.rotation.y = 0;
		msg.rotation.z = 0;
		msg.rotation.w = 1;
	}
	if (abs(increment.translation.x) > 1000 or abs(increment.translation.y) > 1000 or abs(increment.translation.z) > 1000)
		return true;
	else
		return false;
}


bool roll(double angle)
{
	ros::NodeHandle n;
	pub_automove = n.advertise<raven_2::raven_automove>("/raven_automove", 6000);
	raven_2::raven_automove msg;
	geometry_msgs::Transform tf_end;
	ros::Rate r(20.0);
	bool new_end = true;
	int num = 0;
	while (n.ok() and runcheck){
		ros::spinOnce();
		if(init_status and init_IK)
		{
			tf_end = set_roll(tf_current, angle/abs(angle)*.1);
			roll_step(tf_end, msg.tf_incr[1]);
			num++;
			if (num > abs(angle))
			{
				return false;
				break;
			}
			/*while(new_end){
			tf_end = set_roll(tf_current);
			printf("new end\n");
			new_end = false;
			}

			if (!roll_step(tf_end, msg.tf_incr[1]))
			{
				ROS_ERROR("Done");
				new_end = true;
				num++;
				//ROS_ERROR();
				if (num>5)
				{
					return false;
					break;
				}
			}*/
			msg.tf_incr[0].translation.x = 0;
			msg.tf_incr[0].translation.y = 0;
			msg.tf_incr[0].translation.z = 0;
			msg.tf_incr[0].rotation.x = 0;
			msg.tf_incr[0].rotation.y = 0;
			msg.tf_incr[0].rotation.z = 0;
			msg.tf_incr[0].rotation.w = 1;
			pub_automove.publish(msg);
			r.sleep();
		}
	}
	ros::spin();
	return true;
}
bool roll_step(geometry_msgs::Transform tf_end, geometry_msgs::Transform &msg)
{
	geometry_msgs::Transform increment = set_increment(set_current(tf_current), tf_end);// (set_current, set_end)
	ROS_ERROR("incr"); print_geometry_msgs(increment);
	tf::Quaternion dq;
	dq[0] = increment.rotation.x;
	dq[1] = increment.rotation.y;
	dq[2] = increment.rotation.z;
	dq[3] = increment.rotation.w;	
	//print_quaternion(dq);
	tf::Vector3 dp;
	dp[0] = increment.translation.x;
	dp[1] = increment.translation.y;
	dp[2] = increment.translation.z;
	//print_vector(dp);	
	if (increment.rotation.w<.98)
		is_safe = false;

	float kp_p;
	float kp_q;
	if (is_safe and vel_safe)
	{
		kp_p = 1;
		msg.translation.x = kp_p * (increment.translation.x)/1000;
		msg.translation.y = kp_p * (increment.translation.y)/1000;
		msg.translation.z = kp_p * (increment.translation.z)/1000;

		kp_q = 1;
		msg.rotation.x = kp_q * increment.rotation.x;
		msg.rotation.y = kp_q * increment.rotation.y;
		msg.rotation.z = kp_q * increment.rotation.z;
		msg.rotation.w = kp_q * increment.rotation.w;//positive after homing

	}else
	{
		msg.translation.x = 0;
		msg.translation.y = 0;
		msg.translation.z = 0;
		msg.rotation.x = 0;
		msg.rotation.y = 0;
		msg.rotation.z = 0;
		msg.rotation.w = 1;
	}
	if (true)
		return true;
	else
		return false;
}

bool wrist(double angle)
{
	ros::NodeHandle n;
	pub_automove = n.advertise<raven_2::raven_automove>("/raven_automove", 6000);
	raven_2::raven_automove msg;
	geometry_msgs::Transform tf_end;
	ros::Rate r(20.0);
	bool new_end = true;
	int num = 0;
	while (n.ok() and runcheck){
		ros::spinOnce();
		if(init_status and init_IK)
		{
			tf_end = set_wrist(tf_current, angle/abs(angle)*.1);
			wrist_step(tf_end, msg.tf_incr[1]);
			num++;
			if (num > abs(angle))
			{
				return false;
				break;
			}
			/*while(new_end){
			tf_end = set_wrist(tf_current);
			printf("new end\n");
			new_end = false;
			}

			if (!wrist_step(tf_end, msg.tf_incr[1]))
			{
				ROS_ERROR("Done");
				new_end = true;
				num++;
				//ROS_ERROR();
				if (num>5)
				{
					return false;
					break;
				}
			}*/
			msg.tf_incr[0].translation.x = 0;
			msg.tf_incr[0].translation.y = 0;
			msg.tf_incr[0].translation.z = 0;
			msg.tf_incr[0].rotation.x = 0;
			msg.tf_incr[0].rotation.y = 0;
			msg.tf_incr[0].rotation.z = 0;
			msg.tf_incr[0].rotation.w = 1;
			pub_automove.publish(msg);
			r.sleep();
		}
	}
	ros::spin();
	return true;
}
bool wrist_step(geometry_msgs::Transform tf_end, geometry_msgs::Transform &msg)
{
	geometry_msgs::Transform increment = set_increment(set_current(tf_current), tf_end);// (set_current, set_end)
	ROS_ERROR("incr"); print_geometry_msgs(increment);
	tf::Quaternion dq;
	dq[0] = increment.rotation.x;
	dq[1] = increment.rotation.y;
	dq[2] = increment.rotation.z;
	dq[3] = increment.rotation.w;	
	//print_quaternion(dq);
	tf::Vector3 dp;
	dp[0] = increment.translation.x;
	dp[1] = increment.translation.y;
	dp[2] = increment.translation.z;
	//print_vector(dp);	
	if (increment.rotation.w<.98)
		is_safe = false;

	float kp_p;
	float kp_q;
	if (is_safe and vel_safe)
	{
		kp_p = 1;
		msg.translation.x = kp_p * (increment.translation.x)/1000;
		msg.translation.y = kp_p * (increment.translation.y)/1000;
		msg.translation.z = kp_p * (increment.translation.z)/1000;

		kp_q = 1;
		msg.rotation.x = kp_q * increment.rotation.x;
		msg.rotation.y = kp_q * increment.rotation.y;
		msg.rotation.z = kp_q * increment.rotation.z;
		msg.rotation.w = kp_q * increment.rotation.w;//positive after homing

	}else
	{
		msg.translation.x = 0;
		msg.translation.y = 0;
		msg.translation.z = 0;
		msg.rotation.x = 0;
		msg.rotation.y = 0;
		msg.rotation.z = 0;
		msg.rotation.w = 1;
	}
	if (true)
		return true;
	else
		return false;
}



bool wrist2(double angle)
{
	ros::NodeHandle n;
	pub_automove = n.advertise<raven_2::raven_automove>("/raven_automove", 6000);
	raven_2::raven_automove msg;
	geometry_msgs::Transform tf_end;
	ros::Rate r(20.0);
	bool new_end = true;
	int num = 0;
	while (n.ok() and runcheck){
		ros::spinOnce();
		if(init_status and init_IK)
		{
			tf_end = set_wrist2(tf_current, angle/abs(angle)*.1);
			wrist2_step(tf_end, msg.tf_incr[1]);
			num++;
			if (num > abs(angle))
			{
				return false;
				break;
			}
			/*while(new_end){
			tf_end = set_wrist2(tf_current);
			printf("new end\n");
			new_end = false;
			}

			if (!wrist2_step(tf_end, msg.tf_incr[1]))
			{
				ROS_ERROR("Done");
				new_end = true;
				num++;
				//ROS_ERROR();
				if (num>5)
				{
					return false;
					break;
				}
			}*/
			msg.tf_incr[0].translation.x = 0;
			msg.tf_incr[0].translation.y = 0;
			msg.tf_incr[0].translation.z = 0;
			msg.tf_incr[0].rotation.x = 0;
			msg.tf_incr[0].rotation.y = 0;
			msg.tf_incr[0].rotation.z = 0;
			msg.tf_incr[0].rotation.w = 1;
			pub_automove.publish(msg);
			r.sleep();
		}
	}
	ros::spin();
	return true;
}
bool wrist2_step(geometry_msgs::Transform tf_end, geometry_msgs::Transform &msg)
{
	geometry_msgs::Transform increment = set_increment(set_current(tf_current), tf_end);// (set_current, set_end)
	ROS_ERROR("incr"); print_geometry_msgs(increment);
	tf::Quaternion dq;
	dq[0] = increment.rotation.x;
	dq[1] = increment.rotation.y;
	dq[2] = increment.rotation.z;
	dq[3] = increment.rotation.w;	
	//print_quaternion(dq);
	tf::Vector3 dp;
	dp[0] = increment.translation.x;
	dp[1] = increment.translation.y;
	dp[2] = increment.translation.z;
	//print_vector(dp);	
	if (increment.rotation.w<.98)
		is_safe = false;

	float kp_p;
	float kp_q;
	if (is_safe and vel_safe)
	{
		kp_p = 1;
		msg.translation.x = kp_p * (increment.translation.x)/1000;
		msg.translation.y = kp_p * (increment.translation.y)/1000;
		msg.translation.z = kp_p * (increment.translation.z)/1000;

		kp_q = 1;
		msg.rotation.x = kp_q * increment.rotation.x;
		msg.rotation.y = kp_q * increment.rotation.y;
		msg.rotation.z = kp_q * increment.rotation.z;
		msg.rotation.w = kp_q * increment.rotation.w;//positive after homing

	}else
	{
		msg.translation.x = 0;
		msg.translation.y = 0;
		msg.translation.z = 0;
		msg.rotation.x = 0;
		msg.rotation.y = 0;
		msg.rotation.z = 0;
		msg.rotation.w = 1;
	}
	if (true)
		return true;
	else
		return false;
}


bool grasp(double angle)
{
	ros::NodeHandle n;
	pub_automove = n.advertise<raven_2::raven_automove>("/raven_automove", 6000);
	raven_2::raven_automove msg;
	ros::Rate r(20.0);
	bool new_end = true;
	int num = 0;
	while (n.ok() and runcheck){
		ros::spinOnce();
		if(init_status and init_IK)
		{
			num++;
			ROS_ERROR("incr %d", num);
			if (num > abs(angle))
			{
				return false;
				break;
			}
			msg.tf_incr[1].translation.x = 0;
			msg.tf_incr[1].translation.y = 0;
			msg.tf_incr[1].translation.z = 0;
			msg.tf_incr[1].rotation.x = 0;
			msg.tf_incr[1].rotation.y = 0;
			msg.tf_incr[1].rotation.z = 0;
			msg.tf_incr[1].rotation.w = 1;
			msg.grasp[1] = angle/abs(angle) * 10;
			msg.tf_incr[0].translation.x = 0;
			msg.tf_incr[0].translation.y = 0;
			msg.tf_incr[0].translation.z = 0;
			msg.tf_incr[0].rotation.x = 0;
			msg.tf_incr[0].rotation.y = 0;
			msg.tf_incr[0].rotation.z = 0;
			msg.tf_incr[0].rotation.w = 1;
			msg.grasp[0] = 0;
			pub_automove.publish(msg);
			r.sleep();
		}
	}
	ros::spin();
	return true;
}
