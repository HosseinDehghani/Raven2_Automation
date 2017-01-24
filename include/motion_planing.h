#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>

enum l_r {
	dh_left = 0,
	dh_right = 1,
	dh_l_r_last=2
};

// Robot constants
const double La12 = 75 * M_PI/180;
const double La23 = 52 * M_PI/180;
const double La3 = 0;
const double V = 0;
const double d4 = -0.47;  // m
//const double d4 = -0.482; // 0.482 for daVinci tools  // m test value with connector
//const double Lw = 0.009;   // m
const double Lw = 0.013;   // m // .013 for raven II tools, 0.009 for daVinci tools
const double GM1 = sin(La12), GM2 = cos(La12), GM3 = sin(La23), GM4 = cos(La23);


geometry_msgs::Transform set_end(btTransform tf_current);
geometry_msgs::Transform set_current(btTransform tf_current);
geometry_msgs::Transform set_increment(geometry_msgs::Transform msg_current, geometry_msgs::Transform msg_end);
geometry_msgs::Transform set_insertion(btTransform tf_current);
geometry_msgs::Transform set_roll(btTransform tf_current, double step);
geometry_msgs::Transform set_wrist(btTransform tf_current, double step);
geometry_msgs::Transform set_wrist2(btTransform tf_current, double step);
geometry_msgs::Transform set_Z_axis(btTransform tf_current);
void set_tool_axis(tf::Vector3 tool_axis_);
tf::Vector3 get_tool_axis();
void set_T(btTransform T03_, btTransform T04_, btTransform T05_);
btTransform get_T04();




