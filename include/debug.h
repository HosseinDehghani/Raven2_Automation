#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>


struct trajectory_data{
	float numberArray[153][3];
};

void print_matrix(btMatrix3x3 m);
void print_vector(tf::Vector3 v);
void print_quaternion(tf::Quaternion q);
void print_btTransform(btTransform tf);
void print_geometry_msgs(geometry_msgs::Transform msg);
void write_file(char *name, int argc, double argv[]);
struct trajectory_data read_trajectory();
