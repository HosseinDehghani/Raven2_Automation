#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>
#include "debug.h"

void print_matrix(btMatrix3x3 m)
{
	printf("\n%f	%f	%f\n%f	%f	%f\n%f	%f	%f\n",
					m[0][0], m[0][1], m[0][2],
					m[1][0], m[1][1], m[1][2],
					m[2][0], m[2][1], m[2][2]);
}
void print_vector(tf::Vector3 v)
{
	printf("\n%f	%f	%f\n",
					v[0], v[1], v[2]);
}
void print_quaternion(tf::Quaternion q)
{
	printf("\n%f	%f	%f	%f\n",
					q[0], q[1], q[2], q[3]);
}
void print_btTransform(btTransform tf)
{
	printf("\btTransform:\n");
	tf::Vector3 origin = tf.getOrigin();
	print_vector(origin);
	btMatrix3x3 basis = tf.getBasis();
	print_matrix(basis);
}
void print_geometry_msgs(geometry_msgs::Transform msg)
{
	printf("\ngeometry_msgs:\n");
	printf("\n%f	%f	%f\n",
					msg.translation.x, msg.translation.y, msg.translation.z);
	printf("\n%f	%f	%f	%f\n",
					msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
}

void write_file(char *name, int argc, double argv[])
{
	FILE *file;
	file  = fopen(name ,"a");
	for(int i=0; i<argc; i++)
		fprintf(file, "%f	", argv[i]);
	fprintf(file, "\n");
	fclose(file);
}

struct trajectory_data read_trajectory()
{


	
	FILE *myFile;
	myFile = fopen("/opt/raven_2/raven_ros/raven_2/data/goal.txt", "r");

	//read file into array
	//float numberArray[153][3];
	struct trajectory_data data;
	if (myFile == NULL)
	{
	printf("Error Reading File\n");
	exit (0);
	}
	else
	{
	    for (int i = 0; i < 153; i++)
	    	for (int j = 0; j < 3; j++)
		fscanf(myFile, "%f,", &data.numberArray[i][j] );



		/*for (int i = 0; i < 153; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				printf("%f	", data.numberArray[i][j]);
			}
			printf("\n");
		}*/

	}

    fclose(myFile);
    

    return data;

}




