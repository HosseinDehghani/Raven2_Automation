void inputCallback(raven_2::raven_input msg);
void ravenstateCallback(raven_2::raven_state msg);
void jointstatesCallback(sensor_msgs::JointState msg);
void raventfCallback(raven_2::raven_tf msg);
void telopmodeCallback(raven_2::telOp_mode msg);



struct increment{
	float px;
	float py;
	float pz;
	float qw;
	float qx;
	float qy;
	float qz;
};

struct data{
	int num;
	float var[100];
	FILE *file;
};










void joint2theta(double *out_iktheta, double *in_J);
btTransform getFKTransform(int a, int b);
int xxx(double joint[6]);
void set_tf_current(btTransform tf_msg);
btTransform get_tf_current();
void ros_initialization();
bool insertion_step(geometry_msgs::Transform tf_end, geometry_msgs::Transform &msg);
bool insertion(double L);
bool roll_step(geometry_msgs::Transform tf_end, geometry_msgs::Transform &msg);
bool roll(double angle);
bool wrist_step(geometry_msgs::Transform tf_end, geometry_msgs::Transform &msg);
bool wrist(double angle);
bool wrist2_step(geometry_msgs::Transform tf_end, geometry_msgs::Transform &msg);
bool wrist2(double angle);
bool grasp(double angle);

const static double d2r = M_PI/180;
const static double r2d = 180/M_PI;
const static double eps = 1.0e-5;
const static double TH1_J0_L  = 205;//-180;//-205;   //add this to J0 to get \theta1 (in deg)
const static double TH2_J1_L  = 180;//-180;   //add this to J1 to get \theta2 (in deg)
const static double D3_J2_L   = 0.0;    //add this to J2 to get d3 (in meters????)
const static double TH4_J3_L  = 0;      //add this to J3 to get \theta4 (in deg)
const static double TH5_J4_L  = -90; //90;     //add this to J4 to get \theta5 (in deg)
const static double TH6A_J5_L = 0;     //add this to J5 to get \theta6a (in deg)
const static double TH6B_J6_L = 0;     //add this to J6 to get \theta6b (in deg)
