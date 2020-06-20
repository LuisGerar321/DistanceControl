#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <cstdlib>

#define MAX_VEL_ 0.08
#define MIN_VEL_ -0.08
#define pi 3.1416



struct Quaternion{
	double q1; //x
	double q2; //y
	double q3; //z
	double q0; //w
};
struct Euler_angle{
	double pitch; //x
	double roll;  //y
	double yaw;   //z
};

enum state{
	start,
	line,
	curve,
	off
};

state state_a = start; // el robot esta listo para iniciar. 

Euler_angle p;
Quaternion  q;

double d_input=0; //Distance desired.
double a_input=0; //Angle desired.
double error_=0;
double last_error=0;
double derror=0; 
bool success_pd;

double x, y, angle;
double my_out, my_out2;
double angle_yaw;



void Quaternion_to_Euler(Quaternion q);

class PD_Control_Class{
	private:
		struct PD_var{
			double d_input=0; //Distance desired.
			double kp=0, kd=0;
			double error_=0;
			double last_error=0;
			double derror=0;

			double error_to=0;
			double MAX_VEL = 0.05;
			double MIN_VEL = MAX_VEL*(-1);
			double output =0;
		} pd_var;
	public:
		bool success_pd;
		PD_Control_Class(double d_input_, double kp_, double kd_, double error_to_, double vel){
			pd_var.d_input= d_input_;
			pd_var.kp = kp_;
			pd_var.kd = kd_;
			pd_var.error_to= error_to_;
			pd_var.MAX_VEL = vel;
		}
		void start_control(){
			std::cout<<"\nPD Control:\t"<<"Desired_input: "<<pd_var.d_input<<" kp: "<<pd_var.kp<<" kd: "<<pd_var.kd<<std::endl;
			success_pd=false;
		}

		double PD_Control(double feedback){

			if(!success_pd){

				pd_var.error_ = pd_var.d_input - feedback;
				pd_var.derror = pd_var.error_ - pd_var.last_error;
				pd_var.output = pd_var.kp * pd_var.error_ + pd_var.kd * pd_var.derror;

				if(pd_var.output>=pd_var.MAX_VEL){
					pd_var.output=pd_var.MAX_VEL;
				}
				else if(pd_var.output<=pd_var.MIN_VEL){
					pd_var.output=pd_var.MIN_VEL;
				}
				pd_var.last_error = pd_var.error_;
				std::cout<<"\nError: "<<pd_var.error_<<std::endl;
				if(pd_var.error_<=pd_var.error_to && pd_var.error_>=pd_var.error_to*(-1)){

					success_pd=true; 
					pd_var.error_=0; 
					pd_var.last_error=0;
					pd_var.derror=0;}
			}

			return 	pd_var.output;
		}
};


void Odom_Callback(const nav_msgs::Odometry::ConstPtr& data){
	x = data->pose.pose.position.x;
	y = data->pose.pose.position.y;

	//Acctualizando el Quaternion:
		q.q1 = data->pose.pose.orientation.x;
		q.q2 = data->pose.pose.orientation.y;
		q.q3 = data->pose.pose.orientation.z;
		q.q0 = data->pose.pose.orientation.w;
	Quaternion_to_Euler(q);
	angle_yaw = (p.yaw*180)/pi;

	std::cout<<"x_pos: "<<x<<"\ty_pos: "<< y<<"\tAngle: "<<angle_yaw<<std::endl;
}

int main(int argc, char** argv){

	ros::init(argc, argv,"Distance_Control");
	ros::NodeHandle n;
	//ros::Subscriber sub = n.subscribe("/odom",2,Odom_Callback);
	ros::Subscriber sub = n.subscribe("/mobile_base_controller/odom",2,Odom_Callback);
	ros::Rate r(10);

	//ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/rb1/cmd_vel", 2);
	ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 2);
	geometry_msgs::Twist obj;

	d_input =std::stod(argv[1]);
	a_input =std::stod(argv[2]);

	PD_Control_Class PD_distance(d_input, 1.8, 1.6, 0.0005, 0.06);
	PD_distance.start_control();

	PD_Control_Class PD_position(a_input, 0.1,0.08, 0.07, 0.20);
	PD_position.start_control();
	  
	while(ros::ok()){
		system("clear");
		std::cout<<"Distance position pd controler with input = "<<d_input<<" State: "<<state_a<<std::endl;

		switch(state_a){
			case start:{
				//Do something.
				state_a= line;

				//std::cout<<"PD Line_Class: "<<my_out<<std::endl;
				break;

			}
			case line:{
				//Do so...
				my_out = PD_distance.PD_Control(x);
				obj.linear.x = my_out;
				if(PD_distance.success_pd){
					state_a=curve;
				}
				break;
				// my_out = PD_Control(1.2, 1.6,d_input, x, &error_, &last_error, &derror);
				// std::cout<<"PD Line: "<<my_out<<std::endl;
				// obj.linear.x= my_out;
				// if(success_pd){ error_=0; last_error=0; derror=0; state_a=curve; success_pd=false;}
				// break; 
			}
			case curve:{
				//Do so..	
				my_out = PD_position.PD_Control(angle_yaw);
				obj.angular.z = my_out;
				if(PD_position.success_pd){
					state_a=off;
				}
				break;
			}
			case off:{
				//Do so..
				//obj.linear.x= 0;
				//obj.angular.z= 0;
				std::cout<<"Control terminado.\n"<<std::endl;
				return EXIT_SUCCESS;
				break;
			}
			default:{
				//Do so...
				break; 
			}
		}

		pub_vel.publish(obj); 
		ros::spinOnce();
		r.sleep();
	}
	d_input =std::stod(argv[1]);

	
	///ros::spin();
	return EXIT_SUCCESS;
}

void Quaternion_to_Euler(Quaternion q){
	double nume, deno;

	//Math to get pich //
	nume = 2*((q.q0*q.q1) + (q.q2*q.q3));
	deno = 1-( 2*( pow(q.q1,2) + pow(q.q2,2) )); 
	p.pitch = atan(nume/deno);

	//Math to get roll
	p.roll = asin(2*((q.q0*q.q2) -(q.q3*q.q1)));

	//Math to get yaw;
	nume = 2*((q.q0*q.q3) + (q.q1*q.q2));
	deno = 1-( 2*( pow(q.q2,2) + pow(q.q3,2) ));
	p.yaw  =atan(nume/deno);
}