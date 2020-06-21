#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//My libraries//
#include <proto_unit2/PDControl.h>
#include <proto_unit2/QuaternionEuler.h>

#define MAX_VEL_ 0.08
#define MIN_VEL_ -0.08
#define pi 3.1416


enum state{
	start,
	line,
	curve,
	off
};

state state_a = start; // el robot esta listo para iniciar. 

double d_input=0; //Distance desired.
double a_input=0; //Angle desired.
double error_=0;
double last_error=0;
double derror=0; 
bool success_pd;

double x, y, angle;
double my_out, my_out2;
double angle_yaw;


void Odom_Callback(const nav_msgs::Odometry::ConstPtr& data){
	x = data->pose.pose.position.x;
	y = data->pose.pose.position.y;

	//Acctualizando el Quaternion:
		//A data structure in my library  QuaternionEuler.h
		q.q1 = data->pose.pose.orientation.x;
		q.q2 = data->pose.pose.orientation.y;
		q.q3 = data->pose.pose.orientation.z;
		q.q0 = data->pose.pose.orientation.w;
	Quaternion_to_Euler(q); /*A function in QuaternionEuler.h library*/
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

				state_a= line;
				break;

			}
			case line:{

				my_out = PD_distance.PD_Control(x); /*A PD method in library PD.h*/
				obj.linear.x = my_out;
				if(PD_distance.success_pd){
					state_a=curve;
				}
				break;

			}
			case curve:{
				my_out = PD_position.PD_Control(angle_yaw); /*A PD method in library PD.h*/
				obj.angular.z = my_out;
				if(PD_position.success_pd){
					state_a=off;
				}
				break;
			}
			case off:{
				std::cout<<"Control terminado.\n"<<std::endl;
				return EXIT_SUCCESS;
				break;
			}
			default:{
				break; 
			}
		}
		pub_vel.publish(obj); 
		ros::spinOnce();
		r.sleep();
	}
	d_input =std::stod(argv[1]);
	return EXIT_SUCCESS;
}