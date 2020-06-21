#include <iostream>
#include <cmath>
#include <cstdlib>

struct Quaternion{
	double q1; //x
	double q2; //y
	double q3; //z
	double q0; //w
}q; //A data structure in my library  QuaternionEuler.h
struct Euler_angle{
	double pitch; //x
	double roll;  //y
	double yaw;   //z
}p;  //A data structure in my library  QuaternionEuler.h

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