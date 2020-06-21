#include <iostream>
#include <cmath>
#include <cstdlib>

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
		void Set_Point(double desired){
			pd_var.d_input = desired;
		}
		void Control_Info(){
			std::cout<<"Set Point: "<<pd_var.d_input<<std::endl<<"KP: "<<pd_var.kp<<" KD: "<<pd_var.kd<<std::endl<<"Min_error: "<<pd_var.error_to<<std::endl<<"Max vel: "<<pd_var.MAX_VEL<<std::endl;
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