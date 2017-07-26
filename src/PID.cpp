#include "PID.h"
#include <math.h>
#include <iostream>
using namespace std;

#define MAX_STEER 1
#define MIN_STEER -1
#define SETTLE 200
/*
* TODO: Complete the PID class.
*/

PID::PID(){

	prev_cte = 0;
	total_cte= (0);
	Kp = 1;
	Ki = 1;
	Kd = 1;
	count = 0;
	total_err = 0;
}

//PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
	// Pass the values of PID coeffs from main file
	Kp = kp;
	Ki = ki;
	Kd = kd;

}

void PID::UpdateError(double cte) {


}

double PID::TotalError(double cte) {

	// Calculate the output of the PID controller which should be the steering angle

	double delta_t = 0.065;  // time between two sensor measurements
	total_cte += cte;
	double steer = -Kp *cte - Ki*total_cte -Kd*(cte-prev_cte)/delta_t;
	prev_cte = cte;

	//Bring steer angle in range
	if (steer > MAX_STEER)
		steer  = MAX_STEER;
	else if (steer < MIN_STEER)
		steer  = MIN_STEER;

	++count;


	if (count > SETTLE){ // get combined CTE once the loop is settled

		total_err += cte;
		double final_err = (sqrt(total_err*total_err))/(-SETTLE + count);
		std::cout<< "The average CTE over last " <<-SETTLE+count<<" Iterations is " << final_err<<"\n\n";

	}

	return steer;






}

