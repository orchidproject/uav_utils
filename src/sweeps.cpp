#include "uav_utils.h"
#include<stdlib.h>
#include<math.h>

int spiral_size(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval){
	const double r = sqrt((start_x - end_x)*(start_x - end_x)+(start_y - end_y)*(start_y - end_y));
	const double omega = 2*r*M_PI / width;
	const double alpha = width / (4*M_PI);
	const double length = omega * sqrt(omega*omega + 1) + log(omega + sqrt(omega*omega+1));
	return ceil((alpha*length) / interval);
}


int spiral_sweep(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval,
						const bool inward, const double tol, 
						double* x, double* y, const int size){
	const double r = sqrt((start_x - end_x)*(start_x - end_x)+(start_y - end_y)*(start_y - end_y));
	const double omega = 2*r*M_PI / width;
	const double alpha = width / (4*M_PI);
	const double length = omega * sqrt(omega*omega + 1) + log(omega + sqrt(omega*omega+1));
	const int n = ceil((alpha*length) / interval);
	if(n != size)
		return -1;
		
	//Theta keeps the angles of the spiral of each point
	double* theta = (double*) malloc(n*sizeof(double));
	theta[0] = 0;
	
	//Helper variables
	const double increment = length / (n - 1);
	double df,f, angle;
	double current_length = 0;
	double delta = omega / n;
	
	for(int i=1;i<n;i++){
		//Netwon's method to solve equation for theta
		current_length += increment;
		angle = theta[i-1] + delta;
		df = sqrt(angle*angle + 1);
		f = angle * df + log(angle + df) - current_length;
		while(f < -tol || f > tol){
			angle -= f / (2*df);
			df = sqrt(angle*angle + 1);
			f = angle * df + log(angle + df) - current_length;
		}
		theta[i] = angle;
		delta = theta[i] - theta[i-1];
	}
	//Rotational shift so that the end point of the spiral matches the outer point
	const double shift = atan2(end_y - start_y, end_x - start_x) - theta[n-1] + floor(theta[n-1] / (2 * M_PI)) * 2 * M_PI;
	if(inward)
		for(int i=0;i<n;i++){
			x[i] = 2 * alpha * theta[n-i-1] * cos(theta[n-i-1] + shift) + start_x;
			y[i] = 2 * alpha * theta[n-i-1] * sin(theta[n-i-1] + shift) + start_y;
		}
	else
		for(int i=0;i<n;i++){
			x[i] = 2 * alpha * theta[i] * cos(theta[i] + shift) + start_x;
			y[i] = 2 * alpha * theta[i] * sin(theta[i] + shift) + start_y;
		}
	free(theta);
	return 0;
}


int rect_size(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval){
	double measure_x = (start_x > end_x) ? start_x - end_x : end_x - start_x;
	double measure_y = (start_y > end_y) ? start_y - end_y : end_y - start_y;
	double temp;
	if(measure_x < measure_y){
		temp = measure_x;
		measure_x = measure_y;
		measure_y = temp;
	}
    int n_x = ceil(measure_x / interval) + 1;
    int n_y = ceil(measure_y / width) + 1;
	if(n_y % 2 == 0)
		n_y++;
	return n_x * n_y;
}

int rect_sweep(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval, 
						double* x, double* y, const int size){
	double measure_x = (start_x > end_x) ? start_x - end_x : end_x - start_x;
	double measure_y = (start_y > end_y) ? start_y - end_y : end_y - start_y;
	double temp;
	
	int flip = measure_x < measure_y;
	if(flip){
		temp = measure_x;
		measure_x = measure_y;
		measure_y = temp;
	}
	
    int n_x = ceil(measure_x / interval) + 1;
    int n_y = ceil(measure_y / width) + 1;
	if(n_y % 2 == 0)
		n_y++;
	if(n_x * n_y != size)
		return -1;
	
	double increment_x = - measure_x / (n_x - 1);
	double increment_y = measure_y / (n_y - 1);
	short sign_x = (start_x - end_x  < 0) ? 1 : -1;
	short sign_y = (start_y - end_y < 0)? 1 : -1;
	if(flip){
		short temp = sign_x;
		sign_x = sign_y;
		sign_y = temp;
	}
	const double offset_x = flip ? start_y : start_x;
	const double offset_y = flip ? start_x : start_y;

	if(flip)
		for(int i=0;i<n_y;i++){
			increment_x = - increment_x;
			x[i*n_x]= i % 2 == 0 ? offset_x : offset_x + sign_x * measure_x;
			y[i*n_x] = offset_y + sign_y * i * measure_y / (n_y - 1);
			for(int j=1;j<n_x;j++){
				x[i*n_x + j] = x[i*n_x + j - 1] + sign_x * increment_x;
				y[i*n_x + j] = y[i*n_x];
			}
		}
	else
		for(int i=0;i<n_y;i++){
			increment_x = - increment_x;
			y[i*n_x]= i % 2 == 0 ? offset_x : offset_x + sign_x * measure_x;
			x[i*n_x] = offset_y + sign_y * i * measure_y / (n_y - 1);
			for(int j=1;j<n_x;j++){
				y[i*n_x + j] = y[i*n_x + j - 1] + sign_x * increment_x;
				x[i*n_x + j] = x[i*n_x];
			}
		}
	return 0;
}
