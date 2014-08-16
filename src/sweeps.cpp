#include<stdlib.h>
#include<math.h>


double** spiral_sweep(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval,
						const bool outward, const double tol, int* size){
							
	double const shift_theta = atan2(end_y - start_y, end_x - start_x);
	double const r = sqrt((start_x - end_x)*(start_x - end_x)+(start_y - end_y)*(start_y - end_y));
	double const omega = 2*r*M_PI / width;
	double const alpha = width / (4*M_PI);
	double const length = omega * sqrt(omega*omega + 1) + log(omega + sqrt(omega*omega+1));
	int n = floor((alpha*length) / interval);
	if(n * interval < alpha*length)
		n++;
	double theta[n];
	theta[0] = 0;
	
	//Helper variables
	int c;
	double const increment = length / (n - 1);
	double df,f, angle;
	double current_length = 0;
	double delta = omega / n;
	
	for(int i=1;i<n;i++){
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
	
	double shift = shift_theta - theta[n-1] + floor(theta[n-1] / (2 * M_PI)) * 2 * M_PI;
	double** points = (double**)malloc(n*sizeof(double));
	for(int i=0;i<n;i++)
		points[i] = (double*)malloc(2*sizeof(double));
	if(outward)
		for(int i=0;i<n;i++){
			points[i][0] = 2 * alpha * theta[i] * cos(theta[i] + shift) + start_x;
			points[i][1] = 2 * alpha * theta[i] * sin(theta[i] + shift) + start_y;
		}
	else
		for(int i=0;i<n;i++){
			points[i][0] = 2 * alpha * theta[n-i-1] * cos(theta[n-i-1] + shift) + start_x;
			points[i][1] = 2 * alpha * theta[n-i-1] * sin(theta[n-i-1] + shift) + start_y;
		}
	*size = n;
	return points;
}

double** rectangular_sweep(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval, int* size){
	double measure_x = (start_x > end_x) ? start_x - end_x : end_x - start_x;
	double measure_y = (start_y > end_y) ? start_y - end_y : end_y - start_y;
	double temp;
	
	int flip = measure_x < measure_y;
	if(flip){
		temp = measure_x;
		measure_x = measure_y;
		measure_y = temp;
	}
	
	int n_x = ceil(measure_x / interval);
	int n_y = ceil(measure_y / width);
	if(n_y % 2 == 0)
		n_y++;
		
	double increment_x = - measure_x / (n_x - 1);
	double increment_y = measure_y / (n_y - 1);
	short sign_x = (start_x - end_x  < 0) ? 1 : -1;
	short sign_y = (start_y - end_y < 0)? 1 : -1;
	if(flip){
		short temp = sign_x;
		sign_x = sign_y;
		sign_y = temp;
	}
	double offset_x = flip ? start_y : start_x;
	double offset_y = flip ? start_x : start_y;
	
	double** points = (double**)malloc(n_x*n_y*sizeof(double));
	for(int i=0;i<n_x*n_y;i++)
		points[i] = (double*)malloc(2*sizeof(double));

	for(int i=0;i<n_y;i++){
		increment_x = - increment_x;
		points[i*n_x][flip] = i % 2 == 0 ? offset_x : offset_x + sign_x * measure_x;
		points[i*n_x][!flip] = offset_y + sign_y * i * measure_y / (n_y - 1);
		for(int j=1;j<n_x;j++){
			points[i*n_x + j][flip] = points[i*n_x + j - 1][flip] + sign_x * increment_x;
			points[i*n_x + j][!flip] = points[i*n_x][!flip];
		}
	}
	*size = n_x*n_y;
	return points;
}
						
						
						
