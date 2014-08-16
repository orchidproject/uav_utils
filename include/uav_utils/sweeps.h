#ifndef __SWEEPS_H_INCLUDED__
#define __SWEEPS_H_INCLUDED__ 1
 
double** spiral_sweep(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval,
						const bool outward, const double tol, int* size);

double** rectangular_sweep(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval, int* size);

#endif
