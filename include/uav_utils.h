#ifndef __SWEEPS_H_INCLUDED__
#define __SWEEPS_H_INCLUDED__ 1

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

int spiral_size(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval);
						
int spiral_sweep(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval,
						const bool inward, const double tol, 
						double* x, double* y, const int size);
						
int rect_size(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval);

int rect_sweep(const double start_x, const double start_y, 
						const double end_x, const double end_y,
						const double width, const double interval, 
						double* x, double* y, const int size);

#ifdef __cplusplus /* If this is a C++ compiler, end C linkage */
}
#endif

#endif
