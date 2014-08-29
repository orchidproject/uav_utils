#!/usr/bin/env python

import uav_utils.sweeps

if __name__ == '__main__':
    #print uav_utils.sweeps.spiral_sweep([2, -2], [18, -18], 2, 5)
    print uav_utils.sweeps.rect_sweep([2, 2], [18, 18], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([2, -2], [18, -18], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([-2, -2], [-18, -18], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([-2, 2], [-18, 18], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([2, 2], [18, 20], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([2, -2], [18, -20], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([-2, -2], [-18, -20], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([-2, 2], [-18, 20], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([2, 2], [20, 18], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([2, -2], [20, -18], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([-2, -2], [-20, -18], 2, 5), "\n======"
    print uav_utils.sweeps.rect_sweep([-2, 2], [-20, 18], 2, 5), "\n======"
