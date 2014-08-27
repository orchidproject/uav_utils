#!/usr/bin/env python

import uav_utils.sweeps

if __name__ == '__main__':
    print uav_utils.sweeps.spiral_sweep([0, 0], [20, 21], 3, 10)
