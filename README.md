# clover_bruce_summer2025

BRUCE Simulation assignments for clover summer course 2025.

## Dependencies

#### Python 3.6+ (pip, numpy, pyserial, termcolor, matplotlib, scipy, osqp, numba, dynamixel, posix_ipc)

## Instructions

### USE one terminal to run the commands below until you are told to open more terminals.
#### 1. Link BRUCE model to Gazebo
```bash
cd clover_bruce_summer2025
mkdir ~/.gazebo/models
ln -s $PWD/Simulation/models/bruce ~/.gazebo/models/bruce
```

#### 2. Add BRUCE Gym plugins to Gazebo
```bash
cp -r Library/BRUCE_GYM/GAZEBO_PLUGIN ~/.gazebo
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/.gazebo/GAZEBO_PLUGIN"  >>  ~/.bashrc
source ~/.bashrc
```

#### 3. Compiling Code Ahead of Time
Make sure to precompile all the Python scripts in the ``Library/ROBOT_MODEL`` and ``Library/STATE_ESTIMATION`` folders before use. 
```bash
python3 -m Library.ROBOT_MODEL.BRUCE_DYNAMICS_AOT
python3 -m Library.ROBOT_MODEL.BRUCE_KINEMATICS_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ORIENTATION_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ESTIMATION_CF_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ESTIMATION_KF_AOT
```

#### 4. Loading BRUCE model in Gazebo
```bash
cd /Simulation/worlds
gazebo --verbose bruce.world
```
If everything goes well, BRUCE should be fixed in the air in its nominal posture.

_ATTENTION:_
BRUCE Gym is built on Ubuntu 22.04.1 x86_64. Any lower version might need an upgrade of the GNU C libraries, e.g., GLIBC and GLIBCXX. Please refer to the error messages in this regard.

## Implement the code
#### Assignment 1 - Square Drawing
Please go to ``Play/Walking/walking_macros.py`` to finish the code.
For testing, start all six terminals following "Full Operating" below. In the last terminal, press "3" to test planning in joint space or, press "4" to test planning in cartesian space.
#### Assignment 1 - Arms Swinging
Please go to ``Play/Walking/walking_macros.py`` and ``Play/Walking/top_level.py`` to finish the code. For testing, start all six terminals following "Full Operating" below. Press "SPACE" and then quickly press "1" to let BRUCE walk forward. Press "SPACE" and then quickly press "0" to let BRUCE stop moving.


## Full Operating
### Now you should have six terminals.

1. Make all terminals go to clover_bruce_summer2025.
    ```bash
   cd clover_bruce_summer2025
    ```
2. Run Gazebo Simulator in terminal 0 (the same as the last step in "Instructions")
   ```bash
    cd /Simulation/worlds
    gazebo --verbose bruce.world
    ```
3. In terminal 1, run shared memory modules.
    ```bash
    python3 -m Startups.memory_manager
    ```
    Start Gazebo simulation afterwards.
    ```bash
    python3 -m Simulation.sim_bruce
    ```
4. In terminal 2, start state estimation thread.
    ```bash
    python3 -m Startups.run_estimation
    ```
5. In terminal 3, start low-level whole-body control thread.
    ```bash
    python3 -m Play.Walking.low_level
    ```
    Enter "y".
6. In terminal 4, start high-level DCM footstep planning thread.
    ```bash
    python3 -m Play.Walking.high_level
    ```
    Enter "y".
7. In terminal 5, start top-level user keyboard input thread.
    ```bash
    python3 -m Play.Walking.top_level
    ```
    Press any key, then you can press "2" to wave. Press "3" to test planning in joint space. Press "4" to test planning in cartesian space. Press "SPACE" then quickly press "1" to let the robot walk forward (and swing arms). Press "SPACE" then quickly press "0" to let the robot stop moving.

### Got Errors?
1. There is probably another Gazebo process running ...
 ```bash
killall gzserver
killall gzclient
 ```

### Other Notes
To quickly visualize your URDF online: [HERE](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html)
