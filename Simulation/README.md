# BRUCE Gym

BRUCE Gym is the simulation environment for BRUCE-OP. It utilizes a custom library to interact BRUCE in Gazebo using python.

## Dependencies

#### Python 3.6+ (pip, numpy, pyserial, termcolor, matplotlib, scipy, osqp, numba, dynamixel, posix_ipc)

## Installation

#### 1. [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
The latest version, Gazebo11, is recommended.

#### 2. Link BRUCE model to Gazebo
```bash
cd BRUCE-OP
mkdir ~/.gazebo/models
ln -s $PWD/Simulation/models/bruce ~/.gazebo/models/bruce
```

#### 3. Add BRUCE Gym plugins to Gazebo
```bash
cd BRUCE-OP
cp -r Library/BRUCE_GYM/GAZEBO_PLUGIN ~/.gazebo
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/.gazebo/GAZEBO_PLUGIN"  >>  ~/.bashrc
source ~/.bashrc
```

## Instructions

### Indicating Running in Simulation
Modify line 14 of ``Play/config.py``.
```python
SIMULATION = True  # if in simulation or not
```

### Compiling Code Ahead of Time
Make sure to precompile all the Python scripts in the ``Library/ROBOT_MODEL`` and ``Library/STATE_ESTIMATION`` folders before use. 
```bash
cd BRUCE-OP
python3 -m Library.ROBOT_MODEL.BRUCE_DYNAMICS_AOT
python3 -m Library.ROBOT_MODEL.BRUCE_KINEMATICS_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ORIENTATION_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ESTIMATION_CF_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ESTIMATION_KF_AOT
```

### Loading BRUCE model in Gazebo
```bash
cd BRUCE-OP/Simulation/worlds
gazebo --verbose bruce.world
```
If everything goes well, BRUCE should be fixed in the air in its nominal posture.

_ATTENTION:_
BRUCE Gym is built on Ubuntu 22.04.1 x86_64. Any lower version might need an upgrade of the GNU C libraries, e.g., GLIBC and GLIBCXX. Please refer to the error messages in this regard.

### Quick Launch
1. Go to BRUCE-OP folder.
    ```bash
    cd BRUCE/BRUCE-OP
    ```
2. Config launch setup in ``Play/config.py``.
3. Run the bash file and follow the guidance.
    ```bash
    Play/sim_bootup.sh
    ```

### Full Operating
1. Make all terminals go to BRUCE-OP folder.
    ```bash
   cd BRUCE-OP
    ```
2. In terminal 1, run shared memory modules.
    ```bash
    python3 -m Startups.memory_manager
    ```
    Start Gazebo simulation afterwards.
    ```bash
    python3 -m Simulation.sim_bruce
    ```
3. In terminal 2, start state estimation thread.
    ```bash
    python3 -m Startups.run_estimation
    ```
4. In terminal 3, start low-level whole-body control thread.
    ```bash
    python3 -m Play.Walking.low_level
    ```
5. In terminal 4, start high-level DCM footstep planning thread.
    ```bash
    python3 -m Play.Walking.high_level
    ```
6. In terminal 5, start top-level user keyboard input thread.
    ```bash
    python3 -m Play.Walking.top_level
    ```

### Got Errors?
1. There is probably another Gazebo process running ...
 ```bash
killall gzserver
killall gzclient
 ```

### Other Notes
To quickly visualize your URDF online: [HERE](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html)