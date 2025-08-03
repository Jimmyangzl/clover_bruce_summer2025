# BRUCE-OP

Repo for BRUCE Open-Platform.

_ATTENTION:_

_1. Always remember to press the E-STOP immediately in emergencies!_

_2. Your system may need some fine-tuning before it can perform perfectly, i.e., due to slight differences in joint resistance and part tolerances, systems may vary in tuning parameters._

_3. For more technical details, please go to [BRUCE-OP Wiki](https://wiki.bruce-op.com/)._

## Licensing
BRUCE is dual-licensed under both commercial and open-source licenses. The work in this repo is under GNU General Public License (GPL) version 3, which is ideal for use cases such as open-source projects with open-source distribution, student/academic purposes, hobby projects, internal research projects without external distribution, or other projects where all GPL obligations can be met. Read the full text of [the GNU GPL version 3](https://www.gnu.org/licenses/gpl-3.0.html) for details.  
Contact us for commercial license should you need full rights to create and distribute the platform on your own terms without any open-source license obligations.

## Dependencies

#### 1. Python 3.6+ (pip, numpy, pyserial, termcolor, matplotlib, scipy, osqp, numba, dynamixel, posix_ipc)

#### 2. PyBEAR

Please use [PyBEAR](https://github.com/Westwood-Robotics/PyBEAR) from Westwood Robotics for communication with the BEAR actuators.

## Instructions
### Compiling Code Ahead of Time
Make sure to precompile all the Python scripts in the ``Library/ROBOT_MODEL`` and ``Library/STATE_ESTIMATION`` folders before use. 
```bash
cd BRUCE/BRUCE-OP
python3 -m Library.ROBOT_MODEL.BRUCE_DYNAMICS_AOT
python3 -m Library.ROBOT_MODEL.BRUCE_KINEMATICS_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ORIENTATION_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ESTIMATION_CF_AOT
python3 -m Library.STATE_ESTIMATION.BRUCE_ESTIMATION_KF_AOT
```

### SSH Manual
1. Make sure your laptop and BRUCE's onboard computer share the same network, e.g., you can use a Wi-Fi hotspot.
2. Check the username and ip address (``ip addr``) of the onboard computer.
3. Login to remote server with the password ``khadas``. The ip address may vary, e.g.,
    ```bash
    ssh khadas@khadas.local
    ssh khadas@192.168.1.137
    ```

### Serial Port & MAC Address
Please config the serial port names (of BEAR, Dynamixel, and Pico) and MAC address of your gamepad (if applicable) in ``BRUCE_SERIAL_PORT`` or ``Settings/BRUCE_macros.py`` (line 272-283) for your BRUCE before use.

You can check the serial port names by entering the following command in terminal:
```bash
ls /dev/serial/by-id/
```

### Allow Executing Bash/Binary File As Program
1. Download the binary demo controller according to [README](https://github.com/Westwood-Robotics/BRUCE-OP/blob/main/Play/Demo/README.md).
2. Go to BRUCE-OP folder.
    ```bash
    cd BRUCE/BRUCE-OP
    ```
3. Run the following commands.
    ```bash
    chmod +x Play/bootup.sh
    chmod +x Play/bootup_gamepad.sh
    chmod +x Play/sim_bootup.sh
    chmod +x Play/terminate.sh
    chmod +x Play/Demo/low_level-aarch64
    chmod +x Play/Demo/low_level-x86_64
    chmod +x Play/Demo/high_level-aarch64
    chmod +x Play/Demo/high_level-x86_64
    chmod +x Startups/startup_setup.sh
    chmod +x Startups/usb_latency_setup.sh
    ```

### Startup Setup Before Launch
_BRUCE after VERSION 0.0.4 will be automatically configured at bootup with [Startup Applications](https://help.ubuntu.com/stable/ubuntu-help/startup-applications.html.en)._
1. Go to BRUCE-OP folder.
    ```bash
    cd BRUCE/BRUCE-OP
    ```
2. Run the bash file.
    ```bash
    Startups/startup_setup.sh
    ```

### Quick Launch
1. Go to BRUCE-OP folder.
    ```bash
    cd BRUCE/BRUCE-OP
    ```
2. Config launch setup in ``Play/config.py``.
3. Run the bash file and follow the guidance.
    ```bash
    Play/bootup.sh
    ```
4. Terminate in the end.
    ```bash
    Play/terminate.sh
    ```

### Full Operating
We suggest use terminator as an alternative terminal for Linux since we need to run 7 threads concurrently.
1. Make all terminals go to BRUCE-OP folder.
    ```bash
    cd BRUCE/BRUCE-OP
    ```
2. In terminal 1, run USB low latency setup and shared memory modules.
    ```bash
    Startups/usb_latency_setup.sh
    python3 -m Startups.memory_manager
    ```
    Start Dynamixel motor thread afterwards.
    ```bash
    python3 -m Startups.run_dxl
    ```
3. In terminal 2, start BEAR actuator thread.
    ```bash
    python3 -m Startups.run_bear
    ```
4. In terminal 3, initialize BRUCE (enter ``s``) and then start sense communication thread after BRUCE can stand on the ground on its own.
    ```bash
    python3 -m Play.initialize
    python3 -m Startups.run_sense
    ```
5. In terminal 4, start state estimation thread.
    ```bash
    python3 -m Startups.run_estimation
    ```
6. In terminal 5, start low-level whole-body control thread.
    ```bash
    python3 -m Play.Walking.low_level
    ```
7. In terminal 6, start high-level DCM footstep planning thread.
    ```bash
    python3 -m Play.Walking.high_level
    ```
8. In terminal 7, start top-level user input thread.
    ```bash
    python3 -m Play.Walking.top_level
    ```

### Gamepad Operating
1. Modify line 15 of ``Play/config.py`` as follows and reboot.
    ```python
    GAMEPAD = True  # if using gamepad or not
    ```
2. Please follow the instructions shown on the gamepad.

### Simulation (BRUCE Gym)
BRUCE Gym simulation is based on a custom library to interact with Gazebo. Please refer to [README](https://github.com/Westwood-Robotics/BRUCE-OP/blob/main/Simulation/README.md) in the Simulation folder for detailed instructions.

## In case of a glitch in your brain ...
### VIM (for simple modification on files)
 ```bash
 vim filepath
 ```
1. Press ``i`` to enter insert mode and edit. 
2. After finishing editing, press ``ESC`` and then type ``:wq`` to save changes (or type ``:q!`` to ignore any changes).
