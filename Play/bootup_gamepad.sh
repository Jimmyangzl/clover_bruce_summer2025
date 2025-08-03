#!/bin/bash
<< 'COMMENT'
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"
COMMENT

# terminate threads
for i in $(seq 1 5)
do
  screen -S bruce -p bear       -X stuff "^C"
  screen -S bruce -p dxl        -X stuff "^C"
  screen -S bruce -p sense      -X stuff "^C"
  screen -S bruce -p estimation -X stuff "^C"
  screen -S bruce -p low_level  -X stuff "^C"
  screen -S bruce -p high_level -X stuff "^C"
  screen -S bruce -p top_level  -X stuff "^C"
  sleep 0.1s
done

# reset shared memory
screen -S bruce -p dxl -X stuff 'python3 -m Startups.reset_memory^M'

# read config
demo=`python3 -m Play.config`

# run_dxl
echo $'\e[31mATTENTION: Press ENTER to start DXL control! DXL LED will be on!\e[0m'
kill -19 $(pgrep bootup_gamepad)
screen -S bruce -p dxl -X stuff 'python3 -m Startups.run_dxl^M'
sleep 1.5s
echo '====== Dynamixel Motor Online ======'
echo ''

# run_bear
echo $'\e[31mATTENTION: Press ENTER to start BEAR control! BEAR LED will be on!\e[0m'
kill -19 $(pgrep bootup_gamepad)
screen -S bruce -p bear -X stuff 'python3 -m Startups.run_bear^M'
sleep 1.5s
echo '====== BEAR Motor Online ======'
echo ''

# initialize, run_sense & run_estimation
echo $'\e[31mATTENTION: Press START to initialize BRUCE! Limbs will move!\e[0m'
kill -19 $(pgrep bootup_gamepad)
screen -S bruce -p sense -X stuff 'python3 -m Play.initialize^M'
sleep 0.5s
screen -S bruce -p sense -X stuff "s^M"
sleep 2s
echo '====== BRUCE Initialized ======'
echo ''

echo $'\e[31mATTENTION: Place BRUCE on the ground! Press Enter to start estimation! Wait until arms move!\e[0m'
kill -19 $(pgrep bootup_gamepad)
screen -S bruce -p sense -X stuff 'python3 -m Startups.run_sense^M'
sleep 1.5s
echo '====== BRUCE Sense Online ======'
echo ''
screen -S bruce -p estimation -X stuff 'python3 -m Startups.run_estimation^M'
sleep 1.5s
echo '====== State Estimation Online ======'
echo ''

# low_level
if [ $demo = "True" ]
then
  # screen -S bruce -p low_level -X stuff 'python3 -m Play.Demo.low_level^M'
  screen -S bruce -p low_level -X stuff 'Play/Demo/low_level-aarch64^M'
  sleep 0.5s
  screen -S bruce -p low_level -X stuff "n^M"
else
  screen -S bruce -p low_level -X stuff 'python3 -m Play.Walking.low_level^M'
fi
sleep 0.5s
echo $'\e[31mATTENTION: Press ENTER to start low-level control! BRUCE may twitch a bit!\e[0m'
kill -19 $(pgrep bootup_gamepad)
sleep 0.5s
screen -S bruce -p low_level -X stuff "y^M"
sleep 0.5s
echo '====== Low-Level Controller Online ======'
echo ''

# high_level
if [ $demo = "True" ]
then
  # screen -S bruce -p high_level -X stuff 'python3 -m Play.Demo.high_level^M'
  screen -S bruce -p high_level -X stuff 'Play/Demo/high_level-aarch64^M'
  sleep 0.5s
  screen -S bruce -p high_level -X stuff "n^M"
else
  screen -S bruce -p high_level -X stuff 'python3 -m Play.Walking.high_level^M'
fi
sleep 0.5s
echo $'\e[31mATTENTION: Press ENTER to start high-level control! BRUCE may twitch a bit!\e[0m'
kill -19 $(pgrep bootup_gamepad)
sleep 0.5s
screen -S bruce -p high_level -X stuff "y^M"
sleep 0.5s
echo '====== High-Level Controller Online ======'
echo ''

# top_level
echo $'\e[31mATTENTION: Press ENTER to enter cockpit!\e[0m'
kill -19 $(pgrep bootup_gamepad)
if [ $demo = "True" ]
then
  screen -S bruce -p top_level -X stuff 'python3 -m Play.Demo.top_level^M'
else
  screen -S bruce -p top_level -X stuff 'python3 -m Play.Walking.top_level^M'
fi
sleep 0.5s
echo '====== Top-Level Controller Online ======'
echo ''