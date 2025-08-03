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

sleep 1s

# go to folder
cd /home/khadas/BRUCE/BRUCE-OP

# USB low latency setup
Startups/usb_latency_setup.sh
echo '====== USB Low Latency Setup Finished ======'
echo ''

# create shared memory
python3 -m Startups.memory_manager
echo '====== Shared Memory Created ======'
echo ''

# create a background screen named 'bruce'
screen -dmS bruce

# create a window named 'gamepad'
screen -S bruce -X screen -t gamepad
screen -S bruce -p gamepad -X stuff 'python3 -m Startups.run_gamepad^M'
echo '====== BRUCE Gamepad Online ======'
echo ''

# create a window named 'dxl'
screen -S bruce -X screen -t dxl

# create a window named 'bear'
screen -S bruce -X screen -t bear

# create a window named 'sense'
screen -S bruce -X screen -t sense

# create a window named 'estimation'
screen -S bruce -X screen -t estimation

# create a window named 'low_level'
screen -S bruce -X screen -t low_level

# create a window named 'high_level'
screen -S bruce -X screen -t high_level

# create a window named 'top_level'
screen -S bruce -X screen -t top_level

sleep 1s