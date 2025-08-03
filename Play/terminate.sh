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

# terminate bootup
if pgrep bootup
then kill -9 $(pgrep bootup)
fi

if pgrep bootup_gamepad
then kill -9 $(pgrep bootup_gamepad)
fi

# terminate threads
for i in $(seq 1 10)  # make sure !!!
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
