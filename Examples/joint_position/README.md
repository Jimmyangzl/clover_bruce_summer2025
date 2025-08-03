# How to get and set joint positions?
```bash
cd BRUCE-OP
```

### Step 1 - Create shared memory segments.
```bash
python3 -m Startups.memory_manager
```

### Step 2 - Start communication with actuators.
#### In simulation (refer to README.md in the Simulation folder)
```bash
python3 -m Simulation.sim_bruce
```
#### On hardware
```bash
Startups/usb_latency_setup.sh
python3 -m Startups.run_bear
python3 -m Startups.run_dxl
```

### Step 3 - Get/Set joint positions.
```bash
python3 -m Examples.joint_position.get_joint_position
python3 -m Examples.joint_position.set_joint_position
```