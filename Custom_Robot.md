# HOW TO SIMULATE A CUSTOM AERIAL ROBOT USING PX4

Roboticists often come up with new robot designs. Many of them prefer to have custom robot models. 

While using PX4, many models like X500, Omnicopter, etc have already been provided by PX4. Here is the method how to simulate our own custom model in Gazebo using PX4.

### Step 1 : Make sure you have proper `sdf` and `config` files. Save these in a folder.
### Step 2 : Go to the location `PX4-Autopilot/Tools/simulation/gz/models` and save the folder here.
### Step 3 : Next, go to the following folder : `PX4-Autopilot/ROMFOS/px4fmu_common/init.d-posix/models`. Here create a file, let say `2498_gz_vyomaveera` for our case. Here, add the airframe properties. For our case, it all goes as the follows: 
```
 #!/bin/sh
#
# @name Gazebo x500
#
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=vyomaveera}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM  0.05

param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM  0.05

param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

param set-default MPC_THR_HOVER 0.60
param set-default NAV_DLL_ACT 2 
```

Save this file.
### Step 4 : Now, in the same folder, in the `Cmakelists.txt`, add the name of the file we created now under `px4_add_romfs_files()`.
### Step 5 : Now, go to the root directory of PX4-Autopilot and run the following command: 
``` bash
make px4_sitl gz_vyomaveera     # Can be any drone name
```

Running this command will open gazebo and this is how we simulate our custom Aerial Robot.
