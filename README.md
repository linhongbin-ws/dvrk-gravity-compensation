## Overview
This _dvrk\_gravity\_compensation_ package is designed for gravity compensation(GC) of Master Tool Manipulator(MTM) for dVRK. Compared to former work, our repository solves following problems:

* Elastic force and friction modeling in dynamics model 
* New data collection strategy of 7 DOF serial manipulator for further estimation 
* Multi-steps least square estimation
* Low-Friction Zero-Gravity controller

## Usage
-----
### 1. launch dVRK console

Open a terminal to start roscore

```
$ roscore
```

Open another terminal to launch dVRK console
```
$ qlacloserelays
#use a console config that contains the config file names of MTML and MTMR
$rosrun dvrk_robot dvrk_console_json -j <path_to_your_console_config.json>
```
After opening console, press `home` button to turn on MTM arms and move them to home position.

-----
### 2.Initialze MATLAB

Open MATLAB and go to the folder, _dvrk\_gravity\_compensation_. 

Then initialize the system:
```
addpath('<path-to-dvrk_matlab>');
rosinit;
```

-----
### 3. Runing MATLAB Script Program


Run the whole program in one command
```
mtm_gc_controller = estimate_gc_params('<ARM-NAME>','<Serial-Number>')

```
For example:
```
% For MTML
mtml_gc_controller = estimate_gc_params('MTML','12345')
% For MTMR
mtmr_gc_controller = estimate_gc_params('MTMR','54321')
```

Then the program starts. It will go through 4 processes:

**A) [wizard\_config\_dataCollection]**,

**B) [dataCollection]**, 

**C) [mlse]** 

**D) [gc_controller]**.

-----
### A) **[wizard\_config\_dataCollection]** (require user input)
In this process, users need to set joint limits for the data collection process. Going through this process is necessary because we don't want our MTMs hit the environments. For the reason that each MTM can be installed in different environment, we need this process to set joints limits.

After you step into this process, you will go through some instructions to set joint limits and then execute the collision checking. 

-----

#### A) Step #1:
MATLAB console:
<p align="center">
 <img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_1.png" width="1000"  />
</p>

The console will show some information of instructions， including instruction goal, MTM arm, Joint No which will be moved, the customized and current value and keyboard instruction. 

- To increase joint angle by 1 degree, type _i_ and return. 

- To decrease joint angle by 1 degree, type _d_ and type return.

- To set joint limit value, type _f_ and type return

Set the joint limit when distal link of MTM is around 10cm away from top panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_1_real.png" width="500"  />
 </p>
 

----------
#### A) Step #2:
MATLAB console:
<p align="center">
 <img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_2.png" width="1000"  />
 </p>

Set the joint limit when distal link of MTM is around 10cm away from front panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_2_real.png" width="500"  />
 </p>


----------
#### A) Step #3:
MATLAB console:
<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_3.png" width="1000"  />
 </p>

Set the joint limit when distal link of MTM is around 10cm away from top panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_3_real.png" width="500"  />
 </p>


----------
#### A) Step #4:
MATLAB console:
<p align="center">
   <img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_4.png" width="1000"  />
 </p>

Set the joint limit when distal link of MTM is around 10cm away from top panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_3_real.png" width="500"  />
 </p>


----------
#### A) Step #5:
MATLAB console:
<p align="center">
 <img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_5.png"  />
 </p>
 
Set the joint limit when distal link of MTM is around 10cm away from left panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_5_real.png" width="500"  />
 </p>


----------
#### A) Step #6:
MATLAB console:
<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_6.png" width="1000"  />
 </p>
 
Set the joint limit when distal link of MTM is around 10cm away from right panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/jhu-dvrk/dvrk-gravity-compensation/images/wizard_6_real.png" width="500"  />
 </p>

Afterwards, it will execute **collision checking**. If you set the limit properly according to the previous instructions, MTM will not hit the environment. But if it unluckily hits the environment by mistakes, push E-button immediately.  

**Collision checking** is making MTM to move according to preset trajectory which will be apply in dataCollection. Therefore, if collision checking passes, MTM in the dataCollection process will be free from collision.  

### B) **[dataCollection]** (auto):
In this process, dataCollection of MTM will be executed. This usually spend about 1hour. (No user input is required)

### C) Process#3 **[mlse]** (auto):
In this process, the dynamic parameters will be estimated by multi-steps least square estimation. (No user input is required)

### D) Process#4 **[gc_controller]** (auto):
In this process, gravity compensation controller will be applied by loading the dynamics parameters.(No user input is required)

After 4 processes are finished, users is able to move MTM in Gravity-Compensation mode.

## Loading GC configuration file in dVRK console
After the thrid process is finished, a GC configuration file will be generated according to the serial number of the MTM if GC controller can perform well with the parameters estimated in Process#3.

<!--**../GC_Data_stable/<ARM_NAME>_<SN>/<date&time>/gc-<ARM-Name>-<SN>.json**-->
For example: `../GC_Data_stable/MTML_41878/November-30-2018-10:57:53/gc-MTML-41878.json`.

Copy the GC configuration file to `~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/<your dVRK console configuration folder>`. In the console configuration file, add a field "gravity-compensation" in the specific MTM object:

For example: 

```
......
        {
            "name": "MTMR",
            "type": "MTM",
            "io": "sawRobotIO1394-MTMR-31519.xml",
            "pid": "sawControllersPID-MTMR.xml",
            "kinematic": "mtm.json",
            "gravity-compensation": "gc-MTML-31519.json"
        }
......
```

## Contact
This software is being developed by Biorobotics and Medical Technology Group of The Chinese University of Hong Kong (CUHK).
Feel free to contact us.  

Hongbin LIN:  [hongbinlin@cuhk.edu.hk](hongbinlin@cuhk.edu.hk)

Vincent Hui: [vincent.hui@cuhk.edu.hk](vincent.hui@cuhk.edu.hk) 
