# Proportional Navigation Missile Autopilot and Simulation
The files in this repository are the engineering work support the guidance, navigation, and control subsystem for my low-cost anti-missile missile senior design project. The files uploaded include the simulink for the main simulation, a Matlab file containing control loop gains and the initial conditions used in the simulations, and separate simulink files containing the block diagrams for each subsystem on their own. 

**Important Note**:\
All work is my own unless otherwise stated.

# File Descriptions and Instructions
**PNG3.slx**:\
Main simulation file. Includes Scope and/or Display blocks for range, Missile and Target X and Z positions, and acceleration ccommands. PN_Guidance_matlab.m needs to be executed before running the simulation, as it contains all initial conditions and control gains. Also before executing, 1) ensure persistent variables are enabled (Simulation tab --> Model Settings --> Search Bar --> search "persistent" and enable), and 2) disable zero-crossing errors (Simulation tab --> Model Settings --> Search Bar --> search "zero crossing" and disable).

**PN_Guidance_matlab.m**:\
File containing initial conditions and control gains. Execute before running PNG3.slx.

**autopilot_subsystem.slx**:\
Autopilot subsystem file. The autopilot design is from a Johns Hopkins University paper on pitch autopilots using Proportional Navigation (PN) guidance. The transfer functions and control gains were pulled from this paper, and are "reasonable" for a canard-controlled missile. The acceleration commands are normal to the body (that is, centripetal acceleration). Also has a Scope block within the Autopilot subsystem that shows how the Autopilot tracks acceleration commands.

Johns Hopkins University PN Guidance Autopilot: https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V29-N01/29-01-Jackson.pdf

**guidance_law_subsystem.slx**:\
Guidance Law subsystem file. Calculates range to target and acceleration commands to autopilot and includes acceleration limits to prevent oversaturation. The guidance law used is shown below:

$A_{cmd}=N V_M\dot \lambda$

where $A_{cmd}$ is the acceleration command, $N$ is a proportionality constant, $V_M$ is the missile cruise velocity, and $\dot \lambda$ is the Line of Sight angular rate.

**missile_position_subsystem.slx**:\
Missile Position subsystem file. Uses geometry to track missile position.

**target_motion_subsystem.slx**:\
Target Motion subsystem file. Uses geometry to track target position. Includes launch command for interceptor missile based on ground range from target.

# Future Work
1. Update to a 3D model/simulation, and include a yaw rate autopilot. Expected March 2025.
2. Create "Battle Management" subsystem. Expected March 2025.
3. Create "Homing Sensor" subsystem. Expected March 2025.
4. Include IMU noise model, wind model, and radome effects model into the simulation. Expected April 2025.
