# Proportional Navigation Missile Autopilot and Simulation
The files in this repository are the engineering work that support the guidance, navigation, and control subsystem for my low-cost anti-missile missile senior design project. 

## File Descriptions
### pronav_main_script.m:
Main simulation handling script. Generates autopilot characteristics, and steps through each scenario's pre-launch phase, boost phase, and engagement phase. Tracks and stores data for each scenario in the simulation.

### pronav_sim.m:
Script contains the ode45 integration solvers for the engagement phases. Optimizes total engagement time estimate for the guidance law before simulating the engagement. This works by simulating simple engagements with no target maneuvering, measurement errors, or wind, until the predicted engagement time with the least miss distance is found.

### PN_3D_Engagement_EOM.m:
Equations of motion input to ode45.

### preLaunch.m:
Estimates optimal initial heading for the pursuer missile at the time of launch.

### generate_threats2.m:
Generates a randomly specified number of threat missile scenarios for the simulation.

### boost_phase.m:
Integrates the boost phase of the simulation; there is no pursuer tracking in this phase.

### getAutopilot.m:
Generates autopilot information in state-space form to be input into the integration solver.

### plots.m:
Used for processing results.

## Autopilot Design
The autopilot design uses a PID compensator to control canard servos and airframe. Lateral accelerations on the body-pitch and body-yaw axes are controlled indepently in two separate channels. The vehicle has one set of canards responsible for the body-pitch axis and another for the body-yaw axis. The autopilot was designed for a rise time of 0.2 seconds and minimal overshoot (typically, tactical missile actuation times are much faster, but due to the low budget, the design required a relatively slower response speed). The overshoot for the final design was 11%, with a 5% settling time of 1s. The closed loop poles were -31 +/- 40, -6.2, and -3.5. Shown below are the closed-loop step response and root locus plots.

![Step Response](https://github.com/PN_Guidance/Images/CL_Step.png)
![Root Locus](https://github.com/PN_Guidance/Images/CL_Root_Locus.png)

## Simulation Results

