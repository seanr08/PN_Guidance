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

![Step Response](https://github.com/seanr08/PN_Guidance/blob/main/Images/CL_Step.png) \
![Root Locus](https://github.com/seanr08/PN_Guidance/blob/main/Images/CL_Root_Locus.png)

## Simulation Results
A total of 967 scenarios were run for the simulation (if that seems like an odd number, it is: I tried running 1000 but the computer quit before getting there). For each scenario, the miss distance and probability of kill were recorded. 3D trajectory plots were generated for each scenario showing the pursuer's trajectory intercepting the target. The mean miss distance was 4.5 ft, with a standard deviation of 1.5 ft, and a mean probability of kill of 79%. The failure rate was 7%, which was computed as the number of intercepts with a miss distance of greater than 50 ft. Assuming the probability of kill of a miss of greater than 50 ft is 0%, the computed success rate is 73%. Plots of a sample trajectory, distribution of miss distance, and probability of kill vs miss distance are shown below.

![Sample Trajectory](https://github.com/seanr08/PN_Guidance/blob/main/Images/trajectories.png) \
![Distribution of Miss Distances](https://github.com/seanr08/PN_Guidance/blob/main/Images/Dist_Miss.png) \
![Probability of Kill vs Miss Distance](https://github.com/seanr08/PN_Guidance/blob/main/Images/PK_vs_MD.png)
