### Proportional Navigation Missile Autopilot and Simulation
The files in this repository are the engineering work that support the guidance, navigation, and control subsystem for my low-cost anti-missile missile senior design project. 

#### File Descriptions
**pronav_main_script.m:**
Main simulation handling script. Generates autopilot characteristics, and steps through each scenario's pre-launch phase, boost phase, and engagement phase. Tracks and stores data for each scenario in the simulation.

**pronav_sim.m:**
Script contains the ode45 integration solvers for the engagement phases. Optimizes total engagement time estimate for the guidance law before simulating the engagement. This works by simulating simple engagements with no target maneuvering, measurement errors, or wind, until the predicted engagement time with the least miss distance is found.

**PN_3D_Engagement_EOM.m:**
Equations of motion input to ode45.

**preLaunch.m:**
Estimates optimal initial heading for the pursuer missile at the time of launch.

**generate_threats2.m:**
Generates a randomly specified number of threat missile scenarios for the simulation.

**boost_phase.m:**
Integrates the boost phase of the simulation; there is no pursuer tracking in this phase.

**getAutopilot.m:**
Generates autopilot information in state-space form to be input into the integration solver.

**plots.m:**
Used for processing results.

#### Simulation Results and Autopilot Design


#### Future Work
None. Project is Finished!
