% Proportional Navigation Main Simulation Script

clear
close all
clc

% Initial Values:
% N: Navigation Gain
% aT_fpss: Target Maneuver Acceleration
% omega: Target Maneuver Rate
% AP_init_vec: 1x3 Column Vector of pursuer initial acceleration
% accel_lim: Pursuer max acceleration limit in g's

% Pointers to States
sel_RP1_ft = 1;
sel_RP2_ft = 2;
sel_RP3_ft = 3;
sel_VP1_fps = 4;
sel_VP2_fps = 5;
sel_VP3_fps = 6;
sel_RT1_ft = 7;
sel_RT2_ft = 8;
sel_RT3_ft = 9;
sel_VT1_fps = 10;
sel_VT2_fps = 11;
sel_VT3_fps = 12;
sel_aP1_FC_fps2 = 13;
sel_aP2_FC_fps2 = 14;
sel_aP3_FC_fps2 = 15;

% Simulation Parameters
dt=1e-3;
N=3.10;
GR_thresh = 50000; % ft
alt_range = [6000, 30000];
vel_range = [0.8, 2.5]*1125.33; % ft/s
num_sims=1000;
g=32.2; % ft/s2

% Initial Conditions
RP0_vec=[0;0;0];
aT_fpss=3*g;
omega=1;
AP_init_vec=[0;0;0];
accel_lim=25;

% Boost Acceleration and Desired Airspeed
a_boost_fps2=437.44;
v_desired=3000;

% Autopilot
[K_delta, M_delta, V, I_yy, omega_a, zeta_a, TF_par] = getAutopilot();

% Generate Scenarios
[positions, velocities] = generate_threats2(num_sims,alt_range,vel_range);

% Miss Distance Vector
miss_vec = zeros(1,num_sims);

% Engagement Time Vector
etime_vec = zeros(1,num_sims);

% Total Miss Counter
complete_miss = 0;

% Main Loop
for k=1:num_sims

    % Pre-Launch Phase
    [RT_L, VT_L, dir] = ...
         preLaunch(positions(:,k), velocities(:,k), 2500, GR_thresh);

    % Boost Phase
    [ RP_boost, VP_boost, RT_boost, VT_boost, t_boost, RP_hist, VP_hist, RT_hist, VT_hist, t_hist ] = boost_phase( ...
    RP0_vec, RT_L, VT_L, a_boost_fps2, v_desired, dir, dt );

    % Engagement Phase
    [x, tf_sec, miss] = pronav_sim(RT_boost,RP_boost,VT_boost,VP_boost, ...
        N,aT_fpss,omega,AP_init_vec,accel_lim,t_boost, ...
        K_delta, M_delta, V, I_yy, omega_a, zeta_a, ...
        TF_par);

    if miss > 50
        complete_miss=complete_miss+1;
    end

    % Populate Vectors
    miss_vec(k) = miss;
    etime_vec(k) = 6.860+tf_sec;

    % Combine Data
    RT1_hist=[RT_hist(:,1);x(:,sel_RT1_ft)];
    RT2_hist=[RT_hist(:,2);x(:,sel_RT2_ft)];
    RT3_hist=[RT_hist(:,3);x(:,sel_RT3_ft)];
    RP1_hist=[RP_hist(:,1);x(:,sel_RP1_ft)];
    RP2_hist=[RP_hist(:,2);x(:,sel_RP2_ft)];
    RP3_hist=[RP_hist(:,3);x(:,sel_RP3_ft)];

    if min(RT3_hist) < 10
        miss_vec(k) = 1000;
        fprintf('Missile Intercept Failure: Target reached ground')
    end

    if k < 10
        % Plot Scenario Trajectory
        figure;
        plot3(RP0_vec(1)./1000, RP0_vec(2)./1000, RP0_vec(3)./1000, 'b.','markersize',12); hold on
        plot3(RT_L(1)./1000, RT_L(2)./1000, RT_L(3)./1000, 'r.','markersize',12); 
        plot3(RP1_hist./1000, RP2_hist./1000, RP3_hist./1000, 'b', 'linewidth', 2); 
        plot3(RT1_hist./1000, RT2_hist./1000, RT3_hist./1000, 'r', 'linewidth', 2);
        set(gca,'fontsize',14);
        set(gcf,'color','w');
        xlabel('Downrange [kft]','fontsize',14);
        ylabel('Crossrange [kft]','fontsize',14);
        zlabel('Altitude [kft]','fontsize',14);
        grid on
        hold on
    end
end

% Compute Mean Miss Distance
miss_vec = sort(miss_vec);
mean_miss = mean(miss_vec);
median_miss = median(miss_vec);
std_miss = std(miss_vec);

% Compute Mean Engagement Time
etime_vec = sort(etime_vec);
mean_time = mean(etime_vec);
median_time = median(etime_vec);
std_time = std(etime_vec);

% Effective Lethal Radius for Fragmentation Warhead (in ft)
R_e = 1.8*3.28; % Adjust based on warhead specs or test data

% Compute Pk using fragmentation lethality model
Pk_vec = 1 - exp(-(R_e ./ miss_vec).^2);

figure;
plot(miss_vec,Pk_vec.*100)
title('Probability of Kill vs Miss Distance')
xlabel('Miss Distance (ft)')
ylabel('Probability of Kill %')

% Plot Normal Distribution of PK
pk_x=[0:0.1:100];
pk_y=normpdf(pk_x,mean(Pk_vec)*100,std(Pk_vec)*100);
figure;
plot(pk_x,pk_y);
title('Distribution of Probability of Kill')
xlabel('Probability of Kill (%)')
ylabel('Chance of Achieving Probability of Kill (%)')

% Plot Normal Distribution of Miss
miss_x=[min(miss_vec):0.1:max(miss_vec)];
miss_y=normpdf(miss_x,mean_miss,std_miss);
figure;
plot(miss_x,miss_y);
title('Distribution of Miss Distance')
xlabel('Miss Distance')
ylabel('Chance of Achieving Miss Distance (%)')

% Display Results
fprintf('Mean Miss Distance: %.4f\n', mean_miss);
fprintf('Median Miss Distance: %.4f\n', median_miss);
fprintf('Standard Deviation of Miss: %.4f\n', std_miss);
fprintf('Mean Engagement Time: %.4f\n', mean_time);
fprintf('Median Engagement Time: %.4f\n', median_time);
fprintf('Standard Deviation of Time: %.4f\n', std_time);
fprintf('Mean Probability of Kill (Fragmentation): %.4f\n', mean(Pk_vec));
fprintf('Median Probability of Kill (Fragmentation): %.4f\n', median(Pk_vec));
fprintf('Standard Deviation of Probability of Kill: %.4f\n', std(Pk_vec));
fprintf('Number of Complete Misses: %.4f\n', complete_miss);