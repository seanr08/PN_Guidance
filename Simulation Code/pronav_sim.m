function [x, tf_sec, miss_ft] = pronav_sim(RT_vec, RP_vec, VT_vec, VP_vec, ...
    N, aT_fpss, omega, AP_init_vec, accel_lim, t_boost, ...
    K_delta, M_delta, V, I_yy, omega_a, zeta_a, ...
    TF_par)

% Target Initial Conditions
RT1_ft  = RT_vec(1);
RT2_ft  = RT_vec(2);
RT3_ft  = RT_vec(3);
% VT1_fps = -aT_fps2/omega_rps;
VT1_fps = VT_vec(1);
VT2_fps = VT_vec(2);
VT3_fps = VT_vec(3);

% Missile Initial Condition
RP1_ft  = RP_vec(1);
RP2_ft  = RP_vec(2);
RP3_ft  = RP_vec(3);
VP1_fps = VP_vec(1);
VP2_fps = VP_vec(2);
VP3_fps = VP_vec(3);

% Main Loop/Engagement Time Optimization
t0=0;
tol=0.25;
prev_miss_ft=99999;
new_miss_ft=9999;
k=0;
fac=-0.5;
while (abs(new_miss_ft-prev_miss_ft)) > tol
    if k > 200
        break
    end
    k=k+1;
    prev_miss_ft=new_miss_ft;
    % Initial Closing Velocity and Time Vector
    vrel=[VT1_fps-VP1_fps;VT2_fps-VP2_fps;VT3_fps-VP3_fps];
    rrel=[RT1_ft-RP1_ft;RT2_ft-RP2_ft;RT3_ft-RP3_ft];
    vc_init=dot(vrel,rrel)/norm(rrel);
    tf_sec=abs(norm(rrel)/vc_init)+t0;
    dt = 1e-3;

    %disp(tf_sec)

    % AP Initial Condition
    aP1_ach_fps2 = AP_init_vec(1);
    aP2_ach_fps2 = AP_init_vec(2);
    aP3_ach_fps2 = AP_init_vec(3);

    % Pointers to states
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

    % Pursuer acceleration limit (25g accel mag limit)
    fps2_limit = accel_lim*32.2;

    % Initial condition
    x0 = [RP1_ft, RP2_ft, RP3_ft, VP1_fps, VP2_fps, VP3_fps, ...
          RT1_ft, RT2_ft, RT3_ft, VT1_fps, VT2_fps, VT3_fps, ...
          aP1_ach_fps2, aP2_ach_fps2, aP3_ach_fps2, 0, 0, 0, ...
          0, 0, 0, 0, 0, 0, 0, 0, 0];
    % 
    % Engagement Phase
    t_sim=tf_sec-t_boost;
    options = odeset('abstol',1e-2,'reltol',1e-2);
    [t_sec, x] = ode45(@PN_3D_Engagement_EOM, ...
        [0 , t_sim], x0, options, t_sim, N, fps2_limit, ...
        K_delta, M_delta, V, I_yy, omega_a, zeta_a, ...
        false, TF_par, aT_fpss, omega);

    % Compute miss index
    range_ft = sqrt((x(:,sel_RT1_ft:sel_RT3_ft) - x(:,sel_RP1_ft:sel_RP3_ft)).^2*ones(3,1));
    miss_index = find(range_ft == min(range_ft));
    min_range_ft=min(range_ft);
    %disp(min_range_ft_a)

    new_miss_ft=min_range_ft;
    if new_miss_ft < prev_miss_ft
        t0=t0+fac;
    else
        fac=-0.5*fac;
        t0=t0+fac;
    end
    %fprintf('tf (s) = %2.2f, Miss (ft) = %2.2f\n', tf_sec, new_miss_ft)
end

t_sim=tf_sec-t_boost;
options = odeset('abstol',1e-2,'reltol',1e-2);
[t_sec, x] = ode45(@PN_3D_Engagement_EOM, ...
    [0 , t_sim], x0, options, t_sim, N, fps2_limit, ...
    K_delta, M_delta, V, I_yy, omega_a, zeta_a, ...
    true, TF_par, aT_fpss, omega);

range_ft = sqrt((x(:,sel_RT1_ft:sel_RT3_ft) - x(:,sel_RP1_ft:sel_RP3_ft)).^2*ones(3,1));
miss_index = find(range_ft == min(range_ft));
min_range_ft=min(range_ft);
miss_ft = min_range_ft;

fprintf('tf (s) = %2.2f, Miss (ft) = %2.2f\n, Iter = %2.2f\n', tf_sec, miss_ft, k)

%% PLOTS
%--------------------------------------------------------------------------

clear F1
%{
% Plot engagement trajectories
%figure(6*(index-1)+1)
figure(1)
plot3(x(1,sel_RP1_ft)./1000, x(1,sel_RP2_ft)./1000, x(1,sel_RP3_ft)./1000, 'b.','markersize',12); hold on
plot3(x(1,sel_RT1_ft)./1000, x(1,sel_RT2_ft)./1000, x(1,sel_RT3_ft)./1000, 'r.','markersize',12); 
plot3(x(:,sel_RP1_ft)./1000, x(:,sel_RP2_ft)./1000, x(:,sel_RP3_ft)./1000, 'b', 'linewidth', 2); 
plot3(x(:,sel_RT1_ft)./1000, x(:,sel_RT2_ft)./1000, x(:,sel_RT3_ft)./1000, 'r', 'linewidth', 2);
set(gca,'fontsize',14);
set(gcf,'color','w');
xlabel('Downrange [kft]','fontsize',14);
ylabel('Crossrange [kft]','fontsize',14);
zlabel('Altitude [kft]','fontsize',14);
grid on
%}

%{
% Plot range
figure(6*(index-1)+2)
RTP1_ft = x(:,sel_RT1_ft) - x(:,sel_RP1_ft); 
RTP2_ft = x(:,sel_RT2_ft) - x(:,sel_RP2_ft);
RTP3_ft = x(:,sel_RT3_ft) - x(:,sel_RP3_ft);
RTPi_ft = [RTP1_ft, RTP2_ft, RTP3_ft];
range_ft = sqrt(RTP1_ft.^2+RTP2_ft.^2+RTP3_ft.^2);
semilogy(t_sec, range_ft, 'linewidth', 2)
xlabel('Time [sec]','fontsize', 14);
ylabel('Range [ft]','fontsize', 14);
set(gca,'fontsize', 14);
set(gcf,'color','w');
grid on 

% Determine Miss Distance Index
miss_index = find(range_ft == min(range_ft));

% Plot pursuer commanded acceleration
NaN_index = find(isnan(x(:,1))==1,1);
if isempty(NaN_index)
    NaN_index = length(t_sec);
else
    NaN_index = NaN_index-1;
end

tgo_sec = tf_sec - t_sec;

VTP1_fps = x(1:miss_index,sel_VT1_fps) - x(1:miss_index,sel_VP1_fps);
VTP2_fps = x(1:miss_index,sel_VT2_fps) - x(1:miss_index,sel_VP2_fps);
VTP3_fps = x(1:miss_index,sel_VT3_fps) - x(1:miss_index,sel_VP3_fps);

% ZEM resolved in the inertial cs
ZEMi1_ft = RTP1_ft(1:miss_index) + VTP1_fps.*tgo_sec(1:miss_index);
ZEMi2_ft = RTP2_ft(1:miss_index) + VTP2_fps.*tgo_sec(1:miss_index);
ZEMi3_ft = RTP3_ft(1:miss_index) + VTP3_fps.*tgo_sec(1:miss_index);
ZEMi_ft  = [ZEMi1_ft, ZEMi2_ft, ZEMi3_ft];

% Unit vector along line of sight
RTPi_ft_2norm = sqrt(RTP1_ft(1:miss_index).^2 + ...
                     RTP2_ft(1:miss_index).^2 + ...
                     RTP3_ft(1:miss_index).^2);
LOS_Unit_Vector = RTPi_ft(1:miss_index,:)./RTPi_ft_2norm;

% ZEM in the r direction resolved in the inertial cs
ZEMr_ft = (ZEMi_ft*LOS_Unit_Vector')*...
    LOS_Unit_Vector;

% Zero effort miss perpindicular to the line of sight
ZEMn_ft = ZEMi_ft(1:miss_index,:) - ZEMr_ft(1:miss_index,:);

% Acceleration command
aPi1_fps2 = N*ZEMn_ft(1:miss_index,1)./tgo_sec(1:miss_index).^2;
aPi2_fps2 = N*ZEMn_ft(1:miss_index,2)./tgo_sec(1:miss_index).^2;
aPi3_fps2 = N*ZEMn_ft(1:miss_index,3)./tgo_sec(1:miss_index).^2;

aPi1_limit_indices = find(abs(aPi1_fps2) > fps2_limit);
aPi1_fps2(aPi1_limit_indices) = fps2_limit*sign(aPi1_fps2(aPi1_limit_indices));
aPi2_limit_indices = find(abs(aPi2_fps2) > fps2_limit);
aPi2_fps2(aPi2_limit_indices) = fps2_limit*sign(aPi2_fps2(aPi2_limit_indices));
aPi3_limit_indices = find(abs(aPi3_fps2) > fps2_limit);
aPi3_fps2(aPi3_limit_indices) = fps2_limit*sign(aPi3_fps2(aPi3_limit_indices));

% Total g's commanded
gsP = sqrt(aPi1_fps2.^2 + aPi2_fps2.^2 + aPi3_fps2.^2)./32.2;

figure(6*(index-1)+3)
semilogy(t_sec(1:miss_index), gsP(1:miss_index), 'linewidth', 2)
xlabel('Time [sec]','fontsize', 14);
ylabel('Pursuer Gs Commanded','fontsize', 14);
set(gca,'fontsize', 14);
set(gcf,'color','w');
grid on 

% Plot Downrange
figure(6*(index-1)+4)
plot(t_sec, RTP1_ft, 'linewidth', 2)
xlabel('Time [sec]','fontsize', 14);
ylabel('Downrange Relative Position [ft]','fontsize', 14);
set(gca,'fontsize', 14);
set(gcf,'color','w');
grid on 

% Plot Crossrange
figure(6*(index-1)+5)
plot(t_sec, RTP2_ft, 'linewidth', 2)
xlabel('Time [sec]','fontsize', 14);
ylabel('Crossrange Relative Position [ft]','fontsize', 14);
set(gca,'fontsize', 14);
set(gcf,'color','w');
grid on 

% Plot Altitute
figure(6*(index-1)+6)
plot(t_sec, RTP3_ft, 'linewidth', 2)
xlabel('Time [sec]','fontsize', 14);
ylabel('Altitude Relative Position [ft]','fontsize', 14);
set(gca,'fontsize', 14);
set(gcf,'color','w');
grid on 
%
% Plot AP response
aP1_FC_fps2 = x(:,sel_aP1_FC_fps2);
aP2_FC_fps2 = x(:,sel_aP2_FC_fps2);
aP3_FC_fps2 = x(:,sel_aP3_FC_fps2);
plot(t_sec(1:miss_index), aP1_FC_fps2(1:miss_index), 'b', 'linewidth', 2); hold on 
plot(t_sec(1:miss_index), aPi1_fps2(1:miss_index), 'b--', 'linewidth', 2);
plot(t_sec(1:miss_index), aP2_FC_fps2(1:miss_index), 'r', 'linewidth', 2);
plot(t_sec(1:miss_index), aPi2_fps2(1:miss_index), 'r--', 'linewidth', 2);
plot(t_sec(1:miss_index), aP3_FC_fps2(1:miss_index), 'k','linewidth', 2);
plot(t_sec(1:miss_index), aPi3_fps2(1:miss_index), 'k--', 'linewidth', 2);
xlabel('Time [sec]','fontsize', 14);
ylabel('FC Response [ft/s^2]','fontsize', 14);
set(gca,'fontsize', 14, 'ylim', [-1000 1000]);
set(gcf,'color','w');
grid on 
%}
end
