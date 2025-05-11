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

end
