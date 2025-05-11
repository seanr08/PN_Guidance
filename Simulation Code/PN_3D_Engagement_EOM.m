function dx = PN_3D_Engagement_EOM(t, x, tf_sec, N, ...
    fps2_limit, K_delta, M_delta, V, I_yy, omega_a, zeta_a, ...
    noise, TF_par, aT_fps2, omega_rps)

% Pointers to states
sel_RP1_ft       = 1;
sel_RP2_ft       = 2;
sel_RP3_ft       = 3;
sel_VP1_fps      = 4;
sel_VP2_fps      = 5;
sel_VP3_fps      = 6;
sel_RT1_ft       = 7;
sel_RT2_ft       = 8;
sel_RT3_ft       = 9;
sel_VT1_fps      = 10;
sel_VT2_fps      = 11;
sel_VT3_fps      = 12;
sel_aP1_FC_fps2  = 13;
sel_aP2_FC_fps2  = 14;
sel_aP3_FC_fps2  = 15;
sel_n1           = 16;
sel_n2           = 17;
sel_n3           = 18;

% Time to go
tgo_sec = tf_sec - t;

% Preallocate state vector
dx = zeros(15,1);
tau_n = 1/3;

if noise == true
    % Measurement Error Model
    sigma_r = 10*3.28;       % ft stdev on range
    sigma_los = (pi/180)*1;   % rad/s stdev on LOS‐rate
    bound_r = 3*sigma_r;    % Error bound for range measurement
    bound_los = 3*sigma_los;  % Error bound for LOS rate measurement
    noise_r = sigma_r*randn(3,1);
    noise_r = max(min(noise_r, bound_r), -bound_r);
    noise_los = sigma_los*randn(3,1);
    noise_los = max(min(noise_los, bound_los), -bound_los);

    % Wind Model
    sigma_w = 4^2;     % ft/s^2 stdev of wind‐gust
    bound_w = 3*sigma_w;
    w_dist = sigma_w*randn(3,1);
    w_dist = max(min(w_dist, bound_w), -bound_w);
else
    % Measurement Error Model
    sigma_r = 0;     % ft stdev on range
    sigma_los = 0;    % ft/s stdev on LOS‐rate
    noise_r = sigma_r*randn(3,1);
    noise_los = sigma_los*randn(3,1);

    % Wind Model
    sigma_w = 0;     % ft/s^2 stdev of wind‐gust
    w_dist = sigma_w*randn(3,1);
end

% Relative positions and velocities
RTP1_ft = x(sel_RT1_ft) - x(sel_RP1_ft);
RTP2_ft = x(sel_RT2_ft) - x(sel_RP2_ft);
RTP3_ft = x(sel_RT3_ft) - x(sel_RP3_ft);
RTPi_ft = [RTP1_ft, RTP2_ft, RTP3_ft];

VTP1_fps = x(sel_VT1_fps) - x(sel_VP1_fps);
VTP2_fps = x(sel_VT2_fps) - x(sel_VP2_fps);
VTP3_fps = x(sel_VT3_fps) - x(sel_VP3_fps);

% ZEM resolved in the inertial cs
ZEMi1_ft = (RTP1_ft + noise_r(1)) + VTP1_fps*tgo_sec;
ZEMi2_ft = (RTP2_ft + noise_r(2)) + VTP2_fps*tgo_sec;
ZEMi3_ft = (RTP3_ft + noise_r(3)) + VTP3_fps*tgo_sec;
ZEMi_ft  = [ZEMi1_ft, ZEMi2_ft, ZEMi3_ft];

% Unit vector along line of sight
LOS_Unit_Vector = (RTPi_ft')./norm((RTPi_ft'),2);
if noise == true
    delta_los = noise_los*tau_n;
    randvec = randn(3,1);
    randaxis = randvec - (LOS_Unit_Vector' * randvec)*LOS_Unit_Vector;
    randaxis = randaxis./norm(randaxis);
    LOS_noise = cos(delta_los).*LOS_Unit_Vector + sin(delta_los).*(cross(randaxis,LOS_Unit_Vector));
else
    LOS_noise = LOS_Unit_Vector;
end
LOS_Unit_Vector = LOS_noise';

% ZEM in the r direction resolved in the inertial cs
ZEMr_ft = (ZEMi_ft*LOS_Unit_Vector')*LOS_Unit_Vector;

% Zero effort miss perpindicular to the line of sight
ZEMn_ft = ZEMi_ft - ZEMr_ft;

% Kinematic equations
dx(sel_RP1_ft) = x(sel_VP1_fps) + w_dist(1);
dx(sel_RP2_ft) = x(sel_VP2_fps) + w_dist(2);
dx(sel_RP3_ft) = x(sel_VP3_fps) + w_dist(3);

%aPi1_fps2 = N*ZEMn_ft(1)/tgo_sec^2;
%aPi2_fps2 = N*ZEMn_ft(2)/tgo_sec^2;
%aPi3_fps2 = N*ZEMn_ft(3)/tgo_sec^2;

% Acceleration Commands
aPi1_fps2 = N*ZEMn_ft(1)/tgo_sec^2;
aPi2_fps2 = N*ZEMn_ft(2)/tgo_sec^2;
aPi3_fps2 = N*ZEMn_ft(3)/tgo_sec^2;

% Zero the axial acceleration
aPi_raw = [ aPi1_fps2;
            aPi2_fps2;
            aPi3_fps2];

% --- get current pursuer velocity vector and unitize ---
vP     = [ x(sel_VP1_fps)
           x(sel_VP2_fps)
           x(sel_VP3_fps) ];          % fps
uV     = vP / norm(vP);               % unit‐vector along velocity

% --- decompose raw command into axial + lateral parts ---
a_para = (uV'*aPi_raw)*uV;            % component along uV
a_lat  = aPi_raw - a_para;            % everything perpendicular to uV

dec_rate = -32;
if norm(vP) > 2000
    a_para = a_para - dec_rate*uV;
end

% --- apply acceleration limit to the lateral magnitude only ---
a_lat_mag = norm(a_lat);
if a_lat_mag > fps2_limit
    a_lat = a_lat * (fps2_limit / a_lat_mag);
end

% --- now a_lat is purely lateral, within g‐limit ---
aPi1_fps2 = a_lat(1);
aPi2_fps2 = a_lat(2);
aPi3_fps2 = a_lat(3);

%aPi1_fps2 = min([fps2_limit, abs(aPi1_fps2)])*sign(aPi1_fps2);
%aPi2_fps2 = min([fps2_limit, abs(aPi2_fps2)])*sign(aPi2_fps2);
%aPi3_fps2 = min([fps2_limit, abs(aPi3_fps2)])*sign(aPi3_fps2);

aPi = [ aPi1_fps2; aPi2_fps2; aPi3_fps2 ];

n_tf = size(TF_par.A,1);
sel_tf = 16 : 15 + n_tf;

A = TF_par.A;
B = TF_par.B;
C = TF_par.C;
D = TF_par.D;

x_tf = x(sel_tf);
dx_tf = A*x_tf + B*aPi;
a_act = C*x_tf + D*aPi;
a_act = max(min(a_act, fps2_limit), -fps2_limit);
dx(sel_tf) = dx_tf;

x(sel_aP1_FC_fps2) = a_act(1);
x(sel_aP2_FC_fps2) = a_act(2);
x(sel_aP3_FC_fps2) = a_act(3);

x(sel_aP1_FC_fps2)=a_act(1);
x(sel_aP2_FC_fps2)=a_act(2);
x(sel_aP3_FC_fps2)=a_act(3);

%%–– kinematics:  dV/dt = aP
dx(sel_VP1_fps) = x(sel_aP1_FC_fps2) + a_para(1);
dx(sel_VP2_fps) = x(sel_aP2_FC_fps2) + a_para(2);
dx(sel_VP3_fps) = x(sel_aP3_FC_fps2) + a_para(3);

dx(sel_RT1_ft) = x(sel_VT1_fps);
dx(sel_RT2_ft) = x(sel_VT2_fps);
dx(sel_RT3_ft) = x(sel_VT3_fps);

dx(sel_VT1_fps) = aT_fps2*sin(omega_rps*t);
dx(sel_VT2_fps) = 0;
dx(sel_VT3_fps) = aT_fps2*cos(omega_rps*t);