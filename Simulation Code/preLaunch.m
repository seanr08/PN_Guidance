function [RT_L, VT_L, u0] = ...
         preLaunch(RT0, VT0, Vm, GR_thresh)
% planLaunchAndIntercept  
%   Launch when target ground-range ≥ GR_thresh, then compute
%   the missile’s initial heading to intercept the moving target.
%
% Inputs:
%   RT0       – 3×1 initial target position [x;y;z] (ft)
%   VT0       – 3×1 target velocity               (ft/s)
%   Vm        – pursuer launch speed               (ft/s)
%   GR_thresh – ground-range launch threshold      (ft)
%
% Outputs:
%   doLaunch  – 1 if/when launch+intercept solution exists
%   t_launch  – launch time (s) (0 if already ≥GR_thresh)
%   RT_L      – target pos at launch               (ft)
%   VT_L      – target vel at launch               (ft/s)
%   u0        – 3×1 missile initial heading unit-vector
%   t_int     – time from launch to intercept      (s)

  % defaults
  doLaunch = false;
  t_launch = [];
  RT_L     = [];
  VT_L     = [];
  u0       = [0;0;0];
  t_int    = [];

  random_err = randn;

  % 1) find launch time based on ground-range
  R0g = RT0(1:2)+max(min(RT0(1:2)*random_err,0.05*RT0(1:2)), -0.05*RT0(1:2));  Vtg = VT0(1:2);
  if norm(R0g) >= GR_thresh
    t_launch = 0;
  else
    a = Vtg.'*Vtg;
    b = 2*(R0g.'*Vtg);
    c = R0g.'*R0g - GR_thresh^2;
    sol = roots([a b c]);
    sol = sol(imag(sol)==0 & sol>=0);
    if isempty(sol)
      return   % never reaches ground-range
    end
    t_launch = min(sol);
  end

  % 2) propagate target to launch
  RT_L = RT0+max(min(RT0*random_err,0.05*RT0)) + VT0 * t_launch;
  VT_L = VT0;

  % 3) solve pure-pursuit intercept: ||RT_L + VT_L*t|| = Vm*t
  %    => (VT_L·VT_L – Vm^2) t^2 + 2(RT_L·VT_L) t + (RT_L·RT_L)=0
  a2 = VT_L.'*VT_L - Vm^2;
  b2 = 2*(RT_L.'*VT_L);
  c2 = RT_L.'*RT_L;
  sol2 = roots([a2 b2 c2]);
  sol2 = sol2(imag(sol2)==0 & sol2>0);
  %
  if isempty(sol2)
    return   % no intercept possible at this speed
  end
  %}
  t_int = min(sol2);

  % 4) heading unit-vector
  P_int = RT_L + VT_L * t_int;
  if norm(P_int)==0
    return
  end
  u0 = P_int / norm(P_int);

  doLaunch = true;
end