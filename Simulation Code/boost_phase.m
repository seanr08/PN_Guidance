function [ RP_boost, VP_boost, RT_boost, VT_boost, t_boost, ...
           RP_hist, VP_hist, RT_hist, VT_hist, t_hist ] = boost_phase( ...
    RP0, RT0, VT0, a_boost_fps2, v_desired, dir, dt )

  % Estimate max number of steps so we can pre-allocate
  t_max    = v_desired / a_boost_fps2;
  n_steps  = ceil( t_max / dt ) + 1;

  % Pre-allocate history arrays
  RP_hist = zeros(n_steps,3);
  VP_hist = zeros(n_steps,3);
  RT_hist = zeros(n_steps,3);
  VT_hist = zeros(n_steps,3);
  t_hist  = zeros(n_steps,1);

  % Initialize
  RP = RP0(:)';
  VP = [0 0 0];
  RT = RT0(:)';
  VT = VT0(:)';
  t  = 0;
  k  = 1;

  % Store initial state
  RP_hist(k,:) = RP;
  VP_hist(k,:) = VP;
  RT_hist(k,:) = RT;
  VT_hist(k,:) = VT;
  t_hist(k)    = t;

  % Main boost loop
  while norm(VP) < v_desired
    k = k + 1;
    % Update states
    VP = VP + (a_boost_fps2*dt) * dir(:)';   % accelerate in 'dir'
    RP = RP + VP * dt;                      % pursuer position
    RT = RT + VT * dt;                      % propagate target
    % VT stays constant (no target accel in this phase)
    t  = t + dt;

    % Store into history
    RP_hist(k,:) = RP;
    VP_hist(k,:) = VP;
    RT_hist(k,:) = RT;
    VT_hist(k,:) = VT;
    t_hist(k)    = t;
  end

  % Trim off the unused pre-allocated rows
  RP_hist = RP_hist(1:k,:);
  VP_hist = VP_hist(1:k,:);
  RT_hist = RT_hist(1:k,:);
  VT_hist = VT_hist(1:k,:);
  t_hist  = t_hist(1:k);

  % Return final “boost‐out” state
  RP_boost = RP;
  VP_boost = VP;
  RT_boost = RT;
  VT_boost = VT;
  t_boost  = t;
end