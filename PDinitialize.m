function [plant_params, ctrl, trajectorydes, trajectoryact, rpms] = PDinitialize(N) 

%% True plant parameters
% Motor spread angle: Angle between two front motors
% Thrust coefficient: c_T in the slides
% Moment scale: c_Q = moment_scale*c_T
% Motor constant: wdot = -k_m (rpm - rpm_des)
% zoffset: ignore (use in example to represent ground)
plant_params = struct(...
    'mass',                   0.770, ...
    'gravity',                9.80665, ...
    'arm_length',             0.1103, ...
    'motor_spread_angle',     0.925, ...
    'thrust_coefficient',     8.07e-9, ...
    'moment_scale',           0.017, ...
    'motor_constant',        36.5, ...
    'rpm_min',             3000, ...
    'rpm_max',            20000, ...
    'inertia',            diag([0.0033 0.0033 0.005]),...
    'zoffset',                0.0);

% --------------------  QUESTION 7  --------------------

% CASE 4 
% plant_params.motor_constant = 4*plant_params.motor_constant;
% plant_params.rpm_max = 4*plant_params.rpm_max;

%% Create reference trajectory
% waypoints are [x y z yaw]
wpvec = [ 0  0 0.2 0;
          0  0 0.8 0;
         -0.5 0.5 1.0 0;
         -1  1 1.0 pi/2;
         1.5 1 1.0 pi;
         1.5 -1 0.5 pi];


ctrl = struct();

% --------------------------  QUESTION 2 ----------------------------
% % CASE 0: default gain values
% ctrl.Kp = [17 17 20];
% ctrl.Kv = [6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [30 30 17.88];
% CASE 1: Tuned gain values
ctrl.Kp = [13 17 20];
ctrl.Kv = [6.6 6.6 9];
ctrl.Kr = [190 198 80];
ctrl.Kw = [30 30 17.88];
% % CASE 2: Increase Kp and Kr (2.5 times)
% ctrl.Kp = 2.5*[17 17 20];
% ctrl.Kv = [6.6 6.6 9];
% ctrl.Kr = 2.5*[190 198 80];
% ctrl.Kw = [30 30 17.88];
% % CASE 3: Decrease Kp and Kr (0.6 times)
% ctrl.Kp = 0.6*[17 17 20];
% ctrl.Kv = [6.6 6.6 9];
% ctrl.Kr = 0.6*[190 198 80];
% ctrl.Kw = [30 30 17.88];
% % CASE 4: Increase Kv and Kw (2.5 times)
% ctrl.Kp = [17 17 20];
% ctrl.Kv = 2.5*[6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = 2.5*[30 30 17.88];
% % CASE 5: Decrease Kv and Kw (0.6 times)
% ctrl.Kp = [17 17 20];
% ctrl.Kv = 0.6*[6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = 0.6*[30 30 17.88];


% ------------------  QUESTION 3 AND QUESTION 5  ---------------
% % CASE 0: tuned gain values
% ctrl.Kp = [13 17 20];
% ctrl.Kv = [6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [30 30 17.88];
% % CASE 1: Increase Kp (2.5 times)
% ctrl.Kp = 2.5*[17 17 20];
% ctrl.Kv = [6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [30 30 17.88];
% % CASE 2: Decrease Kp (0.6 times)
% ctrl.Kp = 0.6*[17 17 20];
% ctrl.Kv = [6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [30 30 17.88];
% % CASE 3: Increase Kv (2.5 times)
% ctrl.Kp = [17 17 20];
% ctrl.Kv = 2.5*[6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [30 30 17.88];
% % CASE 4: Decrease Kv (0.6 times)
% ctrl.Kp = [17 17 20];
% ctrl.Kv = 0.6*[6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [30 30 17.88];

% -----------------  QUESTION 10 -------------------

% CASE 1 

% ctrl.Kp = [13 17 20];
% ctrl.Kv = [3 3 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [15 15 9];

% CASE 2

% ctrl.Kp = [13 17 20];
% ctrl.Kv = [12 12 11];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [40 40 25];

ctrl.inertia = plant_params.inertia;
ctrl.ct = plant_params.thrust_coefficient;
ctrl.ms = plant_params.moment_scale;

%% Set initial conditions
% trajectorydes format - (posdes, veldes, accdes, jerkdes, snapdes, rpydes, rpy_d_des, rpy_dd_des)
% trajectoryact format - (posact, velact, accact, jerkact, snapact, rpyact, rpy_d_act, rpy_dd_act)
trajectoryact = zeros(3,8,N);
trajectorydes = zeros(3,8,N);

init_rpm = plant_params.rpm_min*ones(4,1);
rpms = zeros(4,N);
rpms(:,1) = init_rpm;

end