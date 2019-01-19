function [state, A, B, C, Q, R, plant_params, trajectorydes, rpms] = LQRinitialize(N)


%% True plant parameters
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

%% Define state struct
state = struct(...    
    'x', zeros(1,N), ...
    'y', zeros(1,N), ...
    'z', zeros(1,N), ...
    'roll', zeros(1,N), ...
    'pitch', zeros(1,N), ...
    'yaw', zeros(1,N), ...
    'x_d', zeros(1,N), ...
    'y_d', zeros(1,N), ...
    'z_d', zeros(1,N), ...
    'roll_d', zeros(1,N), ...
    'pitch_d', zeros(1,N), ...
    'yaw_d', zeros(1,N) );

g = plant_params.gravity;
i = plant_params.inertia;
syms yaw yaw_d ;

A = [ 0,0,0,0,0,0,1,0,0,0,0,0 ;
      0,0,0,0,0,0,0,1,0,0,0,0 ;
      0,0,0,0,0,0,0,0,1,0,0,0 ;
      0,0,0,0,0,0,0,0,0,1,0,0 ;
      0,0,0,0,0,0,0,0,0,0,1,0 ;
      0,0,0,0,0,0,0,0,0,0,0,1 ;
      0,0,0,g*sin(yaw),g*cos(yaw),0,0,0,0,0,0,0 ;
      0,0,0,-g*cos(yaw),g*sin(yaw),0,0,0,0,0,0,0 ;
      0,0,0,0,0,0,0,0,0,0,0,0 ;
      0,0,0,((i(2,2)- i(3,3))*(yaw_d^2))/i(1,1),0,0,0,0,0,0,((i(2,2)- i(3,3))*yaw_d)/i(1,1),0 ;
      0,0,0,0,((i(1,1)- i(3,3))*(yaw_d^2))/i(2,2),0,0,0,0,-((i(1,1)- i(3,3))*yaw_d)/i(2,2),0,0 ;
      0,0,0,0,0,0,0,0,0,0,0,0   ];
  
% disp(vpa(A, 4));

matlabFunction(A, 'File', 'A_equation', 'Vars', ...
      [yaw yaw_d ] );

B  = [ 0,0,0,0 ;
       0,0,0,0 ;
       0,0,0,0 ;
       0,0,0,0 ;
       0,0,0,0 ;
       0,0,0,0 ;
       0,0,0,0 ;
       0,0,0,0 ;
       1/plant_params.mass,0,0,0 ;
       0,1/i(1,1),0,0 ;
       0,0,1/i(2,2),0 ;
       0,0,0,1/i(3,3)  ];
   
% disp(B);

C = [ 1,0,0,0,0,0,0,0,0,0,0,0  ;
      0,1,0,0,0,0,0,0,0,0,0,0  ; 
      0,0,1,0,0,0,0,0,0,0,0,0  ;
      0,0,0,0,0,1,0,0,0,0,0,0  ];
  
% disp(C);

Q = 0.1 * eye(12);
Q(1,1)= 17;
Q(2,2)= 17;
Q(3,3)= 20;
Q(6,6)= 80;

R = 1 * eye(4);
R(1,1)= 0.001;

%% Set initial conditions
% trajectorydes format - (posdes, veldes, accdes, jerkdes, snapdes, rpydes, rpy_d_des, rpy_dd_des)
trajectorydes = zeros(3,8,N);

init_rpm = plant_params.rpm_min*ones(4,1);
rpms = zeros(4,N);
rpms(:,1) = init_rpm;

end