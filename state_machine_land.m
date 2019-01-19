function [trajectorydes, trajectoryact, rpms_current, time] = state_machine_land( timestep, rpms_current, trajectoryact_last_val)

%% Initialize everything
time =2;
N = (time/timestep)+1;
[plant_params, ctrl, trajectorydes, trajectoryact, rpms] = PDinitialize(N);

trajectoryact(:,1,1) = trajectoryact_last_val(:, 1,1) ;
x = trajectoryact( 1, 1, 1);
y = trajectoryact( 2, 1, 1);
z = trajectoryact( 3, 1, 1);

%% Create Trajectory
% trajectorydes format - (posdes, veldes, accdes, jerkdes, snapdes, rpydes, rpy_d_des, rpy_dd_des, rpmdes)
trajectorydes( 1, 1 , :) =  x;
trajectorydes( 2, 1 , :) =  y;
trajectorydes( 3, 1 , 1:int16(N/3)) =  linspace( z, 0, int16(N/3));
trajectorydes( 3, 1 , int16(N/3):end) = 0;
%% Run controller
[trajectorydes, trajectoryact, rpms_current] = PDcontroller(plant_params, ctrl, N, timestep, trajectorydes, trajectoryact, rpms_current );
end