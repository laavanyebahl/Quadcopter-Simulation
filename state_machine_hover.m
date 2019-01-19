function [trajectorydes, trajectoryact, rpms_current] = state_machine_hover(time, timestep, rpms_current, trajectoryact_last_val)

%% Initialize everything
N = (time/timestep)+1;
[plant_params, ctrl, trajectorydes, trajectoryact, rpms] = PDinitialize(N);

trajectoryact(:,1,1) = trajectoryact_last_val(:, 1,1) ;
x = trajectoryact_last_val( 1, 1, 1);
y = trajectoryact_last_val( 2, 1, 1);
z = trajectoryact_last_val( 3, 1, 1);

%% Create Trajectory
% trajectorydes format - (posdes, veldes, accdes, jerkdes, snapdes, rpydes, rpy_d_des, rpy_dd_des, rpmdes)
trajectorydes( 1, 1 , :) =  x;
trajectorydes( 2, 1 , :) =  y;
trajectorydes( 3, 1 , :) =  z;

%% Run controller
[trajectorydes, trajectoryact, rpms_current] = PDcontroller(plant_params, ctrl, N, timestep, trajectorydes, trajectoryact,rpms_current );
end