function [trajectorydes, trajectoryact, rpms_current] = state_machine_idle(time, timestep, trajectoryact_last_val)

%% Initialize everything
N = (time/timestep)+1;
[plant_params, ctrl, trajectorydes, trajectoryact, rpms] = PDinitialize(N);

trajectoryact(:,1,1) = trajectoryact_last_val(:, 1,1) ;
x = trajectoryact( 1, 1, 1);
y = trajectoryact( 2, 1, 1);
z = 0;

%% Create Trajectory
% trajectorydes format - (posdes, veldes, accdes, jerkdes, snapdes, rpydes, rpy_d_des, rpy_dd_des, rpmdes)
trajectorydes( 1, 1 , :) =  x;
trajectorydes( 2, 1 , :) =  y;
trajectorydes( 3, 1 , :) =  z;

trajectoryact( 1, 1 , :) =  x;
trajectoryact( 2, 1 , :) =  y;
trajectoryact( 3, 1 , :) =  z;

rpms_current = rpms(:, 1);

end