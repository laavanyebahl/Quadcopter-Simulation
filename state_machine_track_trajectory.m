function [trajectorydes, trajectoryact, rpms_current] = state_machine_track_trajectory(time, timestep, rpms_current, trajectorydes, trajectoryact_last_val)

%% Initialize everything
N = (time/timestep)+1;
N = int16(N);
[plant_params, ctrl, trajectorydes_init, trajectoryact, rpms] = PDinitialize(N);

trajectoryact(:,1,1) = trajectoryact_last_val(:, 1,1) ;

%% Run controller
[trajectorydes, trajectoryact, rpms_current] = PDcontroller(plant_params, ctrl, N, timestep, trajectorydes, trajectoryact, rpms_current );
end