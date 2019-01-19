function [track_trajectorydes, track_trajectoryact, total_time, trajectorydes_total, trajectoryact_total] = state_machine(trajectorydes_test, timestep, track_trajectory_time, take_off_height)

%% Initialize everything
total_time = 0;
state_transition_allowed_error = 0.005;
trajectorydes_total = [];
trajectoryact_total = [];
trajectoryact_last_val = zeros(3,8,1);
rpms_current = zeros(4,1);
states = struct( 'idle', 'idle', 'take_off', 'take_off', 'hover', 'hover', 'track_trajectory', 'track_trajectory', 'land', 'land');

%% Sample input values
idle_time= 2;
hover_time = 5;

%% Follow the state machine sequence
disp('Path started');
disp(['Current position = ', num2str(trajectoryact_last_val(:,1,1)'), ' , current rpms = ', num2str(rpms_current')]);

current_state = states.idle;

%% IDLE
disp(['Current state = ', current_state]);
[trajectorydes, trajectoryact, rpms_current] = state_machine_idle(idle_time, timestep, trajectoryact_last_val);
trajectoryact_last_val = trajectoryact(:,:,end);
trajectorydes_total = cat(3, trajectorydes_total, trajectorydes(:,:, 1:end-1));
trajectoryact_total = cat(3, trajectoryact_total, trajectoryact(:,:, 1:end-1));
total_time = total_time + idle_time;
disp(['Stayed idle for ', num2str(idle_time), ' seconds.']);
current_state = states.take_off;
disp(['Current position = ', num2str(trajectoryact_last_val(:,1,1)'), ' , current rpms = ', num2str(rpms_current')]);

%% TAKE-OFF
disp(['Current state = ', current_state]);
[trajectorydes, trajectoryact, rpms_current, desired_height_reached_time] = state_machine_take_off( timestep, take_off_height, state_transition_allowed_error);
take_off_time = desired_height_reached_time;
trajectoryact_last_val = trajectoryact(:,:,end);
trajectorydes_total = cat(3, trajectorydes_total, trajectorydes(:,:, 1:end-1));
trajectoryact_total = cat(3, trajectoryact_total, trajectoryact(:,:, 1:end-1));
total_time = total_time + take_off_time;
disp(['Took off for ', num2str(take_off_time), ' seconds, to reach height ', num2str(take_off_height),' m.']);
current_state = states.hover;
disp(['Current position = ', num2str(trajectoryact_last_val(:,1,1)'), ' , current rpms = ', num2str(rpms_current')]);

%% HOVER
disp(['Current state = ', current_state]);
[trajectorydes, trajectoryact, rpms_current] = state_machine_hover(hover_time, timestep, rpms_current, trajectoryact_last_val);
trajectoryact_last_val = trajectoryact(:,:,end);
trajectorydes_total = cat(3, trajectorydes_total, trajectorydes(:,:, 1:end-1));
trajectoryact_total = cat(3, trajectoryact_total, trajectoryact(:,:, 1:end-1));
total_time = total_time + hover_time;
disp(['Hovered for ', num2str(hover_time), ' seconds, at height ', num2str(trajectoryact_last_val(3,1,1)) ,' m.']);
current_state = states.track_trajectory;
disp(['Current position = ', num2str(trajectoryact_last_val(:,1,1)'), ' , current rpms = ', num2str(rpms_current')]);

%% TRACK TRAJECTORY
disp(['Current state = ', current_state]);
[trajectorydes, trajectoryact, rpms_current] = state_machine_track_trajectory(track_trajectory_time, timestep, rpms_current, trajectorydes_test, trajectoryact_last_val);
trajectoryact_last_val = trajectoryact(:,:,end);
track_trajectorydes = trajectorydes;  % Store and return track_trajectorydes
track_trajectoryact = trajectoryact;  % Store and return track_trajectoryact
trajectorydes_total = cat(3, trajectorydes_total, trajectorydes(:,:, 1:end-1));
trajectoryact_total = cat(3, trajectoryact_total, trajectoryact(:,:, 1:end-1));
total_time = total_time + track_trajectory_time;
current_state = states.land;
disp(['Tracked trajectory for ', num2str(track_trajectory_time), ' seconds.']);
disp(['Current position = ', num2str(trajectoryact_last_val(:,1,1)'), ' , current rpms = ', num2str(rpms_current')]);

%% LAND
disp(['Current state = ', current_state]);
[trajectorydes, trajectoryact, rpms_current, land_time] = state_machine_land( timestep, rpms_current, trajectoryact_last_val);
disp(['Landed in ', num2str(land_time), ' seconds, from height ', num2str(trajectoryact_last_val(3,1,1)) ,' m.']);
trajectoryact_last_val = trajectoryact(:,:,end);
trajectorydes_total = cat(3, trajectorydes_total, trajectorydes(:,:, 1:end-1));
trajectoryact_total = cat(3, trajectoryact_total, trajectoryact(:,:, 1:end-1));
total_time = total_time + land_time;
current_state = states.idle;
disp(['Current position = ', num2str(trajectoryact_last_val(:,1,1)'), ' , current rpms = ', num2str(rpms_current')]);

%% IDLE
disp(['Current state = ', current_state]);
[trajectorydes, trajectoryact, rpms_current] = state_machine_idle(idle_time, timestep, trajectoryact_last_val);
trajectoryact_last_val = trajectoryact(:,:,end);
trajectorydes_total = cat(3, trajectorydes_total, trajectorydes(:,:, 1:end-1));
trajectoryact_total = cat(3, trajectoryact_total, trajectoryact(:,:, 1:end-1));
total_time = total_time + idle_time;
disp(['Stayed idle for ', num2str(idle_time), ' seconds.']);
disp(['Current position = ', num2str(trajectoryact_last_val(:,1,1)'), ' , current rpms = ', num2str(rpms_current')]);

disp('Path completed,  !');

end