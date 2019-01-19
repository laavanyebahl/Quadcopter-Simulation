function [trajectorydes_takeoff, trajectoryact_takeoff, rpms_current, desired_height_reached_time] = state_machine_take_off( timestep, take_off_height, state_transition_allowed_error)

%% Initialize everything
time = 2;
N = (time/timestep)+1;
[plant_params, ctrl, trajectorydes, trajectoryact, rpms] = PDinitialize(N);

try_height = take_off_height + 0.02;

trajectorydes_total = [];
trajectoryact_total = [];
total_time = 0;
current_height = 0;

%% Create Trajectory
trajectorydes( 3, 1 , 1:N) = try_height;

while( abs(current_height - take_off_height) > state_transition_allowed_error)
    %% Run controller
    [trajectorydes, trajectoryact, rpms_current] = PDcontroller(plant_params, ctrl, N, timestep, trajectorydes, trajectoryact, rpms );
    trajectorydes_total = cat(3, trajectorydes_total, trajectorydes(:,:, 1:end-1));
    trajectoryact_total = cat(3, trajectoryact_total, trajectoryact(:,:, 1:end-1));
    current_height = trajectoryact(3,1,end); 
    if current_height > take_off_height
        N_temp = (total_time+time)/timestep;
        for i=1:N_temp
            if trajectoryact(3,1,i) > take_off_height
                desired_height_reached_time = total_time + i*timestep;
                trajectorydes_takeoff = zeros(3,8,desired_height_reached_time*(1/timestep));
                trajectoryact_takeoff = zeros(3,8,desired_height_reached_time*(1/timestep));

                trajectorydes_takeoff(:,:,:) = trajectorydes_total(:,:,1:desired_height_reached_time*(1/timestep));
                trajectoryact_takeoff(:,:,:) = trajectoryact_total(:,:,1:desired_height_reached_time*(1/timestep));
               
                trajectorydes_takeoff(3,1,end) = take_off_height;
                trajectoryact_takeoff(3,1,end) = take_off_height;
                
                trajectorydes_takeoff(3,1,:)= take_off_height;

                break;
            end
        end
        break;
    else
        total_time = total_time + time;
    end
end

end