close all;
clear;
clc;
addpath('..');

n = 7;
r = 4;

% time cycle is 3 for 1 quarter, 12 for 1 cycle, 48 in total
timestep = 0.01;
dt= 2;
num_rounds = 3;
num_waypoints = 4;

N = (num_waypoints*num_rounds*dt)/timestep;
disp(N);
trajdes = zeros(r+1,4, (num_waypoints*num_rounds*dt)/timestep );

time = 0;


points = zeros(1, num_waypoints*num_rounds );
points(1,1) = timestep;

% CASE 1
% velocity_profile = 1;

% CASE 2
velocity_profile = 1.5;


for i=1:num_waypoints*(num_rounds-1)
    points(1,i+1) = (i)*dt;
    points(1,i+2) = (i+1)*dt;
    points(1,i+3) = (i+2)*dt;
    points(1,i+4) = (i+3)*dt;
end
points_N = (1/timestep)*points;
% disp(points_N);

x = [0, 2, 0, -2, 0, 2, 0, -2, 0, 2, 0, -2, 0, 2, 0, -2, 0 ];
y = [0, 1, 2, 1, 0, 1, 2, 1, 0, 1, 2, 1, 0, 1, 2, 1, 0  ];
v_x = [0, 0, -velocity_profile, 0, velocity_profile, 0, -velocity_profile, 0, velocity_profile, 0, -velocity_profile, 0, velocity_profile, 0, -velocity_profile, 0, velocity_profile ];
v_y = [0, velocity_profile, 0, -velocity_profile, 0, velocity_profile, 0, -velocity_profile, 0, velocity_profile, 0, -velocity_profile, 0, velocity_profile, 0, -velocity_profile, 0 ];

% constraints format = [ value, time, order]
% inequality_constraint format = [ less than this term, time, order]

round = 0;
inequality_constraint_x = [];
inequality_constraint_y = [];
    
for i=1:(num_waypoints*num_rounds)
    
    disp(i);
  
    constraints_x = [ x(i), 0, 0; x(i+1), dt, 0; v_x(i), 0, 1; v_x(i+1), dt, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
    constraints_y = [ y(i), 0, 0; y(i+1), dt, 0; v_y(i), 0, 1; v_y(i+1), dt, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
    
    [ trajdes(:,1,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_x_val ] = generate_trajectory(n, r, constraints_x, inequality_constraint_x, timestep);
    [ trajdes(:,2,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_y_val ]= generate_trajectory(n, r, constraints_y, inequality_constraint_y, timestep);
    
%     disp(v_x_val);
%     disp(v_y_val);
    
    if mod(i,4)==0
        round = round+1;
        x_val = zeros(1,100*dt*num_waypoints);
        y_val = zeros(1,100*dt*num_waypoints);
        start = time-(dt*(num_waypoints-1));
        for i=1:(dt*100*num_waypoints)
            x_val(1,i) = trajdes(1,1,start*100+i);
            y_val(1,i) = trajdes(1,2,start*100+i);
        end
        figure;
        plot(x_val, y_val );
        title([' tangential trajectory for round : ' num2str(round)]);
        grid on;
        daspect([1 1 1]);
    end
       
    time = time+dt;
end


%% Input values
track_trajectory_time = dt*num_waypoints*num_rounds;
take_off_height = 1;
take_off_yaw = -pi/2;

%% Generate trajectory for track_trajectory state from trajdes
trajectory_N = (track_trajectory_time/timestep)+1 ;
trajectorydes_test = zeros(3, 8, trajectory_N);

for i=1:4
    for k=1:2   % For x and y
         trajectorydes_test(k, i, 1:end-1) = trajdes(i,k,:);
         trajectorydes_test(k, i, end) = trajdes(i,k,end);
    end
end
trajectorydes_test(3, 1, 1:end-1) = take_off_height;
trajectorydes_test(3, 1, end) = take_off_height;

%% Add heading to centre condition 

offset = pi;
psi_des = pi/2;

for i=1:trajectory_N
%     if psi_des<pi
%         psi_des =  atan2( trajectorydes_test(2, 1, i) - (y(3)/2), trajectorydes_test(1, 1, i))  ;
%     else
%         psi_des = offset + atan2( trajectorydes_test(2, 1, i) - (y(3)/2), trajectorydes_test(1, 1, i))  ;
%     end

    if mod(i,200)==0
        disp('-------------------------------------')
        disp(i);
    end
%     disp(atan2( trajectorydes_test(2, 1, i) - (y(3)/2), trajectorydes_test(1, 1, i)) );
    psi = atan2( trajectorydes_test(2, 1, i) - (y(3)/2), trajectorydes_test(1, 1, i))  ;
  
    disp(psi);
    if  psi >= -3.1366 && psi <= -3.1316   % 4th waypoint , angle is -180
        offset = offset +  deg2rad(360);
    end
    psi_des = offset + atan2( trajectorydes_test(2, 1, i) - (y(3)/2), trajectorydes_test(1, 1, i))  ;

%     psi_des = offset + atan2( trajectorydes_test(2, 1, i) - (y(3)/2), trajectorydes_test(1, 1, i))  ;

%     if  psi_des< deg2rad(90)
%         psi_des = deg2rad(90) + psi_des ;
%     end
    trajectorydes_test(3, 6, i) =  psi_des  ;
    disp(trajectorydes_test(3, 6, i));
end


%% Start the state machine
[track_trajectorydes, track_trajectoryact, total_time, trajectorydes_total, trajectoryact_total] = state_machine(trajectorydes_test, timestep, track_trajectory_time, take_off_height);

%% Plotting only track_trajectory path
tvec=0:timestep:track_trajectory_time;
N = numel(tvec);

posdes = zeros(3, N);
rpydes = zeros(3, N);
posvec = zeros(3, N);
rpyvec = zeros(3, N);

posvec(:,:)  = track_trajectoryact(:,1,:);
rpyvec(:,:)  = track_trajectoryact(:,6,:);
velvec(:,:)  = track_trajectoryact(:,2,:);
avlvec(:,:)  = track_trajectoryact(:,7,:);
accvec(:,:)  = track_trajectoryact(:,3,:);

posdes(:,:)  = track_trajectorydes(:,1,:);
rpydes(:,:)  = track_trajectorydes(:,6,:);
veldes(:,:)  = track_trajectorydes(:,2,:);
avldes(:,:)  = track_trajectorydes(:,7,:);
accdes(:,:)  = track_trajectorydes(:,3,:);

plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [4 , 4, 1.5], points, points_N);
plot_graph_velocities(veldes, avldes, velvec, avlvec, tvec);



%% Plotting cumulative errors in x, y, z and yaw

error_pos = zeros(3, N);
error_rpy = zeros(3, N);

error_pos(:,:) = posdes(:,:) - posvec(:,:); 
error_rpy(:,:) = rpydes(:,:) - rpyvec(:,:); 

cum_error_pos =  zeros( 3, N);
cum_error_rpy =  zeros( 3, N);

for i=1:3
    cum_error_pos(i,:)  = cumsum(error_pos(i,:));
    cum_error_rpy(i,:)  = cumsum(error_rpy(i,:));
end


figure;
subplot(3,2,1);
a1 = plot(tvec,   cum_error_pos(1,:));
ylabel('x');
xlabel('time');
title('Cumulative error in x');
grid on;

subplot(3,2,2);
a1 = plot(tvec,   cum_error_rpy(1,:));
ylabel(' roll');
xlabel('time');
title('Cumulative error in roll');
grid on;


subplot(3,2,3);
a1 = plot(tvec,   cum_error_pos(2,:));
ylabel(' y');
xlabel('time');
title('Cumulative error in y');
grid on;

subplot(3,2,4);
a1 = plot(tvec,   cum_error_rpy(2,:));
ylabel(' pitch');
xlabel('time');
title('Cumulative error in pitch');
grid on;

subplot(3,2,5);
a1 = plot(tvec,   cum_error_pos(3,:));
ylabel('z');
xlabel('time');
title('Cumulative error in z');
grid on;


subplot(3,2,6);
a1 = plot(tvec,   cum_error_rpy(3,:));
ylabel(' yaw');
xlabel('time');
title('Cumulative error in yaw');
grid on;

suptitle('Cumulative errors for positions and Orientations');


%% Simulating
figure;
simulate_quadcopter(N, posvec, rpyvec, [-4,4], [-4,4], [-4,4], mfilename);