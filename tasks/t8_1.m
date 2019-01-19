close all;
clear;
clc;
addpath('..');

%% Generate elliptical minimum energy trajectory 
n = 7;
r = 4;

% time cycle is 3 for 1 quarter, 12 for 1 cycle, 48 in total
timestep = 0.01;
dt= 2;
num_waypoints = 4;

N = (num_waypoints*dt)/timestep;
disp(N);
trajdes = zeros(r+1,4, (num_waypoints*dt)/timestep );

time = 0;

x = [0, 2, 0, -2, 0 ];
y = [0, 1, 2, 1, 0 ];

% constraints format = [ value, time, order]
% inequality_constraint format = [ less than this term, time, order]

v_x = zeros(1, num_waypoints+1);
v_y = zeros(1, num_waypoints+1);

inequality_constraint_x = [];
inequality_constraint_y = [];

points = [timestep, dt, (2*dt), (3*dt)];
points_N = (1/timestep)*points;

% 1st waypoint
disp(time);
constraints_x = [ x(1,1), 0, 0; x(1,2), dt, 0; v_x(1,1), 0, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
constraints_y = [ y(1,1), 0, 0; y(1,2), dt, 0; v_y(1,1), 0, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
% inequality_constraint_x = [];
% inequality_constraint_y = [1, dt, 1];
[ trajdes(:,1,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_x(1,2) ] = generate_trajectory(n, r, constraints_x, inequality_constraint_x, timestep);
[ trajdes(:,2,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_y(1,2) ]= generate_trajectory(n, r, constraints_y, inequality_constraint_y, timestep);
time = time + dt;   


% 2nd waypoint
disp(time);
constraints_x = [ x(1,2), 0, 0; x(1,3), dt, 0; v_x(1,2), 0, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
constraints_y = [ y(1,2), 0, 0; y(1,3), dt, 0; v_y(1,2), 0, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3];
% inequality_constraint_x = [0, dt, 1];
% inequality_constraint_y = [];
[ trajdes(:,1,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_x(1,3) ] = generate_trajectory(n, r, constraints_x, inequality_constraint_x, timestep);
[ trajdes(:,2,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_y(1,3) ]= generate_trajectory(n, r, constraints_y, inequality_constraint_y, timestep);
time = time + dt;   

% 3rd waypoint
disp(time);
constraints_x = [ x(1,3), 0, 0; x(1,4), dt, 0; v_x(1,3), 0, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
constraints_y = [ y(1,3), 0, 0; y(1,4), dt, 0; v_y(1,3), 0, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
% inequality_constraint_x = [];
% inequality_constraint_y = [0, dt, 1];
[ trajdes(:,1,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_x(1,4) ] = generate_trajectory(n, r, constraints_x, inequality_constraint_x, timestep);
[ trajdes(:,2,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_y(1,4) ]= generate_trajectory(n, r, constraints_y, inequality_constraint_y, timestep);
time = time + dt;   

% 4th waypoint
disp(time);
constraints_x = [ x(1,4), 0, 0; x(1,5), dt, 0; v_x(1,4), 0, 1; 1, dt, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
constraints_y = [ y(1,4), 0, 0; y(1,5), dt, 0; v_y(1,4), 0, 1; 0, dt, 1; 0, 0, 2; 0, dt, 2; 0, 0, 3; 0, dt, 3]; 
% inequality_constraint_x = [];
% inequality_constraint_y = [];
[ trajdes(:,1,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_x(1,5) ] = generate_trajectory(n, r, constraints_x, inequality_constraint_x, timestep);
[ trajdes(:,2,(time*(1/timestep))+1:(time+dt)*(1/timestep)) , v_y(1,5) ]= generate_trajectory(n, r, constraints_y, inequality_constraint_y, timestep);
time = time + dt;   


%% Input values
track_trajectory_time = dt*num_waypoints;
take_off_height = 1;

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

% plot_graph_velocities(veldes, avldes, velvec, avlvec, tvec);

%% Simulating
figure;
simulate_quadcopter(N, posvec, rpyvec, [-4,4], [-4,4], [-1,2], mfilename);
