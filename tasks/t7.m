close all;
clear;
clc;
addpath('..');

%% Initialize everything
n = 7;
r = 4;

% time cycle is 3 for 1 quarter, 12 for 1 cycle, 48 in total
timestep = 0.01;

% CASE 1 : 
dt= 4 ;      % Default motor constant 

% CASE 2 : 
% dt= 2.5 ;      % Default motor constant 

% CASE 3 : 
% dt= 1.6 ;      % Default motor constant 

% CASE 4 : 
% dt= 1.6 ;   % Increased motor constant and motor rpm, changed in PDinitialize

num_waypoints = 1;
acceleration_bound = 3;

N = (num_waypoints*dt)/timestep;
N = int16(N);
disp(N);
trajdes_z = zeros(r+1,4, num_waypoints*N );

constraints_z = [ 1, 0, 0; 10, dt, 0; 0, 0, 1; 0, dt, 1; 0, 0, 3; 0, dt, 3]; 
inequality_constraint_z = [ 3, 0, 2; acceleration_bound, dt, 2];
[ trajdes_z(:,3, 1:end) , v_z ] = generate_trajectory(n, r, constraints_z, inequality_constraint_z, timestep);

%% Input values
track_trajectory_time = dt;
take_off_height = 1;

%% Generate trajectory for track_trajectory state from trajdes
trajectory_N = (track_trajectory_time/timestep)+1 ;
trajectory_N = int16(trajectory_N);

trajectorydes_test = zeros(3, 8, trajectory_N);

for i=1:3
    trajectorydes_test(3, i, 1:end-1) = trajdes_z(i,3,:);
    trajectorydes_test(3, i, end) = trajdes_z(i,3,end);
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

%% Simulating
% figure;
% simulate_quadcopter(N, posvec, rpyvec, [-1,1], [-1,1], [-1,12], mfilename);

%% Plotting
plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [2 , 2, 12], [], []);
plot_graph_velocities(veldes, avldes, velvec, avlvec, tvec);

% Plot acceleration
figure;
a1 = plot(tvec,   accdes(3,:));
hold on;
a2 = plot(tvec,   accvec(3,:));
title('Change in linear acceleration in z: Desired(blue),  Actual(orange) ');
xlabel('time');
ylabel('acc in z');
grid on;