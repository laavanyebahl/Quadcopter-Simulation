close all;
clear;
clc;
addpath('..');

%% Initialize everything
timestep = 0.01;
track_trajectory_time = 5;
take_off_height = 0.05;

%% Generate trajectory for track_trajectory state from trajdes

desired_track_trajectory_height = 0.1;

trajectory_N = (track_trajectory_time/timestep)+1 ;
trajectorydes_test = zeros(3, 8, trajectory_N);

% Case 1 :
trajectorydes_test(3, 1, 1:5) = take_off_height;
trajectorydes_test(3, 1, 1:end) = desired_track_trajectory_height;

% Case 2 (heading of 15 deg) :
trajectorydes_test(3, 1, :) = desired_track_trajectory_height;
trajectorydes_test(3, 6, :) = deg2rad(15);


%% Start the state machine
[track_trajectorydes, track_trajectoryact, total_time, trajectorydes_total, trajectoryact_total] = state_machine(trajectorydes_test, timestep, track_trajectory_time, take_off_height);

%% Plotting only track_trajectory path
tvec=0:timestep:track_trajectory_time;
N = numel(tvec);

posdes = zeros(3, N);
rpydes = zeros(3, N);
veldes = zeros(3, N);
avldes = zeros(3, N);

posvec = zeros(3, N);
rpyvec = zeros(3, N);
velvec = zeros(3, N);
avlvec = zeros(3, N);

posvec(:,:)  = track_trajectoryact(:,1,:);
rpyvec(:,:)  = track_trajectoryact(:,6,:);
velvec(:,:)  = track_trajectoryact(:,2,:);
avlvec(:,:)  = track_trajectoryact(:,7,:);

posdes(:,:)  = track_trajectorydes(:,1,:);
rpydes(:,:)  = track_trajectorydes(:,6,:);
veldes(:,:)  = track_trajectorydes(:,2,:);
avldes(:,:)  = track_trajectorydes(:,7,:);

%% Code for step info
disp('step info for positions');
position_stepinfo = stepinfo(posvec(3,:), tvec, desired_track_trajectory_height);
disp(position_stepinfo);

disp('step info for velocities');
velocity_stepinfo = stepinfo(velvec(3,:), tvec, 0);
disp(velocity_stepinfo);

%% Simulating
% figure;
% simulate_quadcopter(N, posvec, rpyvec, [-1,1], [-1,1], [-1,2], mfilename);

%% Plotting
plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [0.2 , 0.2, 0.2], [], []);
plot_graph_velocities(veldes, avldes, velvec, avlvec, tvec);

