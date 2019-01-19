close all;
clear;
clc;
addpath('..');


%% Initialize everything

timestep = 0.01;
track_trajectory_time = 5;
take_off_height = 0.05;

desired_track_trajectory_height = 0.1;

trajectory_N = (track_trajectory_time/timestep)+1 ;
trajectorydes = zeros(3, 8, trajectory_N);

[state, A, B, C, Q, R, plant_params, trajectorydes, rpms] = LQRinitialize(trajectory_N);
% trajectorydes format - (posdes, veldes, accdes, jerkdes, snapdes, rpydes, rpy_d_des, rpy_dd_des)
% trajectoryact format - (posact, velact, accact, jerkact, snapact, rpyact, rpy_d_act, rpy_dd_act)

%% Create Trajectory

trajectorydes(3, 1, 1:2) = take_off_height;
trajectorydes(3, 1, 3:end) = desired_track_trajectory_height;

state.z(1,1) = take_off_height;

points_N = [];
points = [];

%% Run LQR controller
[state] = LQRcontroller(A, B, C, Q, R, plant_params, trajectory_N, state, rpms, timestep, trajectorydes );

state_arr = struct2cell(state);
state_arr = cell2mat(state_arr);

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

posvec(:,:) = state_arr(1:3, :);
rpyvec(:,:) = state_arr(4:6, :);
velvec(:,:)  = state_arr(7:9,:);
avlvec(:,:)  = state_arr(10:12,:);

posdes(:,:)  = trajectorydes(:,1,:);
rpydes(:,:)  = trajectorydes(:,6,:);
veldes(:,:)  = trajectorydes(:,2,:);
avldes(:,:)  = trajectorydes(:,7,:);

%% Code for step info
disp('step info for positions');
position_stepinfo = stepinfo(posvec(3,:), tvec, desired_track_trajectory_height);
disp(position_stepinfo);

disp('step info for velocities');
velocity_stepinfo = stepinfo(velvec(3,:), tvec, 0);
disp(velocity_stepinfo);

%% Simulating
figure;
simulate_quadcopter(N, posvec, rpyvec, [-1,1], [-1,1], [-1,2], mfilename);

%% Plotting
plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [0.2 , 0.2, 0.2], [], []);
plot_graph_velocities(veldes, avldes, velvec, avlvec, tvec);

