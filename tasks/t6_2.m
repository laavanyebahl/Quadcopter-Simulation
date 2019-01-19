close all;
clear;
clc;
addpath('..');

%% Initialize everything
time = 15;
timestep = 0.005;
tvec=0:timestep:time;
N = numel(tvec);
[state, A, B, C, Q, R, plant_params, trajectorydes, rpms] = LQRinitialize(N);
% trajectorydes format - (posdes, veldes, accdes, jerkdes, snapdes, rpydes, rpy_d_des, rpy_dd_des)
% trajectoryact format - (posact, velact, accact, jerkact, snapact, rpyact, rpy_d_act, rpy_dd_act)

%% Create Trajectory
% trajectorydes( 3, 1 , 1:700) =  linspace(0, 0.5, 700);
trajectorydes( 3, 1 , 3:800) =  0.5;
trajectorydes( 3, 1 , 801:3001) =  0.5;
trajectorydes( 1, 1 , 1001:1500) =  0.1;
trajectorydes( 1, 1 , 1501:2000) =  0.2;
trajectorydes( 1, 1 , 2001:2500) =  0.3;
trajectorydes( 1, 1 , 2501:3001) =  0.4;

points_N = [3, 801, 1001, 1501, 2001, 2501];
points = timestep*points_N;

%% Run LQR controller
[state] = LQRcontroller(A, B, C, Q, R, plant_params, N, state, rpms, timestep, trajectorydes );

state_arr = struct2cell(state);
state_arr = cell2mat(state_arr);
posvec = state_arr(1:3, :);
rpyvec = state_arr(4:6, :);

posdes = trajectorydes(:,1,:);
rpydes = trajectorydes(:,6,:);

%% Simulating
figure;
simulate_quadcopter(N, posvec, rpyvec, [-2,2], [-2,2], [-1,2], mfilename);

%% Plotting
plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [2 , 2, 2], points, points_N);
