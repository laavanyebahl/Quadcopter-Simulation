close all;
clear;
clc;
addpath('..');

%% Sample input values
track_trajectory_time = 5;
take_off_height = 1;
timestep = 0.01;

%% Generate test trajectory for track_trajectory state
trajectory_N = (track_trajectory_time/timestep)+1 ;
trajectorydes_test = zeros(3, 8, trajectory_N);
trajectorydes_test(1, 1, 1:150 ) = linspace(0, 0.5, 150);
trajectorydes_test(1, 1, 151:end ) = 0.5;
trajectorydes_test(2, 1, 201:400 ) = linspace(0, 0.4, 200);
trajectorydes_test(2, 1, 401:end ) = 0.4;
trajectorydes_test(3, 1, :) = take_off_height;

points_N = [1, 151, 201, 401, trajectory_N];
points = timestep*points_N;

%% Start the state machine
[track_trajectorydes, track_trajectoryact, total_time, trajectorydes_total, trajectoryact_total] = state_machine(trajectorydes_test, timestep, track_trajectory_time, take_off_height);

%% Plotting onle track_trajectory path
tvec=0:timestep:track_trajectory_time;
N = numel(tvec);

posdes = zeros(3, N);
rpydes = zeros(3, N);
posvec = zeros(3, N);
rpyvec = zeros(3, N);

posvec(:,:)  = track_trajectoryact(:,1,:);
rpyvec(:,:)  = track_trajectoryact(:,6,:);
posdes(:,:)  = track_trajectorydes(:,1,:);
rpydes(:,:)  = track_trajectorydes(:,6,:);

plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [2 , 2, 2], points, points_N);




%% Plotting combined total path
tvec=2*timestep:timestep:total_time;
N = numel(tvec);

posdes = zeros(3, N);
rpydes = zeros(3, N);
posvec = zeros(3, N);
rpyvec = zeros(3, N);

posvec(:,:)  = trajectoryact_total(:,1,:);
rpyvec(:,:)  = trajectoryact_total(:,6,:);
posdes(:,:)  = trajectorydes_total(:,1,:);
rpydes(:,:)  = trajectorydes_total(:,6,:);


%% Simulating
figure;
simulate_quadcopter(N, posvec, rpyvec, [-2,2], [-2,2], [-1,2], mfilename);

%% Plotting
plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [2 , 2, 2], [], []);

