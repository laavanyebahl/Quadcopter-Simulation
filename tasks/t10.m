close all;
clear;
clc;
addpath('..');

%% Initialize everything
timestep = 0.01;
track_trajectory_time = 5;
take_off_height = 1;

%% Generate trajectory for track_trajectory state from trajdes
trajectory_N = (track_trajectory_time/timestep)+1 ;
trajectorydes_test = zeros(3, 8, trajectory_N);


%% Generate test trajectory for track_trajectory state
trajectory_N = (track_trajectory_time/timestep)+1 ;
trajectorydes_test = zeros(3, 8, trajectory_N);
trajectorydes_test(1, 1, 1:150 ) = 0.5;
trajectorydes_test(1, 1, 151:end ) = 0.5;
trajectorydes_test(2, 1, 201:end ) = 0.4;
trajectorydes_test(3, 1, :) = take_off_height;

points_N = [1, 151, 201, 401, trajectory_N];
points = timestep*points_N;

% NOW CHANGE THE GAINS VALUES
% IN THE FILE PDINITIALIZE.M

% ctrl.Kp = [17 17 20];
% ctrl.Kv = 0.6*[6.6 6.6 9];
% ctrl.Kr = [190 198 80];
% ctrl.Kw = [30 30 17.88];


%% Start the state machine
[track_trajectorydes, track_trajectoryact, total_time, trajectorydes_total, trajectoryact_total] = state_machine(trajectorydes_test, timestep, track_trajectory_time, take_off_height);

%% Plotting only track_trajectory path
tvec=2*timestep:timestep:total_time;
N = numel(tvec);

disp(total_time);
posdes = zeros(3, N);
rpydes = zeros(3, N);
veldes = zeros(3, N);
avldes = zeros(3, N);

posvec = zeros(3, N);
rpyvec = zeros(3, N);
velvec = zeros(3, N);
avlvec = zeros(3, N);

posvec(:,:)  = trajectoryact_total(:,1,:);
rpyvec(:,:)  = trajectoryact_total(:,6,:);
velvec(:,:)  = trajectoryact_total(:,2,:);
avlvec(:,:)  = trajectoryact_total(:,7,:);

posdes(:,:)  = trajectorydes_total(:,1,:);
rpydes(:,:)  = trajectorydes_total(:,6,:);
veldes(:,:)  = trajectorydes_total(:,2,:);
avldes(:,:)  = trajectorydes_total(:,7,:);

% Simulating
figure;
simulate_quadcopter(N, posvec, rpyvec, [-1,1], [-1,1], [-1,2], mfilename);

%% Plotting
plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [1.2 , 1.2, 1.2], [], []);
plot_graph_velocities(veldes, avldes, velvec, avlvec, tvec);

