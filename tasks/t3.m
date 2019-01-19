close all;
clear;
clc;
addpath('..');

%% Initialize everything
time = 20;
timestep = 0.005;
tvec=0:timestep:time;
N = numel(tvec);
[plant_params, ctrl, trajectorydes, trajectoryact, rpms] = PDinitialize(N);
% trajectorydes format - (posdes, veldes, accdes, jerkdes, snapdes, rpydes, rpy_d_des, rpy_dd_des)
% trajectoryact format - (posact, velact, accact, jerkact, snapact, rpyact, rpy_d_act, rpy_dd_act)

%% Create Trajectory
trajectorydes( 3, 3 , 1:1000) =  0.04;
trajectorydes( 3, 3 , 1001:3000) =  -0.04;
trajectorydes( 3, 3 , 3001:4001) =  0.04;
for i=1:4000
    trajectorydes(3, 2, i+1) = trajectorydes(3, 2, i) + trajectorydes(3, 3, i+1)*(tvec(1,i+1)-tvec(1,i));
    trajectorydes(3, 1, i+1) = trajectorydes(3, 1, i) + (  ( trajectorydes(3, 2, i+1)^2 - trajectorydes(3, 2, i)^2 )/ (2*trajectorydes(3, 3,i+1))  );
end

points_N = [1, 1001, 3001];
points = timestep*points_N;


%% Run controller
[trajectorydes, trajectoryact] = PDcontroller(plant_params, ctrl, N, timestep, trajectorydes, trajectoryact, rpms );

posdes = zeros(3, N);
rpydes = zeros(3, N);
posvec = zeros(3, N);
rpyvec = zeros(3, N);

posvec(:,:)  = trajectoryact(:,1,:);
rpyvec(:,:)  = trajectoryact(:,6,:);

posdes(:,:)  = trajectorydes(:,1,:);
rpydes(:,:)  = trajectorydes(:,6,:);

%% Simulating
figure;
simulate_quadcopter(N, posvec, rpyvec, [-2,2], [-2,2], [-1,2], mfilename);

%% Plotting
plot_graph(posdes, rpydes, posvec, rpyvec, tvec, [0,0,0], [0,0,0], [2 , 2, 2], points, points_N);

