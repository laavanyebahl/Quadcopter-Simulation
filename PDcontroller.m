function [ trajectorydes, trajectoryact, rpms_last ] = PDcontroller(plant_params, ctrl, N, dt, trajectorydes, trajectoryact, rpms )
    
    yaw_yawd_yawdd_des = zeros(3, N);

    %% Run simulation
    for kk = 2:N

        yaw_yawd_yawdd_des(:,kk) = [trajectorydes(3, 6, kk); trajectorydes(3, 7, kk); trajectorydes(3, 8, kk) ]; 

        %% Run controllers
    %     disp('position controller: ');
        [errpos_dd, f_des]  = position_controller(trajectoryact(:, 1, kk-1), trajectoryact(:, 2, kk-1), trajectorydes(:, 1, kk), trajectorydes(:, 2, kk), trajectorydes(:, 3, kk), plant_params, ctrl) ;

    %     disp('attitude controller: ');
        [torques, trajectorydes(:, 6, kk)]  = attitude_controller(errpos_dd, trajectoryact(:, 6, kk-1), trajectoryact(:, 7, kk-1), yaw_yawd_yawdd_des(:, kk),trajectorydes(:, 3, kk), trajectorydes(:, 4, kk), trajectorydes(:, 5, kk), plant_params,  ctrl) ;

    %     disp('motor controller: ');
         rpms(:, kk) = motor_controller( f_des, torques , plant_params, rpms(:, kk-1), dt);

         %% Run quadrotor dyanamic model controller
    %     disp('dyanamic model: ');
        [trajectoryact(:, 1, kk), trajectoryact(:, 2, kk), trajectoryact(:, 3, kk),trajectoryact(:, 6, kk), trajectoryact(:, 7, kk)] = dyanamic_model( rpms(:, kk), trajectoryact(:, 1, kk-1), trajectoryact(:, 2, kk-1), trajectoryact(:, 6, kk-1),trajectoryact(:, 7, kk-1), dt, plant_params);

    end
rpms_last = rpms(:, end);
end