function [state] = LQRcontroller(A, B, C, Q, R, plant_params, N, state, rpmvec, dt, trajectorydes )

    m = plant_params.mass;
    g = plant_params.gravity;
    y_des = zeros(4,1);
    y_act = zeros(4,1);
    v = zeros(4,1);
    u = zeros(4,1);
    u_feedback = zeros(4,1);
    
    syms yaw yaw_d;

    %% Run simulation
    for kk = 2:N
     % Get position and yaw reference and derivatives
        
         %% Run plant model
        y_des(1:3,1) = trajectorydes(1:3, 1, kk);
        y_des(4,1) = trajectorydes(1, 4, kk);
        A = A_equation(y_des(4,1), 0 );
        K = lqr(A, B, Q, R);
        v =  - inv((C*(inv(A - (B*K))))*B)*y_des;
        u = v - u_feedback;
        
        % ADD MG VERY IMPORTANT     
        u(1,1) = u(1,1) + m*g; 
        
        %% Run controllers
    %     disp('motor controller: ');
        rpmvec(:, kk) = motor_controller( u(1,1), u(2:4,1) , plant_params, rpmvec(:, kk-1), dt);
        
        %% Run quadrotor dyanamic model controller
    %     disp('dyanamic model: ');
        [ posvec, velvec, accvec, rpyvec, avlvec ] = dyanamic_model( rpmvec(:, kk),  [state.x(1,kk-1), state.y(1,kk-1), state.z(1,kk-1)]', [state.x_d(1,kk-1), state.y_d(1,kk-1), state.z_d(1,kk-1)]',  [state.roll(1,kk-1), state.pitch(1,kk-1), state.yaw(1,kk-1)]',  [state.roll_d(1,kk-1), state.pitch_d(1,kk-1), state.yaw_d(1,kk-1)]', dt, plant_params);
        state.x(1,kk)= posvec(1,1);
        state.y(1,kk)= posvec(2,1);
        state.z(1,kk)= posvec(3,1);
        state.roll(1,kk)= rpyvec(1,1);
        state.pitch(1,kk)= rpyvec(2,1);
        state.yaw(1,kk)= rpyvec(3,1);
        state.x_d(1,kk)= velvec(1,1);
        state.y_d(1,kk)= velvec(2,1);
        state.z_d(1,kk)= velvec(3,1);
        state.roll_d(1,kk)= avlvec(1,1);
        state.pitch_d(1,kk)= avlvec(2,1);
        state.yaw_d(1,kk) = avlvec(3,1); 
        
        state_arr = struct2cell(state);
        state_arr = cell2mat(state_arr);

        state_kk = state_arr(:, kk);
        
        u_feedback = K*state_kk;

        y_act(:, kk) = C*state_kk;
               
    end

end