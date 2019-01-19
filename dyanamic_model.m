function [ pos_act, vel_act, acc_act, rpy_act, avl_act ] = dyanamic_model( rpmvec, posvec, velvec, rpyvec, avlvec, dt, plant_params)

c_T = plant_params.thrust_coefficient;
c_Q = plant_params.moment_scale*c_T ;
d = plant_params.arm_length;
tspan = [0, dt];
m = plant_params.mass;
inertia_matrix = plant_params.inertia;

mat = [ c_T c_T c_T c_T ;
        0  d*c_T  0 -d*c_T ;
        -d*c_T  0  d*c_T  0;
        -c_Q  c_Q  -c_Q  c_Q
        ];
    
forces_torques_act = mat*(rpmvec.^2);
% disp('forces_torques_act')
% disp(forces_torques_act)

f_act = forces_torques_act(1,:);
torque_act = forces_torques_act(2:4,:);
% disp(torque_act)

% disp('rpyvec')
% disp(rpyvec)

% disp('rpyvec after removing imaginary')
% disp(rpyvec)

euler_angle_rot_matrix = eul2rotm(fliplr(rpyvec'));
% disp(euler_angle_rot_matrix)
 
g = plant_params.gravity;
  
g_vec  = [ 0 0 g]' ;

F_b = [ 0 0 f_act ]' ;

acc_act = (1/m) * euler_angle_rot_matrix * F_b - g_vec;
% disp('acc_act')
% disp(acc_act)

[t, pos_dot] = ode45(@(t, pos_dot) acc_act, tspan, velvec  );
vel_act = pos_dot(end, :)';
% disp('vel_act')
% disp(vel_act);

[t, pos] = ode45(@(t, pos) vel_act, tspan, posvec  );
pos_act = pos(end, :)';
% disp('pos_act');
% disp(pos_act);

alpha = inertia_matrix\torque_act;
% disp('alpha');
% disp(alpha);

[t, rpy_dot] = ode45(@(t, rpy_dot) alpha, tspan, avlvec  );
avl_act = rpy_dot(end, :)';
% disp('avl_act');
% disp(avl_act);

[t, rpy] = ode45(@(t, rpy) avl_act, tspan, rpyvec  );
rpy_act = rpy(end, :)';
% disp('rpy_act');
% disp(rpy_act);

end