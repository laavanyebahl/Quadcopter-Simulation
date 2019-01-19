function attitude_controller_equation_generator( plant_params )
    
g = plant_params.gravity;

%% Calculate equations
syms ex_dd(t) ey_dd(t) x_dd_des(t) y_dd_des(t) yaw_des(t)
rollpitchdes = (1/g)*[ sin(yaw_des), -cos(yaw_des); cos(yaw_des), sin(yaw_des) ]* [ (ex_dd + x_dd_des) ; (ey_dd + y_dd_des) ] ;
rollpitchdes = simplify(rollpitchdes);
% disp('rollpitchdes');
% disp(rollpitchdes);

rollpitch_d_des = diff(rollpitchdes, t);
% disp('rollpitch_d_des');
% disp(rollpitch_d_des);

rollpitch_dd_des = diff(rollpitch_d_des, t);
% disp('rollpitch_dd_des');
% disp(rollpitch_dd_des);

%%  Generate matlab function for attitude controller

syms ex_dd_var ey_dd_var x_dd_des_var y_dd_des_var yaw_des_var ;
syms diff_yaw_des_var diff_ex_dd_var diff_ey_dd_var diff_x_dd_des_var diff_y_dd_des_var  diff_diff_yaw_des_var diff_diff_ex_dd_var diff_diff_ey_dd_var diff_diff_x_dd_des_var diff_diff_y_dd_des_var

rollpitchdes_equation = subs( rollpitchdes, {yaw_des, ex_dd, ey_dd, x_dd_des, y_dd_des }, ...
    {yaw_des_var, ex_dd_var, ey_dd_var, x_dd_des_var, y_dd_des_var } );
disp('rollpitchdes_equation');
disp(rollpitchdes_equation);

rollpitch_d_des_equation = subs( rollpitch_d_des, { yaw_des,  ex_dd, ey_dd, x_dd_des, y_dd_des, diff(yaw_des, t), diff(ex_dd, t), diff(ey_dd, t), diff(x_dd_des, t), diff(y_dd_des, t) }, ...
    {yaw_des_var, ex_dd_var, ey_dd_var, x_dd_des_var, y_dd_des_var, diff_yaw_des_var, diff_ex_dd_var, diff_ey_dd_var, diff_x_dd_des_var, diff_y_dd_des_var }  );
disp('rollpitch_d_des_equation');
disp(rollpitch_d_des_equation);

rollpitch_dd_des_equation = subs( rollpitch_dd_des, { yaw_des,  ex_dd, ey_dd, x_dd_des, y_dd_des, diff(yaw_des, t), diff(ex_dd, t), diff(ey_dd, t), diff(x_dd_des, t), diff(y_dd_des, t),  diff(yaw_des, t, t), diff(ex_dd, t, t), diff(ey_dd, t, t), diff(x_dd_des, t, t), diff(y_dd_des, t, t) }, ...
    { yaw_des_var, ex_dd_var, ey_dd_var, x_dd_des_var, y_dd_des_var,  diff_yaw_des_var, diff_ex_dd_var, diff_ey_dd_var, diff_x_dd_des_var, diff_y_dd_des_var, diff_diff_yaw_des_var, diff_diff_ex_dd_var, diff_diff_ey_dd_var, diff_diff_x_dd_des_var, diff_diff_y_dd_des_var }  );
disp('rollpitch_dd_des_equation');
disp(rollpitch_dd_des_equation);

matlabFunction(rollpitchdes_equation, 'File', 'rollpitchdes_equation', 'Vars', ...
      [t yaw_des_var ex_dd_var ey_dd_var x_dd_des_var y_dd_des_var ] );

matlabFunction(rollpitch_d_des_equation, 'File', 'rollpitch_d_des_equation', 'Vars', ...
      [t  yaw_des_var ex_dd_var ey_dd_var x_dd_des_var y_dd_des_var diff_yaw_des_var diff_ex_dd_var diff_ey_dd_var diff_x_dd_des_var diff_y_dd_des_var  ] );
  
matlabFunction(rollpitch_dd_des_equation, 'File', 'rollpitch_dd_des_equation', 'Vars', ...
      [t yaw_des_var ex_dd_var ey_dd_var x_dd_des_var y_dd_des_var  diff_yaw_des_var diff_ex_dd_var diff_ey_dd_var diff_x_dd_des_var diff_y_dd_des_var diff_diff_yaw_des_var diff_diff_ex_dd_var diff_diff_ey_dd_var diff_diff_x_dd_des_var diff_diff_y_dd_des_var ]);

end