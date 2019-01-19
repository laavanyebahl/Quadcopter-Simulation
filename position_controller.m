function [errpos_dot_dot, f_des] = position_controller(posvec, velvec, posdes, veldes, accdes, plant_params, ctrl)
 
Kp = ctrl.Kp' ;
Kd = ctrl.Kv' ;
m = plant_params.mass;
g = plant_params.gravity;

g_vect = [ 0;
      0;
      g ];

b3 = [ 0 ;
       0 ;
       1  ];
   
errpos = posvec - posdes ;
errpos_dot = velvec - veldes;
errpos_dot_dot = -Kp.*errpos -Kd.*errpos_dot;

f_des = m*b3'*(g_vect + errpos_dot_dot  + accdes);

% if f_des<0
%     f_des = 0;
% end

% disp('f_des');
% disp(f_des);


end