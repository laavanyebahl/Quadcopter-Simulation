function [taus, rpydes] = attitude_controller( errpos_dd_val, rpyvec_val, avlvec_val, yaw_yawd_yawdd_des, acc_des_val, jerkdes_val, snapdes_val,  plant_params,  ctrl )
    
inertia_matrix = plant_params.inertia;
Kr = ctrl.Kr';
Kw = ctrl.Kw';
   

%% Calculate value
rollpitchdes_val = rollpitchdes_equation( 0, yaw_yawd_yawdd_des(1,1),  errpos_dd_val(1,1), errpos_dd_val(2,1), acc_des_val(1,1), acc_des_val(2,1)  );
% disp('rollpitchdes_val');
% disp(rollpitchdes_val);

rollpitch_d_des_val =  rollpitch_d_des_equation( 0, yaw_yawd_yawdd_des(1,1), errpos_dd_val(1,1), errpos_dd_val(2,1), acc_des_val(1,1), acc_des_val(2,1), yaw_yawd_yawdd_des(2,1), 0, 0, jerkdes_val(1,1), jerkdes_val(2,1)  );
% disp('rollpitch_d_des_val');
% disp(rollpitch_d_des_val);

rollpitch_dd_des_val = rollpitch_dd_des_equation( 0, yaw_yawd_yawdd_des(1,1), errpos_dd_val(1,1), errpos_dd_val(2,1), acc_des_val(1,1), acc_des_val(2,1), yaw_yawd_yawdd_des(2,1), 0, 0, jerkdes_val(1,1), jerkdes_val(2,1), yaw_yawd_yawdd_des(3,1), 0, 0, snapdes_val(1,1), snapdes_val(2,1)   );
% disp('rollpitch_dd_des_val');
% disp(rollpitch_dd_des_val);


%% 
rpydes = [ rollpitchdes_val;
           yaw_yawd_yawdd_des(1,1)
          ];

% disp('rpydes');
% disp(rpydes);
      
rpydes_d = [ rollpitch_d_des_val;
             yaw_yawd_yawdd_des(2,1)
          ];
% disp('rpydes_d');
% disp(rpydes_d);
      
rpydes_dd = [ rollpitch_dd_des_val;
           yaw_yawd_yawdd_des(3,1)
          ];
% disp('rpydes_dd');
% disp(rpydes_dd);

taus = inertia_matrix * ( -Kr.*( rpyvec_val - rpydes) -Kw.*(avlvec_val - rpydes_d) + rpydes_dd );
% disp('taus');
% disp(taus);
end