function rpms_act = motor_controller(f, taus, plant_params, rpmvec, dt)

c_T = plant_params.thrust_coefficient;
c_Q = plant_params.moment_scale*c_T ;
d = plant_params.arm_length;
Km = plant_params.motor_constant ;
tspan = [0, dt];

mat_inv = [ 1/(4*c_T),            0, -1/(2*c_T*d), -1/(4*c_Q);
 1/(4*c_T),  1/(2*c_T*d),            0,  1/(4*c_Q);
 1/(4*c_T),            0,  1/(2*c_T*d), -1/(4*c_Q);
 1/(4*c_T), -1/(2*c_T*d),            0,  1/(4*c_Q)] ;

forces = [ f ;
           taus
         ];
     
rpm_sqaures = mat_inv * forces;
rpmdes = sqrt(rpm_sqaures);
% disp('rpmdes');
% disp(rpmdes);

[t, wi] = ode45(@(t, wi) Km*(rpmdes - wi), tspan, rpmvec  );
rpmtmp = wi(end, :)';
% disp('rpmtmp');
% disp(rpmtmp);

% Saturate RPM
rpmtmp(rpmtmp > plant_params.rpm_max) = plant_params.rpm_max;
rpmtmp(rpmtmp < plant_params.rpm_min) = plant_params.rpm_min;

rpms_act = rpmtmp;
rpms_act = real(rpms_act);

% disp('rpms_act');
% disp(rpms_act);

end