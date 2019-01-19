function plot_graph_velocities(veldes, avldes, velvec, avlvec, tvec)

figure;
subplot(3,2,1);
plot(tvec, veldes(1,:) - velvec(1,:) );
title('Error in x_d  ');
ylabel('x_d');
grid on;

subplot(3,2,2);
plot(tvec, avldes(1,:) - avlvec(1,:) );
title('Error in roll_d ');
ylabel('roll_d');
grid on;


subplot(3,2,3);
plot(tvec, veldes(2,:) - velvec(2,:) );
title('Error in y_d ');
ylabel('y_d');
grid on;

subplot(3,2,4);
plot(tvec, avldes(2,:) - avlvec(2,:) );
title('Error in pitch_d ');
xlabel('time');
ylabel('pitch_d');
grid on;

subplot(3,2,5);
plot(tvec, veldes(3,:) - velvec(3,:) );
title('Error in z_d ');
xlabel('time');
ylabel('z_d');
grid on;


subplot(3,2,6);
plot(tvec, avldes(3,:) - avlvec(3,:) );
title('Error in yaw_d ');
xlabel('time');
ylabel('yaw_d');
grid on;
suptitle('Errors for linear & angular velocities (Desired - Actual) ');


figure;
subplot(3,2,1);
a1 = plot(tvec,   veldes(1,:));
hold on;
a2 = plot(tvec,   velvec(1,:));
title('Change in X_d ');
ylabel('x_d');
grid on;

subplot(3,2,2);
a1 = plot(tvec,   avldes(1,:));
hold on;
a2 = plot(tvec,   avlvec(1,:));
title('Change in roll_d ');
ylabel('roll_d');
grid on;

subplot(3,2,3);
a1 = plot(tvec,   veldes(2,:));
hold on;
a2 = plot(tvec,   velvec(2,:));
title('Change in Y_d :  ');
ylabel('y_d');
grid on;


subplot(3,2,4);
a1 = plot(tvec,   avldes(2,:));
hold on;
a2 = plot(tvec,   avlvec(2,:));
title('Change in pitch_d');
ylabel('pitch_d');
grid on;


subplot(3,2,5);
a1 = plot(tvec,   veldes(3,:));
hold on;
a2 = plot(tvec,   velvec(3,:));
title('Change in Z_d ');
xlabel('time');
ylabel('z_d');
grid on;

subplot(3,2,6);
a1 = plot(tvec,   avldes(3,:));
hold on;
a2 = plot(tvec,   avlvec(3,:));
title('Change in yaw_d ');
xlabel('time');
ylabel('yaw_d');
grid on;
suptitle('Change in linear & angular velocities: Desired(blue),  Actual(orange)');


end