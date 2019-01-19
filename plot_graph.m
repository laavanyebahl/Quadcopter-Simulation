function plot_graph(posdes, rpydes, posvec, rpyvec, tvec, limits_pos_2d, limits_ang_2d, limits_pos_3d, points, points_N)

figure;
subplot(3,2,1);
plot(tvec, posdes(1,:) - posvec(1,:) );
for i=1:size(points,2)
    hold on;
    plot(points(i), posdes(1, points_N(i)) - posvec(1,  points_N(i)), '-og' )
end
hold on;
title('Error in x  ');
ylabel('x');
grid on;
lim = limits_pos_2d(1);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,2);
plot(tvec, rpydes(1,:) - rpyvec(1,:) );
for i=1:size(points,2)
    hold on;
    plot(points(i),rpydes(1, points_N(i)) - rpyvec(1, points_N(i)), '-og' )
end
hold on;
title('Error in roll ');
ylabel('roll');
grid on;
lim = limits_ang_2d(1);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,3);
plot(tvec, posdes(2,:) - posvec(2,:) );
for i=1:size(points,2)
    hold on;
    plot(points(i), posdes(2,  points_N(i)) - posvec(2,  points_N(i)), '-og' )
end
title('Error in y ');
ylabel('y');
grid on;
lim = limits_pos_2d(2);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,4);
plot(tvec, rpydes(2,:) - rpyvec(2,:) );
for i=1:size(points,2)
    hold on;
    plot(points(i),rpydes(2, points_N(i)) - rpyvec(2, points_N(i)), '-og' )
end
title('Error in pitch ');
ylabel('pitch');
grid on;
lim = limits_ang_2d(2);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,5);
plot(tvec, posdes(3,:) - posvec(3,:) );
for i=1:size(points,2)
    hold on;
    plot(points(i),posdes(3, points_N(i)) - posvec(3, points_N(i)), '-og' )
end
title('Error in z ');
xlabel('time');
ylabel('z');
grid on;
lim = limits_pos_2d(3);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,6);
plot(tvec, rpydes(3,:) - rpyvec(3,:) );
for i=1:size(points,2)
    hold on;
    plot(points(i),rpydes(3, points_N(i)) - rpyvec(3, points_N(i)), '-og' )
end
title('Error in yaw ');
xlabel('time');
ylabel('yaw');
grid on;
lim = limits_ang_2d(3);
if lim~=0 
    ylim([-lim lim]);
end
suptitle('Errors for positions and Orientations: Desired - Actual ');


figure;
subplot(3,2,1);
a1 = plot(tvec,   posdes(1,:));
for i=1:size(points,2)
    hold on;
    plot(points(i), posdes(1, points_N(i)), '-og' );
end
hold on;
a2 = plot(tvec,   posvec(1,:));
title('Change in X ');
ylabel('x');
grid on;
lim = limits_pos_2d(1);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,2);
a1 = plot(tvec,   rpydes(1,:));
for i=1:size(points,2)
    hold on;
    plot(points(i), rpydes(1, points_N(i)), '-og' )
end
hold on;
a2 = plot(tvec,   rpyvec(1,:));
title('Change in roll ');
ylabel('roll');
grid on;
lim = limits_ang_2d(1);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,3);
a1 = plot(tvec,   posdes(2,:));
for i=1:size(points,2)
    hold on;
    plot(points(i), posdes(2, points_N(i)), '-og' )
end
hold on;
a2 = plot(tvec,   posvec(2,:));
title('Change in Y :  ');
ylabel('y');
grid on;
lim = limits_pos_2d(2);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,4);
a1 = plot(tvec,   rpydes(2,:));
for i=1:size(points,2)
    hold on;
    plot(points(i), rpydes(2, points_N(i)), '-og' )
end
hold on;
a2 = plot(tvec,   rpyvec(2,:));
title('Change in pitch');
ylabel('pitch');
grid on;
lim = limits_ang_2d(2);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,5);
a1 = plot(tvec,   posdes(3,:));
for i=1:size(points,2)
    hold on;
    plot(points(i), posdes(3, points_N(i)), '-og' )
end
hold on;
a2 = plot(tvec,   posvec(3,:));
title('Change in Z ');
xlabel('time');
ylabel('z');
grid on;
lim = limits_pos_2d(3);
if lim~=0 
    ylim([-lim lim]);
end

subplot(3,2,6);
a1 = plot(tvec,   rpydes(3,:));
for i=1:size(points,2)
    hold on;
    plot(points(i), rpydes(3, points_N(i)), '-og' )
end
hold on;
a2 = plot(tvec,   rpyvec(3,:));
title('Change in yaw ');
xlabel('time');
ylabel('yaw');
grid on;
lim = limits_ang_2d(3);
if lim~=0 
    ylim([-lim lim]);
end
suptitle('Change in position & Orientation: Desired(blue),  Actual(orange)');


figure;
grid on;
p1 = plot3( posdes(1,:),  posdes(2,:),  posdes(3,:), 'm', 'LineWidth',6);
p1.Color(4) = 0.3;
hold on;
grid on;
p2 = plot3(  posvec(1,:),  posvec(2,:),  posvec(3,:), 'k', 'LineWidth',3);
p2.Color(4) = 1;
title('XYZ Path in 3D: Desired(pink), Actual(black)');

lim = limits_pos_3d(1);
if lim~=0 
    xlim([-lim lim]);
end
lim = limits_pos_3d(2);
if lim~=0 
    ylim([-lim lim]);
end
lim = limits_pos_3d(3);
if lim~=0 
    zlim([-lim lim]);
end

xlabel('x');
ylabel('y');
zlabel('z');
grid on;
hold off;

end