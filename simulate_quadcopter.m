function simulate_quadcopter(N, posvec, rpyvec, lim_x, lim_y, lim_z, name)

    filename = ['./question_videos/', name , '.avi'];
%     maxpos = max( 1, max(posvec(:)) ) +0.5;
%     minpos = min( -1, min(posvec(:)) )  -0.5;
%     
%     maxpos = 1.5;
%     minpos = -1.5;
%     
%     disp(maxpos);
%     disp(minpos);
    
    ax = axes('XLim',[lim_x(1) lim_x(2)],'YLim',[lim_y(1) lim_y(2)],'ZLim',[lim_z(1) lim_z(2)]);
    view(3)
    grid on

    [x,y,z] = cylinder([.05 .05]);
    h(1) = surface(z-0.5,x,y,'FaceColor','blue');
    h(2) = surface(y,z-0.5,x,'FaceColor','magenta');

    t = hgtransform('Parent',ax);
    set(h,'Parent',t);

    drawnow

    v = VideoWriter(filename);
    v.FrameRate = 15;
    open(v);

    for kk = 1:10:N
        % Translation matrixes
        Txyz = makehgtform('translate', posvec(:, kk) );
        % Rotation matrixes
        Rz = makehgtform('zrotate',rpyvec(3, kk));
        Ry = makehgtform('yrotate',rpyvec(2, kk));
        Rx = makehgtform('xrotate',rpyvec(1, kk));
        set(t,'Matrix',Txyz*Rz*Ry*Rx);
        pause(0.1)
        drawnow
        writeVideo(v,getframe(gcf));
    end
    
    close(v);
    
end