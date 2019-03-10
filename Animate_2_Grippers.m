
function [] = Animate_2_Grippers(Tout,Xout,Gripper,Object,speed)
    
 
    figure('units','normalized','outerposition',[0 0 1 1])
    axis equal
    axis manual

    xlim([-.4, .8]);
    ylim([-1.0, .2]);
    x_min = -0.22;
    
    xlabel('x');
    ylabel('y');
    
    hold on
    %grid on    
    
    Dot = 0;
    
    Xeq_left = Xout(1,1:2);
    Xeq_right = Xout(1,7:8);
    
    hleft = PlotGripper(Xout(1,1:3),Xeq_left,Gripper,true);
    hright = PlotGripper(Xout(1,7:9),Xeq_right,Gripper,false);
    hobject = PlotObject(Xout(1,13:14),Object,Dot);

    pause(1);
    i_old = 1;
    tic
    while toc < Tout(end)/speed
        temp = find(Tout > toc*speed);
        i = temp(1);
        delete(hleft);
        delete(hright);
        delete(hobject);
        Dot = Dot + (Tout(i)-Tout(i_old))*Xout(i,17);
        hleft = PlotGripper(Xout(i,1:3),Xeq_left,Gripper,true);
        hright = PlotGripper(Xout(i,7:9),Xeq_right,Gripper,false);
        hobject = PlotObject(Xout(i,13:14),Object,Dot);
        plot([Xout(i_old,13),Xout(i,13)], [Xout(i_old,14),Xout(i,14)],'k','LineWidth',1);

%         uistack(h1(1),'bottom');
        uistack(hobject(1),'bottom');
        i_old = i;
        pause(.01)

    end
    
    

end

