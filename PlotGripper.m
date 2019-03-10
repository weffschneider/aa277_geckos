function [handle] = PlotGripper(X,Xeq,Gripper, left)

    handle = zeros(10,1);
    
    t = .01;
    xG = X(1);
    yG = X(2);
    theta = X(3);
    xeq = Xeq(1);
    yeq = Xeq(2);
    w = Gripper.w;
    L = Gripper.span;
    
    if left
        xGrip = [-t/2,w-t,w-t,w,w,w-t,w-t,-t/2];
        yGrip = [t/2,t/2,L/2,L/2,-L/2,-L/2,-t/2,-t/2];
    else
       xGrip = [t/2, w+t, w+t, w, w, w+t, w+t, t/2];
       yGrip = [t/2, t/2, L/2, L/2, -L/2, -L/2, -t/2, -t/2];
    end
    Grip = [xGrip;yGrip];

    dcm = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    Grip = dcm'*Grip + repmat([xG;yG],1,length(Grip(1,:)));
    handle(1) = patch(Grip(1,:),Grip(2,:),[.5,.5,.5]);

    %plot spring and damper
    if left
        handle(2) = DrawSpring(xeq-.3,yeq+.02,xG-.13,yG/2+.02,.02,6,.02);
        handle(3:4) = DrawDamper(xeq-.3,yeq-.02,xG-.13,yG/2-.02,.02,.02,.04,.03);
    else
        handle(2) = DrawSpring(xeq+.3,yeq+.02,xG+.13,yG/2+.02,.02,6,.02);
        handle(3:4) = DrawDamper(xeq+.3,yeq-.02,xG+.13,yG/2-.02,.02,.02,.04,.03);
    end
    
    %plot plate
    if left
        handle(5) = patch([xG-.13,xG-.14,xG-.14,xG-.13],[yG/2-.05,yG/2-.05,yG/2+.05,yG/2+.05],[.5,.5,.5]);
        handle(6) = patch([xeq-.3,xeq-.31,xeq-.31,xeq-.3],[-.05,-.05,.05,.05],[.5,.5,.5]);
    else
        handle(5) = patch([xG+.13,xG+.14,xG+.14,xG+.13],[yG/2-.05,yG/2-.05,yG/2+.05,yG/2+.05],[.5,.5,.5]); 
        handle(6) = patch([xeq+.3,xeq+.31,xeq+.31,xeq+.3],[-.05,-.05,.05,.05],[.5,.5,.5]);
    end
    
    %draw hinge point
    % handle(6) = scatter(xG,yG,20,'y','filled');
    
    %draw flexure
    xf = 0:.01:.13;
    a = (yG/2)/.13^2;
    yf = yG/2 + a*xf.^2;
    if left
        xf = xf - .13 + xG;
    else
        xf = - xf + .13 + xG;
    end
    handle(7) = plot(xf,yf,'k','LineWidth',4);
    
    %Draw torsional springs
    if left
        x1 = xG-.13;
        y11 = yG/2+.03; y12 = yG/2-.03;
        xy21 = [w-t;.04];
        xy21 = dcm'*xy21 + [xG;yG];
        xy22 = [w-t;-.04];
        xy22 = dcm'*xy22 + [xG;yG];
    else
        x1 = xG+.13;
        y11 = yG/2+.03; y12 = yG/2-.03;
        xy22 = [w+t; -.04];
        xy22 = dcm'*xy22 + [xG;yG];
        xy21 = [w+t;.04];
        xy21 = dcm'*xy21 + [xG;yG];
    end
    
    handle(8) = DrawSpring(x1,y11,xy21(1),xy21(2),.01,10,.04);
    handle(9) = DrawSpring(x1,y12,xy22(1),xy22(2),.01,10,.04);
        
    %Draw sweet spot
    xss = [w,w,w+Gripper.d_norm,w+Gripper.d_norm];
    yss = [-Gripper.d_tan/2,Gripper.d_tan/2,Gripper.d_tan/2,-Gripper.d_tan/2];
    xyss = [xss;yss];
    xyss = dcm'*xyss + repmat([xG;yG],1,length(xyss(1,:)));
    handle(10) = patch(xyss(1,:),xyss(2,:),'g','EdgeColor','none');

end

