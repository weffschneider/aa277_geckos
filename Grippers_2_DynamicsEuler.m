function [Tout,Xout,caught_i,Fn,Ft] = Grippers_2_DynamicsEuler(T,dt,X_0,Gripper,Object)
    

    X_L_eq = X_0(1:3);
    X_R_eq = X_0(7:9);

    caught_i = 0;
    leftContact = false;
    rightContact = false;
    
    Tout = 0:dt:T;
    N = length(Tout);
    n_states = length(X_0);
    Xout = zeros(N,n_states);
    Fn = zeros(N,1);
    Ft = zeros(N,1);
    Xout(1,:) = X_0;
    
    dx = zeros(1,18);
    
    Ft_min = 1;
    
    for i = 2:N
  
        %convenient variable names
        xGL = Xout(i-1,1);      yGL = Xout(i-1,2);      thetaGL = Xout(i-1,3);
        xGL_dot = Xout(i-1,4);  yGL_dot = Xout(i-1,5);  thetaGL_dot = Xout(i-1,6);
        xGR = Xout(i-1,7);      yGR = Xout(i-1,8);      thetaGR = Xout(i-1,9);
        xGR_dot = Xout(i-1,10);  yGR_dot = Xout(i-1,11);  thetaGR_dot = Xout(i-1,12);
        xO = Xout(i-1,13);      yO = Xout(i-1,14);      xO_dot = Xout(i-1,15);     yO_dot = Xout(i-1,16); 
        omega = Xout(i-1,17);  phi = Xout(i-1,18);
    
        normal_L = [cos(thetaGL); -sin(thetaGL); 0]; %vector normal to left gripper plane
        tangent_L = [sin(thetaGL); cos(thetaGL); 0]; %vector tangent to left gripper plane
        normal_R = -[cos(thetaGR); -sin(thetaGR); 0]; %vector normal to right gripper plane
        tangent_R = -[sin(thetaGR); cos(thetaGR); 0]; %vector tangent to right gripper plane
        
        %define position vectors
        r_Ocm_N0_N = [xO; yO; 0]; % position vector from N0 to Ocm in N basis
        r_GL_N0_N = [xGL; yGL; 0];
        r_GR_N0_N = [xGR; yGR; 0];
        % --- assume w=0 --- %
        % r_Gp_Gcm_N = Gripper.w*normal_L; 
        % r_Ocm_Gp_N = r_Ocm_N0_N - r_Gcm_N0_N - r_Gp_Gcm_N;
        
        % --- assume offset = 0 --- %
        % --- TODO: want non-zero offset when it starts spinning -- %
        % d = dot(r_Ocm_Gp_N, tangent_L); % "offset" distance of contact point from Gp
        
        % r_GO_Gcm_N = r_Gp_Gcm_N + d*tangent_L; % from Gcm to the Gripper's point-of-contact
        r_Ocm_GL_N = (r_GL_N0_N - r_Ocm_N0_N) + Object.R*normal_L; % from Ocm to the object's point-of-contact w/ left gripper
        r_Ocm_GR_N = (r_GR_N0_N - r_Ocm_N0_N) + Object.R*normal_R;
        
        %define velocity vectors
        w_GL_N = [0; 0; -thetaGL_dot];
        w_GR_N = [0; 0; -thetaGR_dot];
        w_O_N = [0; 0; omega];
    
        V_GL_N = [xGL_dot; yGL_dot; 0]; % with w=0, V_Gcm = V_GO
        V_GR_N = [xGR_dot; yGR_dot; 0];

        V_Ocm_N = [xO_dot; yO_dot; 0];
        % V_GO_N = V_Gcm_N + cross(w_G_N, r_GO_Gcm_N); % translational + rotational = total vel of contact point on gripper
        
        V_Ocontact_L_N = V_Ocm_N + cross(w_O_N, r_Ocm_GL_N); % translational + rotational = total vel of contact point on object
        V_Ocontact_R_N = V_Ocm_N + cross(w_O_N, r_Ocm_GR_N);
        V_GL_O_N = V_Ocontact_L_N - V_GL_N; % relative total velocity between object & left gripper
        V_GR_O_N = V_Ocontact_R_N - V_GR_N;
        
        if mod(i,1000) == 0
            plot(r_Ocm_N0_N(1), r_Ocm_N0_N(2), 'bo')
            hold on;
            plot(r_GL_N0_N(1), r_GL_N0_N(2), 'kx')
            plot(r_GR_N0_N(1), r_GR_N0_N(2), 'rx')
            plot([r_GL_N0_N(1),r_GL_N0_N(1)+normal_L(1)], [r_GL_N0_N(2),r_GL_N0_N(2)+normal_L(2)], 'k:')
            plot([r_GR_N0_N(1),r_GR_N0_N(1)+normal_R(1)], [r_GR_N0_N(2),r_GR_N0_N(2)+normal_R(2)], 'r:')
        end
    
        P_L = dot(r_Ocm_GL_N, -normal_L); % normal distance of left contact
        P_R = dot(r_Ocm_GR_N, -normal_R);
        P_L_dot = dot(V_GL_O_N, normal_L); % penetration velocity
        P_R_dot = dot(V_GR_O_N, normal_R);
        V_L_tan = dot(V_GL_O_N, tangent_L); % tangent velocity of contact
        V_R_tan = dot(V_GR_O_N, tangent_R);
        
        %spring/damper forces/moments on left gripper
        Fx_L_spring = -(xGL-X_L_eq(1))*Gripper.Kx;
        Fx_L_damp = -xGL_dot*Gripper.Cx;
        Fx_L = Fx_L_spring + Fx_L_damp;
        Fy_L_spring = -(yGL-X_L_eq(2))*Gripper.Ky;
        Fy_L_damp = -yGL_dot*Gripper.Cy;
        Fy_L = Fy_L_spring + Fy_L_damp;
        F_L_Gripper = [Fx_L;Fy_L;0];
        M_L_spring = Gripper.KM*(thetaGL-X_L_eq(3));
        M_L_damp = Gripper.CM*thetaGL_dot;
        M_L_Gripper = [0; 0; M_L_spring + M_L_damp];
        
        Fx_R_spring = -(xGR-X_R_eq(1))*Gripper.Kx;
        Fx_R_damp = -xGR_dot*Gripper.Cx;
        Fx_R = Fx_R_spring + Fx_R_damp;
        Fy_R_spring = -(yGR-X_R_eq(2))*Gripper.Ky;
        Fy_R_damp = -yGR_dot*Gripper.Cy;
        Fy_R = Fy_R_spring + Fy_R_damp;
        F_R_Gripper = [Fx_R;Fy_R;0];
        M_R_spring = Gripper.KM*(thetaGR-X_R_eq(3));
        M_R_damp = Gripper.CM*thetaGR_dot;
        M_R_Gripper = [0; 0; M_R_spring + M_R_damp];

        
        %% NOTE: in the newer (force limit) paper, there is no mention of friction forces.
        % Just Equation (8) from ICRA17 paper. How does that work???
        
        
        if ~leftContact && P_L<0
            leftContact = true;
        end
        
        Fn_L_spring = -Gripper.Ksurf*P_L*leftContact;
        Fn_L_damp = -Gripper.Csurf*P_L_dot*leftContact*(sign(P_L_dot)<0);
        Fn_L = Fn_L_spring + Fn_L_damp;
        Ft_L = (Ft_min+ Gripper.mu_k1*abs(Fn_L))*leftContact*sign(V_L_tan);        
        
        if leftContact && (Fn_L<0)
            leftContact = false;
        end
            
        F_L_contact = -Fn_L*normal_L + Ft_L*tangent_L;
        F_L_Gripper = F_L_Gripper + F_L_contact;
        
        if ~rightContact && P_R<0
            rightContact = true;
        end   
        
        Fn(i) = Fn_L;
        Ft(i) = Ft_L;
        
        Fn_R_spring = -Gripper.Ksurf*P_R*rightContact;
        Fn_R_damp = -Gripper.Csurf*P_R_dot*rightContact*(sign(P_R_dot)<0);
        Fn_R = Fn_R_spring + Fn_R_damp;
        Ft_R = (Ft_min + Gripper.mu_k1*abs(Fn_R))*rightContact*sign(V_R_tan);
        
        if rightContact && (Fn_R<0)
            rightContact = false;
        end
        
        F_R_contact = -Fn_R*normal_R + Ft_R*tangent_R;
        F_R_Gripper = F_R_Gripper + F_R_contact;
        
        F_Object = -F_L_contact - F_R_contact;
        M_Object = cross(r_Ocm_GL_N, -F_L_contact) + cross(r_Ocm_GR_N, -F_R_contact);
   
        
        dx(1:3)   = Xout(i-1,4:6);
        dx(4:5)   = F_L_Gripper(1:2)/Gripper.m;
        dx(6)     = -M_L_Gripper(3)/Gripper.I;
        dx(7:9)   = Xout(i-1, 10:12);
        dx(10:11) = F_R_Gripper(1:2)/Gripper.m;
        dx(12)    = -M_R_Gripper(3)/Gripper.I;
        dx(13:14) = Xout(i-1,15:16);
        dx(15:16) = F_Object(1:2)/Object.m;
        dx(17)    = M_Object(3)/Object.I;
        dx(18)    = omega;

        Xout(i,:) = Xout(i-1,:) + dx*dt;
        

        % Keep track of max forces, for later
%         if abs(d) < Gripper.d_tan/2 && P < Gripper.d_norm && abs(omega)<(400*pi/180)
%             %i_end = i;
%             caught_i = i;
%             break
%         end
   
    end
        

end