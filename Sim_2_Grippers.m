%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gecko gripper simulation (planar, single rigid body)
% Author: Ben Hockman
% Date: 09/10/2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear all
% clc

%----------------------Gripper Parameters------------------------%
m_G = .05; %kg  (mass of gripper)
span = .26; %m  (wingspan of gripper)
I_G = 1/12*m_G*.8*span^2; %kg*m^2  (inertia of gripper about cm)

mu_s1 = 1.5;  %static friction in direction of adhesive
mu_k1 = .5;  %kinetic friction in direction of adhesive
mu_s2 = 1.5;  %static friction against direction of adhesive
mu_k2 = 1;  %kinetic friction against direction of adhesive
alpha = -.25;
beta = pi/6; % fix this eventually

Ksurf = 1e5;
Csurf = 0.1e5;
Kx = 1e5; Cx = .01e5; Ky = 1e5; Cy = .01e5;
CM = .001e5; KM = .001e5;
% Ksurf = 1000; %N/m  (contact spring stiffness)
% Csurf = 20; %N*s/m (contact damper stiffness)
% 
% Kx = 300; %N/m  (X spring stiffness)
% Cx = 1;  %Ns/m (X damping coeff)
% Ky = 97; %N/m (lateral spring stiffness)
% Cy = .15;  %Ns/m (X damping coeff)
% KM = .09; %Nm/rad (torsional spring stiffness)
% CM = .001; %Nms/rad (X damping coeff)

w = 0.0; %can't deal with this right now .01; %m  (hinge offset from gripper plane)
d_tan = .02; %m  (width of gripper "sweet spot")
d_norm = .001; %m  (depth of gripper "sweet spot")

Gripper = struct('m',m_G,'I',I_G,'mu_s1',mu_s1,'mu_k1',mu_k1,'mu_s2',mu_s2, ...
           'mu_k2',mu_k2,'w',w,'span',span,'d_tan',d_tan,'d_norm',d_norm,...
           'Ksurf',Ksurf,'Csurf',Csurf, ...
           'Kx',Kx,'Cx',Cx,'Ky',Ky,'Cy',Cy,'KM',KM,'CM',CM,'alpha',alpha);
%----------------------------------------------------------------%


%-----------------------Object Parameters------------------------%
m_o = 0.01; % 1.5526; %kg  (mass of object)
R_o = 0.4; %9*.5*.0254; %m  (radius of object)
I_o = 0.01; %.0128;%.0214; %kg*m^2  (inertia of object)
Object = struct('m',m_o,'I',I_o,'R',R_o);
%----------------------------------------------------------------%


%----------------------Initial Conditions------------------------%
%---Left Gripper (Origin at left gripper CM) ---%
x_GL = .0; %m
y_GL = .0; %m
theta_GL = pi/2 - beta; %rad
x_GL_dot = .0; %m/s
y_GL_dot = .0; %m/s
theta_GL_dot = .0; %rad/s
%---Right Gripper---%
x_GR = 2*Object.R*sin(beta); %m
y_GR = .0; %m
theta_GR = -(pi/2 - beta); %rad
x_GR_dot = .0; %m/s
y_GR_dot = .0; %m/s
theta_GR_dot = .0; %rad/s

%---Object---%
x_o = Object.R/2; %m
y_o = -0.7; %m
x_o_dot = 0; %m/s
y_o_dot = 5.0; %m/s
phi = 0;
omega = 0.3; %rad/s

speed = 1; %animation playback speed
T = 5; %max time to run simulation
dt = .0001;

% offset = .08; angle = 0*pi/180; vel = .3; Dist = .15;
% [x_o, y_o, x_o_dot, y_o_dot] = GetICs(offset, angle, vel, Object, Gripper, Dist);

X_0 = [x_GL, y_GL, theta_GL, x_GL_dot, y_GL_dot, theta_GL_dot, ...
       x_GR, y_GR, theta_GR, x_GR_dot, y_GR_dot, theta_GR_dot, ...
       x_o, y_o, x_o_dot, y_o_dot, omega, phi];

%----------------------------------------------------------------%

[Tout,Xout,caught,Fn,Ft] = Grippers_2_DynamicsEuler(T,dt,X_0,Gripper,Object);


% TerminalEvent = @(t, x)GripEvent(t, x, Gripper, Object);
% options = odeset('Events',TerminalEvent,'RelTol',1e-6);
%options = odeset('RelTol',1e-6);

% offset_success = zeros(1,100);
% offset_failure = zeros(1,100);
% angvel_success = zeros(1,100);
% angvel_failure = zeros(1,100);
% 
% i_success = 1;
% i_failure = 1;
% tic
% for offset = 0.1 %-.13:.05:.13
%     for rotation =  100 % -1000:100:1000
%         omega = rotation*pi/180;
%         [x_o, y_o, x_o_dot, y_o_dot] = GetICs(offset, angle, vel, Object, Gripper, Dist);
%         X_0 = [x_G, y_G, theta, x_G_dot, y_G_dot, theta_dot, ...
%                x_o, y_o, x_o_dot, y_o_dot, omega, phi];
%         %[Tout,Xout] = ode45(@(t,x)GripperDynamics(t,x,Gripper,Object),[0,T],X_0,options);
%         [Tout,Xout,caught] = GripperDynamicsEuler(T,dt,X_0,Gripper,Object);
%         if caught
%             offset_success(i_success) = offset;
%             angvel_success(i_success) = rotation;
%             i_success = i_success + 1;
%         else
%             offset_failure(i_failure) = offset;
%             angvel_failure(i_failure) = rotation;
%             i_failure = i_failure + 1;
%         end
%     end
% end
% toc
% offset_success = offset_success(1:i_success-1);
% angvel_success = angvel_success(1:i_success-1);
% offset_failure = offset_failure(1:i_failure-1);
% angvel_failure = angvel_failure(1:i_failure-1);

% figure;
% hold on;
% plot(offset_success, angvel_success, 'x')
% plot(offset_failure, angvel_failure, 'o')
% 


%ODE45 solver. Only works for non-stiff formulations
% [Tout,Xout] = ode45(@(t,x)GripperDynamics(t,x,Gripper,Object),[0,T],X_0,options);

%Manual Euler implemen  tation. 
% [Tout,Xout,caught_i] = GripperDynamicsEuler(T,dt,X_0,Gripper,Object);
% if caught_i
%     Tout = Tout(1:caught_i-1);
%     Xout = Xout(1:caught_i-1,:);
% end

% Animate_2_Grippers(Tout,Xout,Gripper,Object,speed);
%MovieGen(Tout,Xout,Gripper,Object,speed,filename)
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%