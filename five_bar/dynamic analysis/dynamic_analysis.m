% This code simulates the rectilinear motion of the effector of a 5 bar
% mechanism.
%
% The analysis is based on generalized coordinates.
%
% Author: Juan Pablo Reyes

%%
clearvars
close all
%% Constraint and actuation equations

% Parameters
scale=0.5;

% Motor coordinates
O_1x_val=-0.075*scale;
O_2x_val=0.075*scale;
O_1y_val=0*scale;
O_2y_val=0*scale;

% Initial position, velocity and angle for rectilinear motion
p_x_0_val=-0.12*scale;
p_y_0_val=0.3*scale;
v_lin_val=0.1;
theta_act_val=deg2rad(30);

% Link lengths
L_p1_val=0.2*scale;
L_p2_val=0.2*scale;
L_d1_val=0.4*scale;
L_d2_val=0.4*scale;

% Symbolic variables
syms t...
    L_p1 L_p2 L_d1 L_d2 ...
    O_1x O_2x O_1y O_2y...
    phi_p1 phi_p2 phi_d1 phi_d2...
    x_p1 y_p1...
    x_p2 y_p2 ...
    x_d1 y_d1...
    x_d2 y_d2...
    dphi_p1 dphi_p2 dphi_d1 dphi_d2...
    dx_p1 dy_p1...
    dx_p2 dy_p2 ...
    dx_d1 dy_d1...
    dx_d2 dy_d2...
    p_x_0 p_y_0 v_lin theta_act

% Auxiliary function for the rotation matrix
rotmat=@(phi) [cos(phi) -sin(phi);sin(phi) cos(phi)];

% Generalized coodinates vector
q=[x_p1;y_p1;phi_p1;...
    x_d1;y_d1;phi_d1;...
    x_d2;y_d2;phi_d2;...
    x_p2;y_p2;phi_p2];

% Derivative of the generalized coodinates vector
dq=[dx_p1;dy_p1;dphi_p1;...
    dx_d1;dy_d1;dphi_d1;...
    dx_d2;dy_d2;dphi_d2;...
    dx_p2;dy_p2;dphi_p2];

% Joint equations:

% Proximal link 1 - Ground
eq1=[x_p1;y_p1]+rotmat(phi_p1)*[-L_p1/2;0]==[O_1x;O_1y];

% Proximal link 1 - Distal Link 1
eq2=[x_p1;y_p1]+rotmat(phi_p1)*[L_p1/2;0]==[x_d1;y_d1]+rotmat(phi_d1)*[-L_d1/2;0];

% Distal Link 1 - Distal Link 2
eq3=[x_d1;y_d1]+rotmat(phi_d1)*[L_d1/2;0]==[x_d2;y_d2]+rotmat(phi_d2)*[-L_d2/2;0];

% Distal Link 2 - Proximal Link 2
eq4=[x_d2;y_d2]+rotmat(phi_d2)*[L_d2/2;0]==[x_p2;y_p2]+rotmat(phi_p2)*[-L_p2/2;0];

% Proximal Link 2 - Ground
eq5=[x_p2;y_p2]+rotmat(phi_p2)*[L_p2/2;0]==[O_2x;O_2y];

% Actuation equations:

% End effector linear velocity equation
eq6=[p_x_0+t*v_lin*cos(theta_act);p_y_0+t*v_lin*sin(theta_act)]==[x_d1;y_d1]+rotmat(phi_d1)*[L_d1/2;0];

% Constraint vector
PHI=[rhs(eq1)-lhs(eq1);rhs(eq2)-lhs(eq2);rhs(eq3)-lhs(eq3);rhs(eq4)-lhs(eq4);rhs(eq5)-lhs(eq5);rhs(eq6)-lhs(eq6)];

% Substitutes the parameters numerical values
PHI=subs(PHI,[O_1x O_2x O_1y O_2y p_x_0 p_y_0 v_lin theta_act L_p1 L_p2 L_d1 L_d2],[O_1x_val O_2x_val O_1y_val O_2y_val p_x_0_val p_y_0_val v_lin_val theta_act_val L_p1_val L_p2_val L_d1_val L_d2_val]);

% Partial derivative of PHI respect to time
PHI_t=diff(PHI,t);

% Second partial derivative of PHI respect to time
PHI_tt=diff(PHI,t,t);

% Partial derivative matrix of PHI respect to the generalized coordinates
PHI_q=[diff(PHI,q(1)) diff(PHI,q(2)) diff(PHI,q(3)) ...
    diff(PHI,q(4)) diff(PHI,q(5)) diff(PHI,q(6)) ...
    diff(PHI,q(7)) diff(PHI,q(8)) diff(PHI,q(9)) ...
    diff(PHI,q(10)) diff(PHI,q(11)) diff(PHI,q(12))];

% Derivative of PHI_q respect to time
PHI_qt=diff(PHI_q,t);

% Derivative q
dq_eq=simplify(-inv(PHI_q)*PHI_t);

% PHI*dq
PHIqdq=PHI_q*dq;

% Derivative of PHI*dq respect to q
PHIqdq_q=simplify([diff(PHIqdq,q(1)) diff(PHIqdq,q(2)) diff(PHIqdq,q(3)) ...
    diff(PHIqdq,q(4)) diff(PHIqdq,q(5)) diff(PHIqdq,q(6)) ...
    diff(PHIqdq,q(7)) diff(PHIqdq,q(8)) diff(PHIqdq,q(9)) ...
    diff(PHIqdq,q(10)) diff(PHIqdq,q(11)) diff(PHIqdq,q(12))]);

% Second derivative q
ddq_eq=-inv(PHI_q)*(PHIqdq_q*dq+2*PHI_qt*dq+PHI_tt);

% Turns symbolic expressions to matlab functions for efficient computation
PHI_fun=matlabFunction(PHI,"Vars",[q;t]);
PHI_fun=@(q,t) PHI_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),t);
 
PHI_t_fun=matlabFunction(PHI_t,"Vars",[q;t]);
PHI_t_fun=@(q,t) PHI_t_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),t);

PHI_q_fun=matlabFunction(PHI_q,"Vars",[q;t]);
PHI_q_fun=@(q,t) PHI_q_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),t);

PHI_tt_fun=matlabFunction(PHI_tt,"Vars",[q;t]);
PHI_tt_fun=@(q,t) PHI_tt_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),t);

PHI_qt_fun=matlabFunction(PHI_qt,"Vars",[q;t]);
PHI_qt_fun=@(q,t) PHI_qt_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),t);

dq_fun=matlabFunction(dq_eq,"Vars",[q;t]);
dq_fun=@(q,t) dq_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),t);

ddq_fun=matlabFunction(ddq_eq,"Vars",[q;dq;t]);
ddq_fun=@(q,dq,t) ddq_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),...
    dq(1),dq(2),dq(3),dq(4),dq(5),dq(6),dq(7),dq(8),dq(9),dq(10),dq(11),dq(12),t);


%% Trajectory Simulation

% Newton Rhapson Solver Parameters
tol=1e-6;
max_iters=50;

% Time vector
t=0:0.01:2;

% Initial feasible configuration
q_0=[-0.14571*scale;0.07071*scale;deg2rad(135);...
    -0.10821*scale;0.30962*scale;deg2rad(57.244623466);...
    0.10821*scale;0.30962*scale;deg2rad(-57.244623466);...
    0.14571*scale;0.07071*scale;deg2rad(-135)];

% Records q, dq and ddq for every time step
qs=zeros(12,length(t));
dqs=zeros(12,length(t));
ddqs=zeros(12,length(t));

for i=1:length(t)
    j=1; % Reset iteration counter
    q_ant=q_0; % Use previous solution as the initial guess for the next iteration
    while true
        % Calculate phi and phi_q
        phi_val=PHI_fun(q_0,t(i));
        phi_q_val=PHI_q_fun(q_0,t(i));

        % Calculate the update value using Newton Rhapson
        q_1=q_0-phi_q_val\phi_val;

        % End if the tolerance is satisfied or the max iterations are reached
        if (j>max_iters)
            qs(:,i)=q_ant;
            break
        end
        if (max(abs(phi_val))<=tol)
            qs(:,i)=q_0;
            break
        end

        % Update the guess and iteration counter
        q_0=q_1;
        j=j+1;
    end
    % Calculate dq and ddq analytically
    dq_act=dq_fun(q_0,t(i));
    dqs(:,i)=dq_act;
    ddq_act=ddq_fun(q_0,dq_act,t(i));
    ddqs(:,i)=ddq_act;
end

%% Animates the trajectory using lines
close all
figure(1);
axis([-0.6 0.6 -0.6 0.6])
axis(gca,'equal')
eslab1=viscircles([O_1x_val O_1y_val],L_p1_val,'LineStyle','--');
eslab2=viscircles([O_2x_val O_2y_val],L_p2_val,'LineStyle','--');
mecha=line(0,0);
for i=1:length(t)
    q_act=qs(:,i);
    delete(mecha);
    pts=[[q_act(1);q_act(2)]+rotmat(q_act(3))*[-L_p1_val/2;0] ...
        [q_act(1);q_act(2)]+rotmat(q_act(3))*[L_p1_val/2;0] ...
        [q_act(4);q_act(5)]+rotmat(q_act(6))*[L_d1_val/2;0] ...
        [q_act(10);q_act(11)]+rotmat(q_act(12))*[-L_p2_val/2;0] ...
        [q_act(10);q_act(11)]+rotmat(q_act(12))*[L_p2_val/2;0]];
    
    mecha=line(pts(1,:),pts(2,:));
    
    pause(0.001);
    
end
%% Graphs (Compares analytical derivatives with numerical derivatives)
fig=figure(2);
subplot(3,1,1)
hold on
plot(t,qs(3,:))
plot(t,qs(12,:))
legend(["$\phi_1$","$\phi_4$"],'Interpreter', 'latex')
xlabel('Tiempo (s)')
ylabel("Posición Angular (rad)")
title("Posición")
subplot(3,1,2)
hold on
plot(t,dqs(3,:))
plot(t,gradient(qs(3,:),t),"--")
plot(t,dqs(12,:))
plot(t,gradient(qs(12,:),t),"--")
legend({'$\dot{\phi}_1$', '$\dot{\phi}_{1 \mathrm{num}}$', '$\dot{\phi}_4$', '$\dot{\phi}_{4 \mathrm{num}}$'}, 'Interpreter', 'latex');
xlabel('Tiempo (s)')
ylabel("Velocidad Angular (rad/s)")
title("Velocidad")
subplot(3,1,3)
hold on
plot(t,ddqs(3,:))
plot(t,gradient(gradient(qs(3,:),t),t),"--")
plot(t,ddqs(12,:))
plot(t,gradient(gradient(qs(12,:),t),t),"--")
legend({'$\ddot{\phi}_1$', '$\ddot{\phi}_{1 \mathrm{num}}$', '$\ddot{\phi}_4$', '$\ddot{\phi}_{4 \mathrm{num}}$'}, 'Interpreter', 'latex');
xlabel('Tiempo (s)')
ylabel("Aceleracón Angular (rad/s^2)")
title("Aceleración")

sgtitle("Cinemática")
%% Store data to CSV (Only orientations)
data=[t;qs(3,:);qs(12,:);dqs(3,:);dqs(12,:)].';
writematrix(data,"trajectory_data.csv");
