% This code calculates the configuration of the mechanism needed for a
% given position of the effector within the workspace
%
% The analysis is based on generalized coordinates.
%
% Author: Juan Pablo Reyes
clearvars
close all
%% Reads the uniformly distributed points in the robot workspace
feasible_pts=readmatrix("random_pts.csv");
%% Constraint and actuation equations

% Parameters
scale=0.5;

% Motor coordinates
O_1x_val=-0.075*scale;
O_2x_val=0.075*scale;
O_1y_val=0*scale;
O_2y_val=0*scale;

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
    val_effx val_effy

% Auxiliary function for the rotation matrix
rotmat=@(phi) [cos(phi) -sin(phi);sin(phi) cos(phi)];

% Generalized coodinates vector
q=[x_p1;y_p1;phi_p1;...
    x_d1;y_d1;phi_d1;...
    x_d2;y_d2;phi_d2;...
    x_p2;y_p2;phi_p2];

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

% Defines the position of end effector
eq6=[val_effx;val_effy]==[x_d1;y_d1]+rotmat(phi_d1)*[L_d1/2;0];

% Constraint vector
PHI=[rhs(eq1)-lhs(eq1);rhs(eq2)-lhs(eq2);rhs(eq3)-lhs(eq3);rhs(eq4)-lhs(eq4);rhs(eq5)-lhs(eq5);rhs(eq6)-lhs(eq6)];

% Substitutes the parameters numerical values
PHI=subs(PHI,[O_1x O_2x O_1y O_2y L_p1 L_p2 L_d1 L_d2],[O_1x_val O_2x_val O_1y_val O_2y_val L_p1_val L_p2_val L_d1_val L_d2_val]);

% Partial derivative matrix of PHI respect to the generalized coordinates
PHI_q=[diff(PHI,q(1)) diff(PHI,q(2)) diff(PHI,q(3)) ...
    diff(PHI,q(4)) diff(PHI,q(5)) diff(PHI,q(6)) ...
    diff(PHI,q(7)) diff(PHI,q(8)) diff(PHI,q(9)) ...
    diff(PHI,q(10)) diff(PHI,q(11)) diff(PHI,q(12))];

% Turns symbolic expressions to matlab functions for efficient computation
PHI_fun=matlabFunction(PHI,"Vars",[q;val_effx;val_effy]);

PHI_q_fun=matlabFunction(PHI_q,"Vars",[q;val_effx;val_effy]);

%% Random configuration finder

% Newton Rhapson Solver Parameters
tol=1e-6;
max_iters=50;

% Initial feasible configuration
q_0=[-0.14571*scale;0.07071*scale;deg2rad(135);...
    -0.10821*scale;0.30962*scale;deg2rad(57.244623466);...
    0.10821*scale;0.30962*scale;deg2rad(-57.244623466);...
    0.14571*scale;0.07071*scale;deg2rad(-135)];

% Tries to solve for data point in feasible_pts
configurations=size(feasible_pts,1);

disp(configurations)

% Preallocate arrays
qs=zeros(12,configurations);
effs=zeros(2,configurations);

% Records q and the effector positions
qs(:,1)=q_0;
effs(:,1)=[q_0(4);q_0(5)]+rotmat(q_0(6))*[L_d1_val/2;0];

for i=2:configurations
    j=1;% Reset iteration counter

    q_ant=qs(:,randi(i-1)); % Random solution from the ones solved

    q_0=q_ant;% Use random solution as the initial guess for the next iteration


    % Random feasible positions are given to the actuator
    effx=feasible_pts(i,1);
    effy=feasible_pts(i,2);

    % Defines matlab functions for phi and phi_q given effx and effy
    PHI_fun_subs=@(q) PHI_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),effx,effy);
    PHI_q_fun_subs=@(q) PHI_q_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),effx,effy);

    while true
        % Calculate phi and phi_q
        phi_val=PHI_fun_subs(q_0);
        phi_q_val=PHI_q_fun_subs(q_0);

        % Calculate the update value using Newton Rhapson
        q_1=q_0-phi_q_val\phi_val;

        % End if the tolerance is satisfied or the max iterations are reached
        if (j>max_iters)
            qs(:,i)=q_ant;
            effs(:,i)=[q_ant(4);q_ant(5)]+rotmat(q_ant(6))*[L_d1_val/2;0];
            break
        end
        if (max(abs(phi_val))<=tol)
            qs(:,i)=q_0;
            effs(:,i)=[q_0(4);q_0(5)]+rotmat(q_0(6))*[L_d1_val/2;0];
            if mod(i, (configurations/1000)) == 0; fprintf('Progress: %.1f %%\n',i/(configurations/100)); end
            break
        end

        % Update the guess and iteration counter
        q_0=q_1;
        j=j+1;
    end

end
%% Plot point workspace
close all    
figure(1);
plot(effs(1,:).',effs(2,:).','*')

%% Writes the orientations and effector positions to Parquet
data=[qs(3,:);qs(6,:);qs(9,:);qs(12,:);effs(1,:);effs(2,:)].';
parquetwrite('initial_pts.parquet',array2table(data))
%% Animation of the generated configurations
fig=figure(2);
axis([-0.8 0.8 -0.8 0.8])
axis(gca,'equal')

eslab1=viscircles([O_1x_val O_1y_val],L_p1_val,'LineStyle','--');
eslab2=viscircles([O_2x_val O_2y_val],L_p2_val,'LineStyle','--');
mecha=line(0,0);
for i=1:length(qs)
    q_act=qs(:,i);
    delete(mecha);
    pts=[[q_act(1);q_act(2)]+rotmat(q_act(3))*[-L_p1_val/2;0] ...
        [q_act(1);q_act(2)]+rotmat(q_act(3))*[L_p1_val/2;0] ...
        [q_act(4);q_act(5)]+rotmat(q_act(6))*[L_d1_val/2;0] ...
        [q_act(10);q_act(11)]+rotmat(q_act(12))*[-L_p2_val/2;0] ...
        [q_act(10);q_act(11)]+rotmat(q_act(12))*[L_p2_val/2;0]];
    
    mecha=line(pts(1,:),pts(2,:));
    axis([-0.8 0.8 -0.8 0.8])
    pause(1);
    
end
