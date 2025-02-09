clear all
close all

%%
feasible_pts=readmatrix("random_pts.csv");
%% Ecuaciones de restricción y actuación
syms t...
    L_p1 L_p2 L_d1 L_d2 ...
    O_1x O_2x O_1y O_2y...
    phi_p1 phi_p2 phi_d1 phi_d2...
    x_p1 y_p1...
    x_p2 y_p2 ...
    x_d1 y_d1...
    x_d2 y_d2...
    val_effx val_effy

%Función Auxiliar para caluclar la matriz de rotación
rotmat=@(phi) [cos(phi) -sin(phi);sin(phi) cos(phi)];

% Coordenadas Generalizadas
q=[x_p1;y_p1;phi_p1;...
    x_d1;y_d1;phi_d1;...
    x_d2;y_d2;phi_d2;...
    x_p2;y_p2;phi_p2];

q_0=[-0.14571;0.07071;deg2rad(135);...
    -0.10821;0.30962;deg2rad(57.24);...
    0.10821;0.30962;deg2rad(-57.24);...
    0.14571;0.07071;deg2rad(-135)];

% Ecuaciones de restricción:

% Eslabón proximal 1 - Tierra
eq1=[x_p1;y_p1]+rotmat(phi_p1)*[-L_p1/2;0]==[O_1x;O_1y];

% Eslabón proximal 1 - distal 1
eq2=[x_p1;y_p1]+rotmat(phi_p1)*[L_p1/2;0]==[x_d1;y_d1]+rotmat(phi_d1)*[-L_d1/2;0];

% Eslabón distal 1 - distal 2
eq3=[x_d1;y_d1]+rotmat(phi_d1)*[L_d1/2;0]==[x_d2;y_d2]+rotmat(phi_d2)*[-L_d2/2;0];

% Eslabón distal 2 - proximal 2
eq4=[x_d2;y_d2]+rotmat(phi_d2)*[L_d2/2;0]==[x_p2;y_p2]+rotmat(phi_p2)*[-L_p2/2;0];

% Eslabón proximal 2 - Tierra
eq5=[x_p2;y_p2]+rotmat(phi_p2)*[L_p2/2;0]==[O_2x;O_2y];

eq6=[val_effx;val_effy]==[x_d1;y_d1]+rotmat(phi_d1)*[L_d1/2;0];

PHI=[rhs(eq1)-lhs(eq1);rhs(eq2)-lhs(eq2);rhs(eq3)-lhs(eq3);rhs(eq4)-lhs(eq4);rhs(eq5)-lhs(eq5);rhs(eq6)-lhs(eq6)];

% Parámetros
O_1x_val=-0.075;
O_2x_val=0.075;
O_1y_val=0;
O_2y_val=0;

L_p1_val=0.2;
L_p2_val=0.2;
L_d1_val=0.4;
L_d2_val=0.4;

% Sustituye parámetros
PHI=subs(PHI,[O_1x O_2x O_1y O_2y L_p1 L_p2 L_d1 L_d2],[O_1x_val O_2x_val O_1y_val O_2y_val L_p1_val L_p2_val L_d1_val L_d2_val]);

PHI_q=[diff(PHI,q(1)) diff(PHI,q(2)) diff(PHI,q(3)) ...
    diff(PHI,q(4)) diff(PHI,q(5)) diff(PHI,q(6)) ...
    diff(PHI,q(7)) diff(PHI,q(8)) diff(PHI,q(9)) ...
    diff(PHI,q(10)) diff(PHI,q(11)) diff(PHI,q(12))];

PHI_fun=matlabFunction(PHI,"Vars",[q;val_effx;val_effy]);

PHI_q_fun=matlabFunction(PHI_q,"Vars",[q;val_effx;val_effy]);

%%
tol=1e-6;
max_iters=100;

t=linspace(0,2,1e3);

q_0=[-0.14571;0.07071;deg2rad(135);...
    -0.10821;0.30962;deg2rad(57.244623466);...
    0.10821;0.30962;deg2rad(-57.244623466);...
    0.14571;0.07071;deg2rad(-135)];

qs=[q_0];
effs=[[[q_0(4);q_0(5)]+rotmat(q_0(6))*[L_d1_val/2;0]]];

for i=1:size(feasible_pts,1)
    j=1;
    q_ant=qs(:, 1);
    effx=feasible_pts(i,1);
    effy=feasible_pts(i,2);

    PHI_fun_subs=@(q) PHI_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),effx,effy);
    PHI_q_fun_subs=@(q) PHI_q_fun(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),effx,effy);

    while true
        

        phi_val=PHI_fun_subs(q_0);
        phi_q_val=PHI_q_fun_subs(q_0);

        q_1=q_0-phi_q_val\phi_val;

        if (j>max_iters)
            disp("Exceed iters")
            break
        end
        if (max(abs(phi_val))<=tol)
            qs=[qs q_0];
            effs=[effs [q_0(4);q_0(5)]+rotmat(q_0(6))*[L_d1_val/2;0]];
            disp("Found solution")
            break
        end

        q_0=q_1;
        j=j+1;
    end

end
%%
close all
fig=figure(1);
plot(effs(1,:).',effs(2,:).','*')

%%
data=[qs(3,:);qs(6,:);qs(9,:);qs(12,:);effs(1,:);effs(2,:)].';
%parquetwrite('initial_pts.parquet',array2table(data))
%%
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
    pause(0.1);
    
end
