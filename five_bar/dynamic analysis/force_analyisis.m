clc;
clear all

syms F1x F1y ...
        F2x F2y ...
        F3x F3y ...
        F4x F4y ...
        F5x F5y ...
        F6x F6y ...
        F7x F7y ...
        F8x F8y ...
        F9x F9y ...
        F10x F10y ...
        T1 T2 ...
        gamma L_p L_d Fa q1 q2 q3 q4

eqs=[F1x+F2x==0;
    F1y+F2y==0;
    T1-sin(pi-q1)*L_p*F2x-cos(pi-q1)*L_p*F2y;
    F2x==-F3x;
    F2y==-F3y;
    F3x+F4x==0;
    F3y+F4y==0;
    cos(q2)*L_d*F4y-sin(q2)*L_d*F4x==0;
    F4x==-F5x;
    F4y==-F5y;
    F5x+F6x+Fa*cos(gamma)==0;
    F5y+F6y+Fa*sin(gamma)==0;
    F9x+F10x==0;
    F9y+F10y==0;
    T2+cos(pi+q4)*L_p*F9y-sin(pi+q4)*L_p*F9x==0;
    F8x==-F9x;
    F8y==-F9y;
    F7x+F8x==0;
    F7y+F8y==0;
    -cos(pi/2+q3)*L_d*F7x-sin(pi/2+q3)*L_d*F7y==0;
    F6x==-F7x;
    F6y==-F7y];

[A,b]=equationsToMatrix(eqs,[F1x F1y F2x F2y F3x F3y F4x F4y F5x F5y F6x F6y F7x F7y F8x F8y F9x F9y F10x F10y T1 T2]);
sol=A\b;
solFcn=matlabFunction(subs(sol,[L_p L_d Fa],[0.2 0.4 3]),'Vars',{gamma,q1,q2,q3,q4});

%%
data=parquetread("initial_pts.parquet");
mat=[];
for i=1:9679
    row=data(i,:);
    q1_=row.data1;
    q2_=row.data2;
    q3_=row.data3;
    q4_=row.data4;
    effx=row.data5;
    effy=row.data6;

    %gettorque1(q1_,q2_,q3_,q4_,pi/6,solFcn)
    %gettorque2(q1_,q2_,q3_,q4_,pi/6,solFcn)
    
     mot1=@(x) -gettorque1(q1_,q2_,q3_,q4_,x,solFcn);
     mot2=@(x) -gettorque2(q1_,q2_,q3_,q4_,x,solFcn);
    
     [x1,v1]=fminsearch(mot1,0);
     [x2,v2]=fminsearch(mot2,0);
    disp(i)
    mat=[mat;effx,effy,-v1,-v2];
    
end
%%
figure;
scatter3(mat(:,1), mat(:,2), mat(:,3), 20, mat(:,3), 'filled'); % Heatmap for v1
colorbar;
xlabel('effx');
ylabel('effy');
title('Heatmap of Torque 1');

figure;
scatter3(mat(:,1), mat(:,2), mat(:,4), 20, mat(:,4), 'filled'); % Heatmap for v2
colorbar;
xlabel('effx');
ylabel('effy');
title('Heatmap of Torque 2');

%%
function torque1=gettorque1(q1_v,q2_v,q3_v,q4_v,gamma_v,solFcn)
    sol=solFcn(gamma_v,q1_v,q2_v,q3_v,q4_v);
    torque1=sol(end);
    torque2=sol(end-1);
end
function torque2=gettorque2(q1_v,q2_v,q3_v,q4_v,gamma_v,solFcn)
    sol=solFcn(gamma_v,q1_v,q2_v,q3_v,q4_v);
    torque1=sol(end);
    torque2=sol(end-1);
end