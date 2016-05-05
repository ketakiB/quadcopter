close all;

%For a square system, we only use outputs x,y,z and psi
% If we use x,y,z and phi or x,y,z and theta, the matrix big_A is singular
big_A = [A_d - eye(nx), B_d; 
        C_d(1:3,:), D_d(1:3,:);
        C_d(6,:), D_d(6,:)];

big_b = [zeros(nx,4);
        eye(4)];
big_N = inv(big_A)*big_b;

N_x = big_N(1:nx,:); % x_ss = N_x * ref
N_u = big_N(nx+1:end,:); % u_ss = N_u * ref

load 'references_ss.mat';
[I,~] = size(refs_ss);
refs_ss = [refs_ss, zeros(I,1)];
M = 125;
T_s = 0.05;
Q = diag([10,10,1e5,1*ones(1,nx-3)]);
R = eye(nu);

% Computing the feedback matrix of the augmented system
[K,S,~] = dlqr(A_d, B_d, Q, R);

U_vector = zeros(I*M,nu);
X_vector = zeros(I*M,nx);
X_k_vector = zeros(I*M,nx);
Y_vector = zeros(I*M,ny);
ref_vector = zeros(I*M,3);

% initial state
x = x0_quadcopter;
x_k = x; % kalman filter estimated states 

for k=1:I
    for m=1:M
        r = refs_ss(k,:)'; % reference 
        u = -K*(x_k-N_x*r)+N_u*r;
        y = C_d*x;
        [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x);
        x = X(end,:)';
        x_k = A_k*x_k + B_k*[u; y]; % state estimates 
        U_vector((k-1)*M+m,:) = u';
        X_vector((k-1)*M+m,:) = x';
        Y_vector((k-1)*M+m,:) = y';
        ref_vector((k-1)*M+m,:)=refs_ss(k,1:3);
        X_k_vector((k-1)*M+m,:) = x_k';
    end 
end

T=T_s*(0:M*I-1);

figure
plot(T,ref_vector);
hold on
plot(T,Y_vector(:,1:3));
legend({'x_ref','y_ref','z_ref','x','y','z'},'FontSize',18);
title('Resulting outputs');
xlabel('T [s]')

figure
plot3(Y_vector(:,1),Y_vector(:,2),Y_vector(:,3));
hold on
for k=1:I
    plot3(refs_ss(k,1),refs_ss(k,2),refs_ss(k,3),'ro-');
end
legend({'quadcopter','reference'},'FontSize',18);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Simulation results LQG')

figure
plot(T,X_vector(:,1:3),'--');
hold on 
plot(T,X_k_vector(:,1:3));
legend({'x','y','z','x_k','y_k','z_k'},'FontSize',18);
%legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')

figure
plot(T,X_vector(:,4:6),'--');
hold on 
plot(T,X_k_vector(:,4:6));
legend({'v_x','v_y','v_z','v_x_k','v_y_k','v_z_k'},'FontSize',18);
%legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')

figure
plot(T,U_vector);
title('Control inputs');
xlabel('T [s]')