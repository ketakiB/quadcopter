close all

%Constructing the Augmented system with the first 3 outputs only (number of
%states is 12+3 = 15

big_A = [A_d  zeros(nx,3) ;
       -C_d(1:3,:) eye(3)];
big_B = [B_d ; -D_d(1:3,:)];

big_nx = nx+3; % Number of states of the augmented system

% Checking the controlabillity of the Augmented system
disp('Rank of the controllability matrix of the augmented system:');
rank(ctrb(big_A,big_B))

load 'references_ss.mat';
[K,~] = size(refs_ss);
M = 125;
T_s = 0.05;
Q_i = diag([1e0*ones(1,3),1e0*ones(1,3),1e3*ones(1,3),1e0*ones(1,3),1e0*ones(1,2),1e4]);
R_i = 1e1*eye(nu);

% Computing the feedback matrix of the augmented system
[big_K,S,~] = dlqr(big_A, big_B, Q_i, R_i);
K_s = big_K(:,1:nx);
K_i = big_K(:,nx+1:end);

U_vector = zeros(K*M,nu);
X_vector = zeros(K*M,nx);
X_k_vector = zeros(K*M,nx);
Y_vector = zeros(K*M,ny);
ref_vector = zeros(K*M,3);

% initial state
x = x0_quadcopter;
x_i = refs_ss(1,:)'; %integral states
x_k = x; % kalman filter estimated states 

for k=1:K
    for m=1:M
        u = -K_s*x_k-K_i*x_i;
        y = C_d*x;
        [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x);
        x = X(end,:)';
        x_k = A_k*x_k + B_k*[u; y]; % state estimates 
        x_i = x_i + refs_ss(k,:)'-y(1:3);
        U_vector((k-1)*M+m,:) = u';
        X_vector((k-1)*M+m,:) = x';
        Y_vector((k-1)*M+m,:) = y';
        ref_vector((k-1)*M+m,:)=refs_ss(k,:);
        X_k_vector((k-1)*M+m,:) = x_k';
    end 
end

T=T_s*(0:M*K-1);

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
for k=1:K
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
legend({'\phi','\tau','\psi','\phi_k','\tau_k','\psi_k'},'FontSize',18);
%legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')

figure
plot(T,U_vector);
title('Control inputs');
xlabel('T [s]')