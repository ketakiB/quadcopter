
close all;
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
Y_vector = zeros(K*M,ny);
ref_vector = zeros(K*M,3);

% initial state
x = x0_quadcopter;
x_i = refs_ss(1,:)'; %integral states


%options = odeset('RelTol',1e-13,'AbsTol',1e-16);
for k=1:K
    for m=1:M
        u = -K_s*x-K_i*x_i;
        y = C_d*x;
        [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x);
        x = X(end,:)';
        x_i = x_i + refs_ss(k,:)'-y(1:3);
        U_vector((k-1)*M+m,:) = u';
        X_vector((k-1)*M+m,:) = x';
        Y_vector((k-1)*M+m,:) = y';
        ref_vector((k-1)*M+m,:)=refs_ss(k,:);
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
title('Simulation results LQR')

figure
plot(T,X_vector);
legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')

figure
plot(T,U_vector);
title('Control inputs');
xlabel('T [s]')