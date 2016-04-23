close all

M = 1000;
T_s = 0.05;
Q = diag([1,1,10,1*ones(1,nx-3)]);
R = eye(nu);
[K,S] = dlqr(A_d,B_d,Q,R);

U_vector = zeros(M,nu);
X_vector = zeros(M,nx);
X_k_vector = zeros(M,nx);
Y_vector = zeros(M,ny);

% initial state
x = x0_quadcopter;
% x(1:3) = 12;
% x(3) = 20;
% x(1:2) = 12;
% x(4:6) = 5; 
% x(7:9) = 1;
x(1:3) = 10; x(4:6) = 2;

x_k = x; % kalman filter estimated states 


%options = odeset('RelTol',1e-13,'AbsTol',1e-16);
for k=1:M
    u = -K*x_k;
    y = C*x;
    [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x);
    x = X(end,:)';
    x_k = A_k*x_k + B_k*[u; y]; % state estimates 
    U_vector(k,:) = u';
    X_vector(k,:) = x';
    X_k_vector(k,:) = x_k';
    Y_vector(k,:) = y';
end

T=T_s*(0:M-1);

figure
plot(T,Y_vector(:,1:3));
legend({'x','y','z'},'FontSize',18);
title('Resulting outputs');
xlabel('T [s]')

figure
plot3(0,0,0,'ro-');
hold on
plot3(Y_vector(:,1),Y_vector(:,2),Y_vector(:,3));
legend({'quadcopter'},'FontSize',18);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Simulation results LQR')

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