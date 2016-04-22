close all
M = 500;
T_s = 0.05;
Q = diag([1,1,10,1*ones(1,nx-3)]);
R = eye(nu);
P = eye(nx);

U_vector = zeros(M,nu);
X_vector = zeros(M,nx);
Y_vector = zeros(M,ny);

% initial state
x = x0_quadcopter;
% x(1:3) = 17;
% x(1:2) = 18;
% x(4:6) = 10; 
x(7:9) = 1;


options = odeset('RelTol',1e-13,'AbsTol',1e-16);

for k=1:M
    u = -R\B'*P*x;
    % solve the Riccati ODE backwards in time
    [~,P] = ode45(@(t,y) riccatiODE(t,y,A,B,Q,R,nx), [T_s,0], reshape(P,nx*nx,1));
    P = reshape(P(end,:),nx,nx);
    y = C*x;
    [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x,options);
    x = X(end,:)';
    
    U_vector(k,:) = u';
    X_vector(k,:) = x';
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
title('Simulation results MPC')

figure
plot(T,X_vector);
legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')

figure
plot(T,U_vector);
title('Control inputs');
xlabel('T [s]')

