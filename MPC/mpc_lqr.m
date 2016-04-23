N = 100; % prediction horizon
M =500;

T_s = 0.05;
Q = diag([1,1,10,1*ones(1,nx-3)]);
R = eye(nu);

U_vector = zeros(M,nu);
X_vector = zeros(M,nx);
Y_vector = zeros(M,ny);

% initial state
x = x0_quadcopter;
% x(1:3) = 12;
% x(3) = 20;
% x(1:2) = 12;
% x(4:6) = 5; 
% x(7:9) = 1.5;
x(1:3) = 10; x(4:6) = 2;

options = odeset('RelTol',1e-13,'AbsTol',1e-16);
for k=1:M
    S = Q;
    % solve the Riccati difference equation backwards in time
    for n=1:N-1
        S = riccati_diffeq(S,A_d,B_d,Q,R);
    end
    K = inv(R+B_d'*S*B_d)*B_d'*S*A_d;
    u = -K*x;
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

