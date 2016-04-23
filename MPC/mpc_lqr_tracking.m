N = 100; % prediction horizon

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

T_s = 0.05;
load 'references.mat';
[M,~] = size(refs);
refs = [refs, zeros(M,1)];
Q = diag([10,10,1e4,1*ones(1,nx-3)]);
R = eye(nu);

U_vector = zeros(M,nu);
X_vector = zeros(M,nx);
Y_vector = zeros(M,ny);

% initial state
x = x0_quadcopter;

options = odeset('RelTol',1e-13,'AbsTol',1e-16);
for k=1:M
    S = Q;
    % solve the Riccati difference equation backwards in time
    for n=1:N-1
        S = riccati_diffeq(S,A_d,B_d,Q,R);
    end
    K = inv(R+B_d'*S*B_d)*B_d'*S*A_d;
    r = refs(k,:)'; % reference 
    u = -K*(x-N_x*r)+N_u*r;
    y = C*x;
    [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x,options);
    x = X(end,:)';
    
    U_vector(k,:) = u';
    X_vector(k,:) = x';
    Y_vector(k,:) = y';
end


T=T_s*(0:M-1);

figure
plot(T,refs,'--');
hold on
plot(T,Y_vector(:,1:3));
legend({'x_{ref}','y_{ref}','z_{ref}','x','y','z'},'FontSize',18);
title('Resulting outputs');
xlabel('T [s]')

figure
plot3(refs(:,1),refs(:,2),refs(:,3),'ro');
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

