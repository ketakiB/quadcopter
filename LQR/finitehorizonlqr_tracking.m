close all;

%For a square system, we only use outputs x,y,z and psi
% If we use x,y,z and phi or x,y,z and theta, the matrix big_A is singular
big_A = [A_d - eye(nx), B_d; 
        C_d(1:3,:), D_d(1:3,:);
        C_d(6,:), D_d(6,:)];

big_b = [zeros(nx,4);
        eye(4)];
big_N = (big_A)\big_b;

N_x = big_N(1:nx,:); % x_ss = N_x * ref
N_u = big_N(nx+1:end,:); % u_ss = N_u * ref

load 'references_ss.mat';
[L,~] = size(refs_ss);
refs_ss = [refs_ss, zeros(L,1)]; 
M = 125;
T_s = 0.05;
Q = diag([10,10,1e4,1*ones(1,nx-3)]);
R = eye(nu);

U_vector = zeros(L*M,nu);
X_vector = zeros(L*M,nx);
Y_vector = zeros(L*M,ny);
ref_vector = zeros(L*M,4);

% initial state
x = x0_quadcopter;

% solve the Discrete time Riccati Difference Equation backwards in time

sol = zeros(M,nx*nx);
for k=M:-1:1
    sol(k,:) = reshape(S,1,nx*nx); %store S_k+1 for iteration k
    S = riccati_diffeq(S,A_d,B_d,Q,R);
end

for k=1:L
    for m=1:M
        r = refs_ss(k,:)'; % reference
        S = reshape(sol(m,:),nx,nx);
        K = (R+B_d'*S*B_d)\B_d'*S*A_d;
        u = -K*(x-N_x*r)+N_u*r;
        y = C_d*x;
        [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x);
        x = X(end,:)';
        %x = ffun2([x;u+u_eq*ones(nu,1)]);
        U_vector((k-1)*M+m,:) = u';
        X_vector((k-1)*M+m,:) = x';
        Y_vector((k-1)*M+m,:) = y';
        ref_vector((k-1)*M+m,:)=r';
    end 
end

T=T_s*(0:M*L-1);

figure
plot(T,ref_vector,'--');
hold on
plot(T,Y_vector(:,1:3));
legend({'x_{ref}','y_{ref}','z_{ref}','x','y','z'},'FontSize',18);
title('Resulting outputs');
xlabel('T [s]')

figure
plot3(Y_vector(:,1),Y_vector(:,2),Y_vector(:,3));
hold on
for k=1:L
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