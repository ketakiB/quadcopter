T_s = 0.05; % Sampling time

% Discretization using Bilinear Transformation

M = inv(eye(nx) - A*T_s/2);

A_d = M*(eye(nx) + A*T_s/2);
B_d = M*B*T_s;
C_d = C*M;
D_d = D + C_d*B*T_s/2;

% Creating the discrete time system
sys = ss(A_d,B_d,C_d,D_d);

%************************
%Checking the stability
%************************
disp('Poles:')
disp(eig (A_d))

% Plotting the location of the poles
pzmap(sys);

% Step response
figure;
step(sys,Tmax)

figure;
impulse(sys,Tmax)

%****************************************************
% Checking whether the system is controlable or not.
%****************************************************

disp ('Controllability matrix');

CO = ctrb(A_d,B_d);

disp('Rank of the controllability matrix:');
rank(CO)

%****************************************************
% Checking whether the system is observable or not.
%****************************************************

disp ('Observability matrix');

OB = obsv(A_d,C_d);

disp('Rank of the observability matrix:');
rank(OB)


