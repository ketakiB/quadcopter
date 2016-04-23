T_s = 0.05; % Sampling time



M = inv(eye(nx) - A*T_s/2);

%[A_d,B_d,C_d,D_d]=c2dm(A,B,C,D,T_s,'tustin'); % Discretization using Bilinear Transformation
[A_d,B_d,C_d,D_d]=c2dm(A,B,C,D,T_s,'zoh'); %Discretization using zero order hold

% Creating the discrete time system
sys = ss(A_d,B_d,C_d,D_d,T_s);

%************************
%Checking the stability
%************************
disp('Poles:')
disp(eig (A_d))

% Plotting the location of the poles
%pzmap(sys);

% Step response
%figure;
%step(sys,Tmax)

%figure;
%impulse(sys,Tmax)

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


