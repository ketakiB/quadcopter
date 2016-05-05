function [ nextstate ] = ffun( stateinput )
%F Summary of this function goes here
%   state = [x; y; z; v_x; v_y; v_z; phi; theta; psi; w_x; w_y; w_z]
state = stateinput(1:12);
input = stateinput(13:16);
x = state(1); y= state(2); z=state(3); v_x =state(4); v_y=state(5);
v_z=state(6); phi = state(7); theta=state(8); psi=state(9); w_x=state(10);
w_y=state(11); w_z=state(12);
u1=input(1); u2=input(2); u3=input(3); u4=input(4);

m = 0.5; % + 0.1; %adding an extra mass (payload) to test robustness
L = 0.25;
k = 3 * 10^(-6);
b = 1 * 10^(-7);
g = 9.81;
k_d = 0.25;
Ixx = 5 * 10^(-3);
Iyy = 5 * 10^(-3);
Izz = 1 * 10^(-2);
c_m = 1 * 10^(4);

nextstate = zeros(size(state));

nextstate(1) =  v_x;
nextstate(2) = v_y;
nextstate(3) = v_z;

nextstate(4) = -k_d/m * v_x + k*c_m/m *(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*(u1+u2+u3+u4);
nextstate(5) = -k_d/m * v_y + k*c_m/m *(cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi))*(u1+u2+u3+u4);
nextstate(6) = -k_d/m * v_z - g + k*c_m/m *(cos(theta)*cos(phi))*(u1+u2+u3+u4);

nextstate(7) = w_x + w_y*(sin(phi)*tan(theta)) + w_z*(cos(phi)*tan(theta));
nextstate(8) = w_y*cos(phi) - w_z*sin(phi);
nextstate(9) = sin(phi)/cos(theta) *w_y + cos(phi)/cos(theta) * w_z;

nextstate(10) = L*k*c_m/Ixx * (u1-u3) - (Iyy-Izz)/Ixx * w_y*w_z;
nextstate(11) = L*k*c_m/Iyy * (u2-u4) - (Izz-Ixx)/Iyy * w_x*w_z;
nextstate(12) = b*c_m/Izz * (u1-u2+u3-u4) - (Ixx-Iyy)/Izz * w_y*w_x;

end

