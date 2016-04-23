function [ S ] = riccati_diffeq( Splus, A, B, Q, R )
%RICCATI_DIFFEQ Summary of this function goes here
%   Detailed explanation goes here

Pplus = R+B'*Splus*B;
S = A'*(Splus - Splus*B*inv(Pplus)*B'*Splus)*A + Q;

end

