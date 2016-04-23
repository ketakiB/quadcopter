function Pdot = riccatiODE(t,P,A,B,Q,R,nx)
P = reshape(P,nx,nx);
Pdot = -(P*A + A'*P - P*B*(R\B'*P) + Q);
Pdot = reshape(Pdot,nx*nx,1);
end