S=Q;
K=inv(R+B_d'*S*B_d)*B_d'*S*A_d;
diff_s = zeros(M,1);
diff_k = zeros(M,1);
for k=M:-1:1
    K_old = K;
    S_old = S;
    S = riccati_diffeq(S,A_d,B_d,Q,R);
    K = inv(R+B_d'*S*B_d)*B_d'*S*A_d;
    diff_k(k) = norm(K-K_old);
    diff_s(k) = norm(S-S_old);
end

f1=figure()
plot(diff_k);
xlabel('Time step k','FontSize',13)
ylabel('||K_k - K_{k-1}||','FontSize',13)

set(f1,'PaperPositionMode','auto');
print(f1,'riccati_k','-dpng','-r300');

f2=figure()
plot(diff_s);
xlabel('Time step k','FontSize',13)
ylabel('||S_k - S_{k-1}||','FontSize',13) 

set(f2,'PaperPositionMode','auto');
print(f2,'riccati_s','-dpng','-r300');
