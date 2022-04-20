%%yaw kalman filter
function [yaw_kal] = yaw_kal2(yaw,gyr_z,L0)
    Q=[0.0025 0;0 0.01];R=[0.005];
    R = diag(R);
    A=[1 1;0 1];C=[1 0];X =zeros(2,1);P=zeros(2,2);I=[1 0;0 1];
    w = [0.04;0.01];
    P=[sqrt(var(yaw)) 0;0 sqrt(var(gyr_z))];X=[128.5;gyr_z(1)];
    len = length(yaw);
    yaw_kal = zeros(len,1);
    len1 = length(L0)
    for i =1:L0(1)
            Pi=A*P*A'+Q;%一步预测
            Xi=A*X;
            K=Pi*C'*(C*Pi*C'+R)^(-1);%增益
            X=Xi+K*([128.5]'-C*Xi);%滤波
            yaw_kal(i) = X(1,1);
            P=(I-K*C)*Pi;
            py(i)=P(1,1);
            pg(i)=P(2,2);
    end
end