%步频探测
% INPUT
%   acc_g_he 合加速度

% OUTPUT
%   pk_num 波峰位置
%   vy_num 波谷位置
%   stp_num 步数

% function [pk_num,vy_num,stp_num] = Stride_detection(acc_g_he);
function [a] = Stride_detection(acc);
N=2;
len_acc = length(acc);
for i = 1:N
    a0(i) = acc(i);
    a0(len_acc - i+1) = acc(len_acc - i+1);
end
for i = N+1:len_acc - N
    a0(i-N) = acc(i-N); 
    for jj = i-N:i+N-1
        a0(jj) = a0(jj) + acc(jj+1);
    end
    a0(i) = a0(i) / (2*N+1);
end
figure
plot(1000:1500,a0(1000:1500))