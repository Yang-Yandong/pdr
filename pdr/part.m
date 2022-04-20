function [E_part ,N_part] = part(E_aft,N_aft,length)
%% 参数设置
N0 = N_aft(1);   %起始点北方向坐标
E0 = E_aft(1);   %起始点东方向坐标
t = 1; %时间间隔设为1s
X0 = [N0;E0;0;0];%滤波初值
random = rand;
Q = [0.1;0.1]; % process noise covariance 过程噪声方差
R = [1 ; 1]; 
% measurement noise covarianc  测量噪声方差
A = [1 0 t 0;0 1 0 t;0 0 1 0;0 0 0 1]; %表示状态方程中的fai
num = 400; % number of particles in the particle filter颗粒过滤器中的粒子数
P = [1;1;0.1;0.1];
tao = [0.5*t*t,0;0,0.5*t*t;t,0;0,t]; %系统噪声系数阵
xhat = X0;     %xhat=x=0.1
xhatPart = X0;    %xhatPart=x=0.1
q_med_arr = 0;
%% 计算
% figure(80)
% plot(E_aft,N_aft);  
% hold on
% Initialize the particle filter. 初始化粒子滤波，xpart值用来在不同时刻生成粒子
for i = 1 : num
    r = [randn,randn,randn,randn];
    random1 = diag(r);
    X_part(:,i) = X0 + random1 * sqrt(P);   %  randn产生标准正态分布的随机数或矩阵的函数。
end      %初始化xpart(i)为生成的100个随机粒子

% xArr = [X0];    %xArr=x=0.1
xhatPartArr = [xhatPart];   %xhatPartArr = [xhatPart]=0.1
% close all;

for k = 1:length
    Z = [N_aft(k);E_aft(k)];
    for i = 1 : num
        r2 = [randn,randn,randn,randn];
        random2 = diag(r2);
        X_partminus(:,i) = A*X_part(:,i)+random2*sqrt(tao*Q);
        X_partminus(:,i);
        Z_part = [X_partminus(1,i);X_partminus(2,i)];
        Z_part;
        V = Z - Z_part;
        V;
        E = X_partminus(2,i);
        N = X_partminus(1,i);
        if(((((E - 513348.0)/(N - 3787889.3)) <= (-43.5/33)&&((E - 513346.0)/(N - 3787886)) >= (-42.5/34))) ||(((N >= 3787886 && N <= 3787889.3)&&(E >= 513346 && E <= 513412))) ||(((N >= 3787886 && N <= 3787930)&&(E >= 513412 && E <= 513415))))
            for j = 1:2
                q(j,i) = (1 / sqrt(R(j)) / sqrt(2*pi)) * exp(-V(j)^2 / 2 / R(j)); %根据差值给出100个粒子对应的权重
                q;
            end
        else
            for j = 1:2
                q(j,i) = 0; %根据差值给出100个粒子对应的权重
                q;
            end
        end
    end
    
    for j = 1:2
        j;
        qsum = sum(q(j,:));
        for i = 1:num
            q(j,i) =  q(j,i)/qsum;
        end
    end
    for i = 1 : num
        q_med(i) = median(q(:,i));
        q_med(i);
    end
    
    for i = 1 : num
        u = rand; % uniform random number between 0 and 1      0和1之间的均匀随机数
        qtempsum = 0;
        for k = 1 : num
            qtempsum = qtempsum + q_med(k);
            if qtempsum >= u
                q_med_arr = [q_med_arr q_med(k)];
                %重采样对低权重进行剔除，同时保留高权重，防止退化的办法
                X_part(:,i) = X_partminus(:,k);
%            figure(39)
%            hold on     %%%
%            plot(X_part(2,i),X_part(1,i))%%%
                break;
            end
        end
    end
    
    % The particle filter estimate is the mean of the particles.      粒子滤波的估计是颗粒的平均值
    for j = 1 : 2
    xhatPart(j) = mean(X_part(j,:)); %经过粒子滤波处理后的均值
    end
    
%     xArr = [xArr X];
    xhatPart;

    xhatPartArr = [xhatPartArr xhatPart];
end
%% 画图
    N_part = xhatPartArr(1,:);
    E_part = xhatPartArr(2,:);
% % % % % % % % % % % % % % % %     
%   figure(60)
%   plot(E_part,N_part,'r')
%   hold on   
    
%  N_mid=zeros(1,106);
%  E_mid=zeros(1,106);
%  N_mid(1)=3787917.5;
%  E_mid(1)=513308;
%  del_s_part_sum = 0;
% 
% %% 坐标1
% for i = 1:38
%     N_mid(i+1) = N_mid(i)-15/19;
%     E_mid(i+1) = E_mid(i)+81/76;
% end
% 
% for i = 39:81
%     N_mid(i+1) = N_mid(i);
%     E_mid(i+1) = E_mid(i)+65/43;
% end
% 
% for i = 82:105
%     N_mid(i+1) = N_mid(i)+1.62;
%     E_mid(i+1) = E_mid(i);
%  end
% % %% 坐标2
% [N_mid,E_mid] = Real_Coordinate;

% for i = 1:106
% del_N_part(i) = N_part(i)-N_mid(i);
%  del_E_part(i) = E_part(i)-E_mid(i);
% del_s_part(i) = sqrt(del_N_part(i)^2+del_E_part(i)^2);
% del_s_part_sum = del_s_part_sum + del_s_part(i)^2;
% end
% % %% 计算
% 
% figure(49)
%  plot(del_s_part)
% 
 %del_s_part_sum = sqrt(del_s_part_sum/106);
% del_s_part_sum;
 %mean(del_s_part);
% max(del_s_part)
% 
% %% 画边框
%  hold on
% 
%  N=[3787920,3787886,3787886,3787895.5,3787895.5,3787930,3787930,3787889.3,3787889.3,3787922.3,3787920];
%  E=[513303.5,513346.5,513418,513418,513415,513415,513412,513412,513348.0,513304.5,513303.5];
%  plot(E,N,'b')