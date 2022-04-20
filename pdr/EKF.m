clear all;clc;
%扩展卡尔曼滤波
acc = load('acc.txt');
gyr = load('gyr.txt');
ori = load('ori.txt');
%  figure
%  plot(ori(:,1))
len = length(acc(:,1));
deg2rad = pi/180;
roll  = ori(:,2) * deg2rad;
pitch = ori(:,3) * deg2rad;
yaw   = ori(:,1) * deg2rad;
X = zeros(4,1);P=zeros(4,4);
R = [sqrt(var(acc(:,1))) sqrt(var(acc(:,2))) sqrt(var(acc(:,3)))];
R = diag(R);
I=eye(4);
qua = zeros(len,4);
q = zeros(len,4);
phi = zeros(len,1);
a_module = zeros(len,1);
sigma1 = zeros(len,1);
sigma2 = zeros(len,1);
Q_gyr = [sqrt(var(gyr(:,1))) sqrt(var(gyr(:,2))) sqrt(var(gyr(:,3)))];
Q_gyr = diag(Q_gyr);

for i = 1:len
    a_module(i) = sqrt(acc(i,1)^2+acc(i,2)^2+acc(i,3)^2);
%     eul_vect(1) =  roll(i) ;
%     eul_vect(2) = pitch(i) ;
%     eul_vect(3) =   yaw(i) ;
%     DCM_n2b = eulr2dcm(eul_vect);
%     q(i,:) = dcm2qua(DCM_n2b');
end
 figure(99)
 plot(a_module);
 xlabel('samples')
ylabel('幅度[m/s^2]')
title('矢量和加速度信号')
 mean(a_module(1:100));
for i = 1:4
    P(i,i) = 0.1;
    Q(i,i) = 0.001;
end

T = 0.02; %采样频率
g = 9.7950;

eul_vect(1) =  roll(1) ;
eul_vect(2) = pitch(1) ;
eul_vect(3) =   yaw(1) ;
DCM_n2b = eulr2dcm(eul_vect)
qua(1,:) = dcm2qua(DCM_n2b');
X = qua(1,:)';
%   ex = [0 -X(4) X(3);
%            X(4) 0  -X(2);
%           -X(3) X(2) 0;];
%     w = [ex+eye(3)*X(1);
%          -X(1) -X(2) -X(3);]*T/2;
%     Q = w*Q_gyr*w';
for i =2:len-29
    del = T*sqrt(gyr(i,1)^2+gyr(i,2)^2+gyr(i,3)^2)*pi/180;
%     A = [1 -gyr(i-1,1)/2*T -gyr(i-1,2)/2*T -gyr(i-1,3)/2*T;
%          gyr(i-1,1)/2*T 1 gyr(i-1,3)/2*T -gyr(i-1,2)/2*T;
%          gyr(i-1,2)/2*T -gyr(i-1,3)/2*T 1 gyr(i-1,1)/2*T;
%          gyr(i-1,3)/2*T gyr(i-1,2)/2*T -gyr(i-1,1)/2*T 1]; %状态方程系数矩阵 
  A=[  cos(del/2) -gyr(i,1)*T*sin(del/2)/del -gyr(i,2)*T*sin(del/2)/del -gyr(i,3)*T*sin(del/2)/del;
       gyr(i,1)*T*sin(del/2)/del cos(del/2) gyr(i,3)*T*sin(del/2)/del -gyr(i,2)*T*sin(del/2)/del;
       gyr(i,2)*T*sin(del/2)/del -gyr(i,3)*T*sin(del/2)/del cos(del/2) gyr(i,1)*T*sin(del/2)/del;
       gyr(i,3)*T*sin(del/2)/del gyr(i,2)*T*sin(del/2)/del -gyr(i,1)*T*sin(del/2)/del cos(del/2)];
    Pi = A*P*A'+Q; 
    Xi = A*X;
    h = [2*g*(Xi(2)*Xi(4)-Xi(1)*Xi(3));2*g*(Xi(3)*Xi(4)+Xi(1)*Xi(2));g*(1-2*(Xi(2)^2+Xi(3)^2))];
%     H = [H h];
    C = [-2*g*Xi(3) 2*g*Xi(4) -2*g*Xi(1) 2*g*Xi(2);
          2*g*Xi(2) 2*g*Xi(1)   2*g*Xi(4) 2*g*Xi(3);
          0         -4*g*Xi(2) -4*g*Xi(3) 0]; 
    K=Pi*C'/(C*Pi*C'+R);  %增益
    X=Xi+K*(acc(i,1:3)'-h);      %滤波
    sigma1(i) = abs(a_module(i) - g);  
    sigma2(i) = var(a_module(i:i+29));
    sigma3(i) = sigma1(i);
   
%     R = diag([sigma3 sigma3 sigma3]);
    if sigma1(i) <2
       R = diag([sigma3 sigma3 sigma3]);
%        R = diag([0.1 0.1 0.1]);
    else
       R = diag([Inf Inf Inf]);
    end
%     R = diag([0.1 0.1 0.1]);
    P=(I-K*C)*Pi;
%      ex = [0 -X(4) X(3);
%           X(4) 0  -X(2);
%           -X(3) X(2) 0;];
%     w = [ex+eye(3)*X(1);
%          -X(1) -X(2) -X(3);]*T/2;
%     Q = w*Q_gyr*w';
    qua(i,1) = X(1);
    qua(i,2) = X(2);
    qua(i,3) = X(3);
    qua(i,4) = X(4);
end

for i =1:len
%     ori_aft(i,:) = quat2euler(qua(i,:));
    DCMbn = qua2dcm(qua(i,:));
    phi(i) = atan2(DCMbn(2,1),DCMbn(1,1));
end
%  for i =1:len
%      DCMbn = qua2dcm(q(i,:));
%      yaw(i) = atan2(DCMbn(2,1),DCMbn(1,1));
%  end
for i = 1:size(phi)-1
    if(phi(i+1) - phi(i)<-pi)
        phi(i+1) = phi(i+1) + 2*pi;
    else if(phi(i+1) - phi(i)>pi)
            phi(i+1) = phi(i+1) - 2*pi;  
        end
    end
end
figure 
plot(phi*180/pi,'b');
legend('手机电子罗盘输出数据（°）');
%%%可能是电子罗盘输出数据，详情见/基于MEMS与智能手机电子罗盘/41页