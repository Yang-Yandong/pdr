clear all;clc;close all;
%% 参数设置
g = 9.8; %设置重力加速度
deg2rad = pi/180;
%% load数据
acc0 = load('C:\Users\Administrator\Desktop\消防员定位项目\项目代码\绕九教走一圈\数据\acc.txt');
% gyr0 = load('gyr.txt');
ori0 = load('C:\Users\Administrator\Desktop\消防员定位项目\项目代码\绕九教走一圈\数据\ang.txt');%角度
% acc0 = acc0(:,1:3); %取acc0的前三列值
% gyr0 = gyr0(:,1:3);
% ori0 = ori0(:,1:3);
% gyr_x = gyr0(:,1);
% gyr_y = gyr0(:,2);
% gyr_z=gyr0(:,3);
roll  = ori0(:,1) * deg2rad; %横滚角
pitch = ori0(:,2) * deg2rad; %俯仰角
yaw   = ori0(:,3) * deg2rad; %航向角
%% 画原始加速度Acc数据图
% figure(1)
% subplot(311)
% acc_g=acc0/g; %加速度/g
% plot(1000:1500,acc_g(1000:1500,1:3))
% legend('x','y','z');
% title('原始加速度/g');
% 
% subplot(312)
% plot(1000:1500,acc_g(1000:1500,3));
% title('Z轴加速度/g');
% 
% acc_g_he=sqrt(acc_g(:,1).^2+acc_g(:,2).^2+acc_g(:,3).^2);
% subplot(313)
% plot(1000:1500,acc_g_he(1000:1500));
% title('合加速度/g');
% xlabel('采样点');
% %% 加速度波形平滑
% acc_g_he_0 = acc_g_he - mean(acc_g_he); %去除重力
% figure(2)
% plot(1000:1500,acc_g_he_0(1000:1500));
% title('去除重力加速度/g');
% 
% % 设计低通滤波器
% 
% fs = 50; % Hz采样频率 
% n = 10; % 滤波器阶数
% f = [0 2 3 50]/fs; % 归一化频率向量,频率系数
% a = [1 1 0 0]; % 振幅向量，放大特性
% b = firls(n,f,a); % 采用firls设计fir滤波器
% [h,w] = freqz(b); % 计算其频率响应
% 
% % 将合加速度通过低通滤波器
% y_acc = filter(b,1,acc_g_he_0);
% figure(3)
% plot(y_acc(300:1000));
% title('低通滤波后的合加速度速度');
% %% 步数探测  峰值探测
% ap = 0.05;
% [pk_num, vy_num, stp_num, pk_ti, vy_ti, py_flg] = IMU_firls_stp_num(y_acc, ap);
% 
% %% 计算步长
% [stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, py_flg,acc_g_he_0);

%% 航向角处理
figure

plot(yaw/deg2rad,'b');
title('校正前');

%校正
yaw2 = map_aid(ori0)*deg2rad;
figure
plot(yaw2/deg2rad,'b');
title('校正后');

%% 位置计算
NE_ini(1,:) = [0,0];
[stp_ori, STP_ORI, pk_pos, vy_pos,NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, NE_ini);
len = length(NE(:,1));
[E_part, N_part] = part(NE(:,2),NE(:,1),len);
figure(5)
plot(NE(:,2),NE(:,1),'g-')
title('行人航迹推算PDR算法')
