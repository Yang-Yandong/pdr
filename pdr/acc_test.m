clear all;clc;close all;
%% ��������
g = 9.8; %�����������ٶ�
deg2rad = pi/180;
%% load����
acc0 = load('C:\Users\Administrator\Desktop\����Ա��λ��Ŀ\��Ŀ����\�ƾŽ���һȦ\����\acc.txt');
% gyr0 = load('gyr.txt');
ori0 = load('C:\Users\Administrator\Desktop\����Ա��λ��Ŀ\��Ŀ����\�ƾŽ���һȦ\����\ang.txt');%�Ƕ�
% acc0 = acc0(:,1:3); %ȡacc0��ǰ����ֵ
% gyr0 = gyr0(:,1:3);
% ori0 = ori0(:,1:3);
% gyr_x = gyr0(:,1);
% gyr_y = gyr0(:,2);
% gyr_z=gyr0(:,3);
roll  = ori0(:,1) * deg2rad; %�����
pitch = ori0(:,2) * deg2rad; %������
yaw   = ori0(:,3) * deg2rad; %�����
%% ��ԭʼ���ٶ�Acc����ͼ
% figure(1)
% subplot(311)
% acc_g=acc0/g; %���ٶ�/g
% plot(1000:1500,acc_g(1000:1500,1:3))
% legend('x','y','z');
% title('ԭʼ���ٶ�/g');
% 
% subplot(312)
% plot(1000:1500,acc_g(1000:1500,3));
% title('Z����ٶ�/g');
% 
% acc_g_he=sqrt(acc_g(:,1).^2+acc_g(:,2).^2+acc_g(:,3).^2);
% subplot(313)
% plot(1000:1500,acc_g_he(1000:1500));
% title('�ϼ��ٶ�/g');
% xlabel('������');
% %% ���ٶȲ���ƽ��
% acc_g_he_0 = acc_g_he - mean(acc_g_he); %ȥ������
% figure(2)
% plot(1000:1500,acc_g_he_0(1000:1500));
% title('ȥ���������ٶ�/g');
% 
% % ��Ƶ�ͨ�˲���
% 
% fs = 50; % Hz����Ƶ�� 
% n = 10; % �˲�������
% f = [0 2 3 50]/fs; % ��һ��Ƶ������,Ƶ��ϵ��
% a = [1 1 0 0]; % ����������Ŵ�����
% b = firls(n,f,a); % ����firls���fir�˲���
% [h,w] = freqz(b); % ������Ƶ����Ӧ
% 
% % ���ϼ��ٶ�ͨ����ͨ�˲���
% y_acc = filter(b,1,acc_g_he_0);
% figure(3)
% plot(y_acc(300:1000));
% title('��ͨ�˲���ĺϼ��ٶ��ٶ�');
% %% ����̽��  ��ֵ̽��
% ap = 0.05;
% [pk_num, vy_num, stp_num, pk_ti, vy_ti, py_flg] = IMU_firls_stp_num(y_acc, ap);
% 
% %% ���㲽��
% [stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, py_flg,acc_g_he_0);

%% ����Ǵ���
figure

plot(yaw/deg2rad,'b');
title('У��ǰ');

%У��
yaw2 = map_aid(ori0)*deg2rad;
figure
plot(yaw2/deg2rad,'b');
title('У����');

%% λ�ü���
NE_ini(1,:) = [0,0];
[stp_ori, STP_ORI, pk_pos, vy_pos,NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, NE_ini);
len = length(NE(:,1));
[E_part, N_part] = part(NE(:,2),NE(:,1),len);
figure(5)
plot(NE(:,2),NE(:,1),'g-')
title('���˺�������PDR�㷨')
