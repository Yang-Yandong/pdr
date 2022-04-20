clear all;clc;close all;
%������Ԫ����ȡ��̬��
% NE_ini(1,:) = [464174.1103  520739.1111];
% NE_ini(1,:) = [3787920,513303.5];
% NE_ini(1,:) = [0 0];
% NE_ini(1,:) = [3787930 513415];
% % % % % % % % % % % % % % % % 
% NE_ini(1,:) = [3787921,513304];
NE_ini(1,:) = [40,0];
delta_yaw1=0; %��ѧ �ö���yaw
steptime = 0.2;


deg2rad = pi/180;
g0 = [0;0;9.7285];

acc00 = load('acc.txt');
gyr00 = load('gyr.txt');
ori00 = load('ori.txt');%�Ƕ�
% acc01 = load('acc1.txt');
% gyr01 = load('gyr1.txt');
% ori01 = load('ori1.txt');%�Ƕ�
acc0 = acc00;
gyr0 = gyr00;
ori0 = ori00;

%coo = load('coo.txt');
acc0 = acc0(:,1:3); %ȡacc0��ǰ����ֵ
gyr0 = gyr0(:,1:3);
ori0 = ori0(:,1:3);
% acc0 = acc0(1:1400,1:3);
% gyr0 = gyr0(1:1400,1:3);
% ori0 = ori0(1:1400,1:3);
% 
roll  = ori0(:,2) * deg2rad; %�����
pitch = ori0(:,3) * deg2rad; %������
yaw   = ori0(:,1) * deg2rad; %�����
 figure(55)
 plot(gyr0(:,3)/deg2rad);
 title('������Z��');
xlabel('���ٶ���');
ylabel('����');

figure(12)
plot(yaw/deg2rad,'b');
legend('�����');
title('У��ǰ');
grid on;
gyr_x = gyr0(:,1);gyr_y = gyr0(:,2);gyr_z=gyr0(:,3);
len_eul = length(ori0(:,1));
g       = zeros(len_eul   ,3); % gravity affection on acc������acc��Ӱ��
a_xyz   = zeros(len_eul   ,3); % acc remove g
x       = zeros(len_eul-2 ,3); % coordinate ����
%%  remove gravity from acc sensorsɾ��acc�д�����������
for i = 1:len_eul
    eul_vect(1) =  roll(i) ; %roll(i) ��i�к����
    eul_vect(2) = pitch(i) ; %������
    eul_vect(3) =   yaw(i) ; %�����
    DCM_n2b = eulr2dcm(eul_vect);   %https://blog.csdn.net/qq_36828395/article/details/86761790   �����ŷ���ǲ��������Ǹú��������룬����õ�һ��3*3�ķ������Ҿ����ṩ�Ӷ�����ϵ����������ϵ�ı任����
    C = DCM_n2b;
    g(i,:) = C * g0;%����MEMS�������ֻ��������̵����ڶ�λP50��(3.10)   %�ɶ�����ϵ�任����������ϵ
     a_xyz(i,:) = acc0(i,:) - g(i,:);   %��������ϵ���ٶ�ɾ���������ٶ�
%     a_xyz(i,:) = acc0(i,:) + g(i,:);  
    a_ned(i,:) = C' * a_xyz(i,:)';   %������ȥ��������ļ��ٶ�ת����������ϵ���ٶ�
end
figure(1000)
plot(a_ned(:,1))
title('................')
acc = acc0(:,3)-mean(acc0(:,3));
figure(35)
plot(acc);
grid on;
% Test
fs = 50; % Hz����Ƶ�� 
n = 10; % �˲�������
f = [0 2 3 50]/fs; % ��һ��Ƶ������,Ƶ��ϵ��
a = [1 1 0 0]; % ����������Ŵ�����
b = firls(n,f,a); % ����firls���fir�˲���
[h,w] = freqz(b); % ������Ƶ����Ӧ
%

figure(1)
plot(w/pi,abs(h),'r')%������Ƶ��Ӧ����
xlabel('��һ��Ƶ��');ylabel('���');
legend('firls'); % ����ͼ��
grid on;
%
figure(19)%acc�˵�����ͼ
plot(a_xyz(:,1:3));
xlabel('���ٶ���')
ylabel('���� [m/s^2]')
title('���ٶ�ȥ������Ӱ��')
legend('x-axis','y-axis','z-axis')
box on
grid on
%
figure(11)%accԭX\Y\Z������ͼ
plot(acc0(1000:1500,1:3));
xlabel('samples')
ylabel('���� [m/s^2]')
title('ԭʼδ����ļ��ٶ��ź�[g]')
legend('x-axis','y-axis','z-axis')
box on
grid on
%
% % % % % % % % % % % % %
len = length(a_xyz(:,1));
a_module = zeros(len,1); %a_module �ϼ��ٶ�
for i = 1:len
   a_module(i) = sqrt(a_xyz(i,1)^2+a_xyz(i,2)^2+a_xyz(i,3)^2);
end
figure (50)
plot(a_module(1000:1500,1));
xlabel('samples')
ylabel('����[m/s^2]')
title('ʸ���ͼ��ٶ��ź�')
% 
% % % % % % % % % % % % % % % % % % % % 
figure(2) 
subplot(1,2,1);
plot(acc(1000:1500,:));
xlabel('samples')
ylabel('����[m/s^2]')
title('ԭʼ�ź�')
subplot(1,2,2);
 ACC=fft(acc(1000:1500,:));
 plot(abs(ACC));
title('ԭʼ�ź�Ƶ��');
% 
% y = filter(b,1,a_module);
y = filter(b,1,acc);
% y(6512)=-1.65;
% y(6533)=2.1;
% y(6534)=1.7;
% y(6906) =  0.56;
figure(3)% plot(y(1200:1550))
% subplot(1,2,1);
plot(y(1000:1500,:))
xlabel('samples')
ylabel('����[m/s^2]')
title('��ͨ�˲��ź�')
% subplot(1,2,2);
% Y=fft(y(4000:5000,:));
% plot(abs(Y));
% title('FIR�˲����Ƶ��');
% % % % % % % % % % % 
% figure(37)
% plot(y);
%
figure(30)%
plot(acc(1000:1500))
xlabel('samples')
ylabel('����[m/s^2]')
title('ʸ���ͼ��ٶ��ź�')
%

yaw = map_aid(ori0,gyr0)*deg2rad; %�����˲�
% for i =1:2582
 %    yaw(i) = 128.5*deg2rad;
 %end
% 
 %for i = 2640:5196
  %   yaw(i) = 90*deg2rad;
 %end
% 
 %for i  = 5270:7031
  %   yaw(i) = 0;
 %end
 figure(13)
plot(yaw/deg2rad,'b');
xlabel('���ٶ���')
legend('�����')
title('У����');
grid on;
%% step number detection
ap = 0.5;
[pk_num, vy_num, stp_num, pk_ti, vy_ti, py_flg] = IMU_firls_stp_num(y, ap);
%% step length detection
[stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, py_flg,acc);
% [stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, py_flg,a_xyz);
%% step orientation estimation and location
% [stp_ori, pk_pos, vy_pos,NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, gyr0,spl_time, NE_ini);
[stp_ori, STP_ORI, pk_pos, vy_pos,NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, NE_ini);
len = length(NE(:,1));
[E_part, N_part] = part(NE(:,2),NE(:,1),len);
%N=[3787920,3787886,3787886,3787930,3787930,3787888,3787888,3787922.3,3787920];
%E=[513303.5,513346.5,513415,513415,513412,513412,513348.0,513304.5,513303.5];
N=[40,6,6,49];
E=[0,42.5,110,110];
figure;
plot(NE(:,2),NE(:,1),'g-')
hold on
plot(E,N,'b-');
% hold on
% plot(E_part,N_part,'r-')
%hold on
%plot(coo(:,2)/100+500000+70.3,coo(:,1)/100+3700000-46.5,'k--')
legend('���˺�λ����PDR�㷨','��ʵ�켣')
xlabel('��[m]')
ylabel('��[m]')
title('��άƽ������ϵ')
 %hold on
 %plot(NN,EE,'r')
% ���10����ȡһ��ֵ
 %acc = acc(1:10:end);
 %figure
 %plot(acc)
 

