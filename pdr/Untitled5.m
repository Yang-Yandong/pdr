clear all;clc;close all;
ori0 = load('C:\Users\Administrator\Desktop\����Ա��λ��Ŀ\��Ŀ����\�ƾŽ���һȦ\����\ang.txt');
gyr0 = load('C:\Users\Administrator\Desktop\����Ա��λ��Ŀ\��Ŀ����\�ƾŽ���һȦ\����\gyr.txt');
yaw = ori0(:,3);
gyr_z = gyr0(:,2) *pi/180;
th = 1;                 %%����ת�ǵ���ֵ

    %%�⺽��ǵĲ���
    for i = 1:size(yaw)-1
        if(yaw(i+1) - yaw(i)<-180)
            yaw(i+1) = yaw(i+1) + 360;
        else if(yaw(i+1) - yaw(i)>180)
                yaw(i+1) = yaw(i+1) - 360;  
            end
        end
    end
figure
plot(yaw)

%%���������Ƕ�ת��λ��
[ L L0] = corner_detect(gyr_z,th);

%%���õ�ͼƥ��ĽǶȶ�yaw�����˲�
yaw_kal = yaw_kal(yaw,gyr_z,L0);
yaw_ = yaw_kal
figure 
plot(yaw_)