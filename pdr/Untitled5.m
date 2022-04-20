clear all;clc;close all;
ori0 = load('C:\Users\Administrator\Desktop\消防员定位项目\项目代码\绕九教走一圈\数据\ang.txt');
gyr0 = load('C:\Users\Administrator\Desktop\消防员定位项目\项目代码\绕九教走一圈\数据\gyr.txt');
yaw = ori0(:,3);
gyr_z = gyr0(:,2) *pi/180;
th = 1;                 %%设置转角的阈值

    %%解航向角的缠绕
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

%%根据陀螺仪定转角位置
[ L L0] = corner_detect(gyr_z,th);

%%利用地图匹配的角度对yaw进行滤波
yaw_kal = yaw_kal(yaw,gyr_z,L0);
yaw_ = yaw_kal
figure 
plot(yaw_)