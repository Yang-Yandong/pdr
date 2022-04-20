function [yaw_] = map_aid(ori0,gyr0)
    %%基于地图匹配的方向滤波
    % ori0 = load('ori.txt');
    % gyr0 = load('gyr.txt');
    yaw = ori0(:,1);
    gyr_z = gyr0(:,3);
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
%      figure
%      plot(yaw)

    %%根据陀螺仪定转角位置
    [ L, L0] = corner_detect(gyr_z,th);

    %%利用地图匹配的角度对yaw进行滤波
    yaw_kal = yaw_kal1(yaw,gyr_z,L0);
    yaw_ = yaw_kal;
end