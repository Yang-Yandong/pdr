function [yaw_] = map_aid(ori0,gyr0)
    %%���ڵ�ͼƥ��ķ����˲�
    % ori0 = load('ori.txt');
    % gyr0 = load('gyr.txt');
    yaw = ori0(:,1);
    gyr_z = gyr0(:,3);
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
%      figure
%      plot(yaw)

    %%���������Ƕ�ת��λ��
    [ L, L0] = corner_detect(gyr_z,th);

    %%���õ�ͼƥ��ĽǶȶ�yaw�����˲�
    yaw_kal = yaw_kal1(yaw,gyr_z,L0);
    yaw_ = yaw_kal;
end