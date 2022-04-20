%  function [stp_ori, pk_pos, vy_pos, NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw, gyr0, NE_ini)
function [stp_ori,STP_ORI, pk_pos, vy_pos,NE] = IMU_firls_stp_ori_loc(stp_num, stp_len, py_flg, yaw,NE_ini)
%{
%   INPUTS
%       stp_num       :  stp_num_detection()��������õ��Ĳ���
%       py_flg        :  stp_num_detection()������¼�Ĳ��岨�ȷ�ֵ py_flg = [pk_flg, vy_flg]
%       yaw           :  ��λ��������
%       gyrz          �� ���� Z ��������

%   OUTPUTS
%       stp_len      �� ���Ƶõ��Ĳ���
%       Distance     �� ���ƵĲ����ۼ�ֵ
%}

deg2rad = pi/180;

pk_pos  = find(py_flg(:,1)~=0);  % pos1 ��ʾ����λ��
vy_pos  = find(py_flg(:,2)~=0);  % pos1 ��ʾ����λ��
dlt_pos = pk_pos(1)-vy_pos(1);

stp_ori = zeros(stp_num,1);

myaw = zeros(stp_num,1);
%sum_gyr = zeros(stp_num-1,1);
sum_gyr = zeros(stp_num,1);

N = zeros(stp_num, 1);
E = zeros(stp_num, 1);

N(1) = NE_ini(1);
E(1) = NE_ini(2);
% % % % % % % % % % % % % % % % % % % % % % % % % % % 

for i = 1:stp_num

    if dlt_pos > 0
        myaw(i) = median(yaw(vy_pos(i):pk_pos(i)));%�������ֵ
%          sum_gyr(i) = sum(gyrz(vy_pos(i)+1:pk_pos(i)).*diff(spl_time(vy_pos(i):pk_pos(i))));
    else
        myaw(i) = median(yaw(pk_pos(i):vy_pos(i)));
%          sum_gyr(i) = sum(gyrz(pk_pos(i)+1:vy_pos(i)).*diff(spl_time(pk_pos(i):vy_pos(i))));
    end
% % % % % % % % % % % % % % % % % % % % % % % % % % %   
%     %% ֻ�ô�����
    if i == 1
        stp_ori(i) = myaw(i);
      N(i) = N(1) +  stp_len(i) * cos(stp_ori(i));
       E(i) = E(1) +  stp_len(i) * sin(stp_ori(i));
    else
        % ����һ����ֵ delta_m = abs(myaw(i)-myaw(i-1)) = 10*deg2rad;
        delta_m = abs(myaw(i)-myaw(i-1));
        if delta_m < 5*deg2rad  %0.087
           stp_ori(i) = stp_ori(i-1); 
       else
         stp_ori(i) = myaw(i);
        end
        
        N(i) = N(i-1) +  stp_len(i) * cos(stp_ori(i));
        E(i) = E(i-1) +  stp_len(i) * sin(stp_ori(i));
    end
    
  STP_ORI(i) = stp_ori(i)/deg2rad;
   
% ��һ���ô����ƣ�֮��������
   
%     if i == 1
%       stp_ori(i) = myaw(i);
%       N(i) = N(1) +  stp_len(i) * cos(stp_ori(i));
%       E(i) = E(1) +  stp_len(i) * sin(stp_ori(i));
%      else
%        stp_ori(i) = stp_ori(i-1)+sum_gyr(i)* deg2rad;
%        N(i) = N(i-1) +  stp_len(i) * cos(stp_ori(i));
%        E(i) = E(i-1) +  stp_len(i) * sin(stp_ori(i));
%     end
%     STP_ORI(i) = stp_ori(i)/deg2rad;
      
end
%  figure
% plot(E, N ,'g')
NE = [N, E];

%  for i = 1:stp_num
%     if dlt_pos > 0
%         myaw(i) = mean(yaw(vy_pos(i):pk_pos(i)));
%     else
%         myaw(i) = mean(yaw(pk_pos(i):vy_pos(i)));
%      end
%     if i == 1
%         stp_ori(i) = myaw(i);
%     else
%         sum_gyr(i-1) = sum(gyr_z(pk_pos(i-1)+1:pk_pos(i)).*diff(spl_time(pk_pos(i-1):pk_pos(i))));
%         gyr_z = mod(sum_gyr(i-1),2/pi);
%      %   if gyr > 10   % 10 ��
%         if abs(sum_gyr(i-1)) >= 10*deg2rad   % 10 ��
%             stp_ori(i) = myaw(i);
%          else
%             stp_ori(i) = stp_ori(i-1);
%         end
%     end
%     end

%  figure(20)
%  plot(1:stp_num,stp_ori/deg2rad,'b-*')
% title('stp_ori');
 figure(21)
plot(1:stp_num,myaw/deg2rad,'r--')
 title('myaw');
 xlabel('����')
 ylabel('����')
 grid on;
% figure(22)
%  plot(1:stp_num,sum_gyr*deg2rad,'m-*')
% title('sum_gyr');

%  figure(23)
%  plot(1:stp_num-1,sum_gyr,'b-*',1:stp_num-1,10*deg2rad,'r-+',1:stp_num-1,-10*deg2rad,'r-+')



