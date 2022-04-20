function [stp_len, dif_acc, s2] = IMU_firls_stp_len(stp_num, flg,acc)

%{
%   INPUTS
%       stp_num       :  stp_num_detection()函数计算得到的步数
%       py_flg        :  stp_num_detection()函数获取的波峰波谷幅值

%   OUTPUTS
%       stp_len      ： 估计得到的步长
%       Distance     ： 估计的步长累加值
%}

stp_len    = zeros(stp_num,1);
dif_acc    = zeros(stp_num,1);
pos1 = find(flg(:,1)~=0);  % pos1 表示波峰位置
pos2 = find(flg(:,2)~=0);  % pos1 表示波谷位置
sumpk = 0;
distance = 0;
% % % % % % % % % % % % % % 
for i = 1:length(pos1)
        sumpk = sumpk + flg(pos1(i),1);  %sumpk 波峰幅值总和
end
mPk = sumpk / length(pos1);    
% K = 0.087 * mPk + 0.175
K = 0.087 * mPk + 0.5;%mPk为波峰平均数值
% K = 0.47;
% % % % % % % % % % % % % % 
for i = 1:stp_num
    sum = 0;
    for j = pos2(i):pos1(i)        %这里要注意先出现的是波峰还是波谷
       sum = sum +acc(j); %单步中采样点的合加速度
    end
    mAcc(i) = sum/(pos1(i)-pos2(i));%pos1(i)-pos2(i)是指单步加速度采样点个数
end
% % % % % % % % % % % % % % %  
for i = 1:stp_num
    dif_acc(i) = abs(flg(pos1(i),1) - flg(pos2(i),2));  %amax - amin
%     K = 0.087 * dif_acc(i)/2 + 0.25;
    stp_len(i) = K * ((dif_acc(i)-1.5)^(1/4));
%      stp_len(i) = K * ((dif_acc(i))^(1/4));
   % stp_len1(i) = 0.8;
    stp_len2(i) = K * (mAcc(i)-flg(pos2(i),2))/dif_acc(i);
    stp_len3(i) = K * mAcc(i).^(1/3);
   % stp_len4(i)=K*(dif_acc(i)*3.5+(dif_acc(i))^(1/4));
    distance = distance + stp_len2(i);
end
% % % % % % % % % % 
    s2 = distance;
% % % % % % % % % % 
figure 
plot(1:stp_num,stp_len,'r--',1:stp_num,stp_len2,'b--',1:stp_num,stp_len3,'k--')

xlabel('步伐数');
ylabel('步长');
legend('Weinberg approach','Scarlet approach','Kim approach');

%% 计算距离    假设是直线运动
 %figure
 %plot(1:stp_num,stp_len,'r--')
 %title('步长');
 %figure
 %plot(1:stp_num,stp_len2,'b--')
 %title('步长');
 %figure
 %plot(1:stp_num,stp_len3,'k--')
 %title('步长');
% Distance = sum(stp_len); 