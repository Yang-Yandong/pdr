
function [pk_num, vy_num, stp_num, pk_ti, vy_ti, py_flg]=IMU_firls_stp_num(y,ap)
%{
%   INPUTS
%       acc_vtr       ： 用检测步伐的数据列 

%   OUTPUTS
%       pk_num       ： 检测到的峰值个数
%       vy_num       ： 检测到的波谷个数
%       stp_num      ： 检测到的步数
%       pk_ti        :  波峰对应的时间点(
%       vy_ti        ： 波谷对应的时间点
%       pk_fig       ： 波峰标志
%       vy_flg       :  波谷标志
%}


dlt_y = diff(y);%求导 
%  figure(29)
%  plot(dlt_y);
%  title('dlt_y');

pk_num  = 0;
vy_num  = 0;
stp_num = 0;

len_y  = length(dlt_y);
pk_flg       = zeros(len_y+1, 1);
vy_flg       = zeros(len_y+1, 1);
pk_ti        = zeros(len_y+1, 1);
vy_ti        = zeros(len_y+1, 1);
flg = 0;

for i = 2 :len_y
    if dlt_y(i-1) > 0 && dlt_y(i) < 0 && y(i) > ap % occour peak
        pk_num = pk_num + 1;
        pk_acc(pk_num) = y(i);
        pk_pos(pk_num) = i;
        pk_flg(i) = y(i);
        if pk_num == 1 && vy_num == 0
              pk_flg(i) = y(i);
            flg = 1;
        elseif (flg == 1 && (pk_num - vy_num) > 1) || (flg == -1 && (pk_num - vy_num) >= 1)
            acc_df = pk_acc(pk_num)-pk_acc(pk_num-1);
            if acc_df > 0 
                pk_num = pk_num - 1;
                pk_flg(pk_pos(pk_num)) = 0;
                 pk_flg(i) = y(i)
                pk_pos(pk_num) = i;
            else
                pk_num = pk_num - 1;
                pk_flg(i) = 0;
            end
        end
    end
    if dlt_y(i-1) < 0 && dlt_y(i) > 0 && y(i) < ap*(-1) % occour vally
        vy_num = vy_num + 1;
        vy_acc(vy_num) = y(i);
        vy_pos(vy_num) = i;
        vy_flg(i) = y(i);
        if vy_num == 1 && pk_num == 0
             vy_flg(i) = y(i);
            flg = -1;
        elseif (flg == 1 && (pk_num - vy_num) <= -1) || (flg == -1 && (pk_num - vy_num) < -1)
            acc_dfvy = vy_acc(vy_num) - vy_acc(vy_num-1);
            if acc_dfvy < 0
                vy_num = vy_num - 1;
                vy_flg(vy_pos(vy_num)) = 0;
                vy_pos(vy_num) = i;
            else
                vy_num = vy_num - 1;
                vy_flg(i) = 0;
            end
        end
    end
end
py_flg = [pk_flg, vy_flg];
py_num = [pk_num,vy_num];
stp_num = min(py_num);
figure(4)
plot(1:len_y+1,pk_flg,'r-.',1:len_y+1,vy_flg,'g-.',1:len_y+1,y,'b-');
% plot(1:500,pk_flg(1:500),'r-.',1:500,vy_flg(1:500),'g-.',1:500,y(1:500),'b-');
xlabel('加速度数');ylabel('振幅');
title('基于滤波器的步频检测');
legend('波峰','波谷','三轴加速度矢量和');
