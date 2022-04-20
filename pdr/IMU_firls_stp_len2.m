function [stp_len, dif_acc] = IMU_firls_stp_len2(stp_num,flg,acc)

%{
%   INPUTS
%       stp_num       :  stp_num_detection()��������õ��Ĳ���
%       py_flg        :  stp_num_detection()������ȡ�Ĳ��岨�ȷ�ֵ

%   OUTPUTS
%       stp_len      �� ���Ƶõ��Ĳ���
%       Distance     �� ���ƵĲ����ۼ�ֵ
%}
stp_len    = zeros(stp_num,1);
stp_len1    = zeros(stp_num,1);
stp_len2    = zeros(stp_num,1);
stp_len3    = zeros(stp_num,1);
mAcc = zeros(stp_num,1);

dif_acc    = zeros(stp_num,1);
pos1 = find(flg(:,1)~=0);  % pos1 ��ʾ����λ��
pos2 = find(flg(:,2)~=0);  % pos1 ��ʾ����λ��
sumpk = 0;
for i = 1:length(pos1)
    sumpk = sumpk + flg(pos1(i),1);
end
mPk = sumpk / length(pos1);
% K = 0.087 * mPk + 0.175
 %K1 = 0.087 * mPk + 0.5;
 K2 = 1.3;
 K3 = 1.3;
 K1 = 0.36;
for i = 1:stp_num
    sum = 0;
    for j = pos2(i):pos1(i)
       sum = sum +acc(j); 
    end
    mAcc(i) = sum/(pos1(i)-pos2(i));
end
mAcc;
for i = 1:stp_num
    dif_acc(i) = abs(flg(pos1(i),1) - flg(pos2(i),2));
    
%     K = 0.087 * dif_acc(i)/2 + 0.25;
    stp_len(i) = 0.6;
    stp_len1(i) = K1 * (dif_acc(i)^(1/4));
    stp_len2(i) = K2 * (mAcc(i)-flg(pos2(i),2))/dif_acc(i);
    stp_len3(i) = K3 * mAcc(i).^(1/3);
    
end
var(stp_len1)
var(stp_len2)
var(stp_len3)
figure 
plot(1:stp_num,stp_len,'r--',1:stp_num,stp_len1,'g--',1:stp_num,stp_len2,'b--',1:stp_num,stp_len3,'k--')
xlabel('������');
ylabel('����');
legend('ʵ�ʲ���','����1','����2','����3');
%plot(1:stp_num,stp_len1,'r--')
%plot(1:stp_num,stp_len2,'b--')
%% �������    ������ֱ���˶�
% figure
% plot(1:stp_num,stp_len,'-*')
% title('����');
% Distance = sum(stp_len); 