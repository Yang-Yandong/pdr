gyr = load('gyr.txt');
figure
plot(gyr(:,3))
len = length(gyr(:,3));
gyr_s = zeros(len,1);
gyr_s = sort(gyr(:,3),'descend');