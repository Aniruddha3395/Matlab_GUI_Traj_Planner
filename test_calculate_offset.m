clc;
clear;
close all;

%test for calculating the offset

x1 = rand(1)*1000;
x2 = rand(1)*1000;
y1 = rand(1)*1000;
y2 = rand(1)*1000;

d = 50;

[p3,p4] = offset_pts([x1,y1],[x2,y2],d);

x3 = p3(1);
y3 = p3(2);
x4 = p4(1);
y4 = p4(2);

figure;
daspect([1,1,1])
x = [x1,x2];
y = [y1,y2];
plot(x,y,'r');
daspect([1,1,1])
hold on;
x = [x3,x4];
y = [y3,y4];
plot(x,y,'g');
hold on;
scatter(x1,y1,'filled')

