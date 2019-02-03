clc;
clear;
close all;

[v1, f1, n1, stltitle] = stlRead('base.stl');
scatter3d(v1,'.');

T = [1 0 0 100;
     0 -1 0 50;
     0 0 1 100;
     0 0 0 1];

v2 = apply_transformation(v1,T);

hold on;
scatter3d(v2,'.');