clc;
clear;
close all;

% gpu test in matlab
% a = eye(10000);
% b = eye(10000).*2+5;
% tic;
% c = a*b;
% toc;

A = gpuArray(eye(10000));
B = gpuArray(eye(10000).*2+5);
tic;
C = A*B;
toc;

