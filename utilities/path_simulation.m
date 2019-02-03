clc;
clear;
close all;
format short
part_stl = 'base.STL';
tool_stl = 'tool.STL';


%%%%%%%%%%%make function....clean this code

[v_part, f_part, n_part, stltitle_part] = stlRead(part_stl);
face_vertex_data = [0.8,0.8,0.8];
face_color = [0.8,0.8,0.8];
opacity = 1;
edge_color = [0, 0, 0];
patch('Faces',f_part,'Vertices',v_part,'FaceVertexCData',face_vertex_data,'FaceColor',face_color,'EdgeColor',edge_color,'FaceAlpha',opacity);

daspect([1,1,1]);

% l1_old = dlmread('delete_l1.txt');
% l2_old = dlmread('delete_l2.txt');
% 
% l3_old = dlmread('delete_l3.txt');

l1 = dlmread('ABB_bottom_layer_points_part2_base_xdir_1mm.csv');

l1(1:650,4:5) = 0;

% l1(:,1:3) = l1(:,1:3)./1000;
% l1(:,4:5) = l1(:,4:5)./10000;
l1(:,4:5) = l1(:,4:5).*(pi/180);

% l2 = dlmread('delete_l2.txt');
% l2(:,1:3) = l2(:,1:3)./1000;
% l3 = dlmread('delete_l3.txt');
% l3(:,1:3) = l3(:,1:3)./1000;
% l4 = dlmread('delete_l4.txt');
% l4(:,1:3) = l4(:,1:3)./1000;
view(-37,32);
hold on;
xlabel('x');
ylabel('y');
zlabel('z');

hold on;
[v_tool, f_tool, n_tool, stltitle_tool] = stlRead(tool_stl);
patch('Faces',f_tool,'Vertices',v_tool,'FaceVertexCData',col_matrix_part,'FaceColor',[0.29,0.29,0.29],'EdgeColor','none','FaceAlpha',0.9);


vnew = v_tool;
l1_new = [l1(1,:)];
for i = 1:size(l1,1)-1
    cla;
    [v_part, f_part, n_part, stltitle_part] = stlRead(part_stl);
    % col_matrix = [0.941176, 0.972549, 1];
    col_matrix_part = [0.8,0.8,0.8];
    col_matrix2_part = [0.3,0.3,0.3];
    xlim([min(v_part(:,1)),max(v_part(:,1))]);
    ylim([min(v_part(:,2)),max(v_part(:,2))]);
    zlim([min(v_part(:,3)),max(v_part(:,3))+50]);
    patch('Faces',f_part,'Vertices',v_part,'FaceVertexCData',col_matrix_part,'FaceColor',col_matrix_part,'EdgeColor','none','FaceAlpha',0.7);
    daspect([1,1,1]);
    view(-37,32);
%     hold on;
%     plot3(l1_old(:,1)/1000,l1_old(:,2)/1000,l1_old(:,3)/1000,'b');
%     hold on;
%     plot3(l2_old(:,1)/1000,l2_old(:,2)/1000,l2_old(:,3)/1000,'r');
%     
%     hold on;
%     plot3(l3_old(:,1)/1000,l3_old(:,2)/1000,l3_old(:,3)/1000,'g');
    hold on;
    l1_new = [l1_new;l1(i+1,:)];
    plot3(l1_new(:,1),l1_new(:,2),l1_new(:,3),'b');
    hold on;
    hold on;
    [v_tool, f_tool, n_tool, stltitle_tool] = stlRead(tool_stl);
%     vnew(:,1) = v_tool(:,1) + l1_new(i+1,1);
%     vnew(:,2) = v_tool(:,2) + l1_new(i+1,2);
%     vnew(:,3) = v_tool(:,3) + l1_new(i+1,3);

    T = [eul2rotm(l1(i+1,4:6),'ZYX'),l1(i+1,1:3)';0,0,0,1];
    vnew = T*[v_tool,ones(size(v_tool,1),1)]';    
    vnew = vnew';
    patch('Faces',f_tool,'Vertices',vnew(:,1:3),'FaceVertexCData',col_matrix_part,'FaceColor',[0.29,0.29,0.29],'EdgeColor','none','FaceAlpha',0.9);
    
    pause(0.0001)
end
