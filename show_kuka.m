% show kuka robot next to mold

clear all;
close all;
clc;
figure;
set(figure,'units','normalized','outerpos',[0 0 1 1.2]);
xlim([-300,1000]);
ylim([-800,800]);
zlim([-100,1500]);
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
view([35,15])
% view([90,1])

load STL_COARSE_DATA_mm.mat;

joint_angles = dlmread('data_files/test_joint_angles.csv');


p0 = patch('Faces',Link0_f,'Vertices',Link0_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
p1 = patch('Faces',Link1_f,'Vertices',Link1_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
p2 = patch('Faces',Link2_f,'Vertices',Link2_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
p3 = patch('Faces',Link3_f,'Vertices',Link3_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
p4 = patch('Faces',Link4_f,'Vertices',Link4_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
p5 = patch('Faces',Link5_f,'Vertices',Link5_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
p6 = patch('Faces',Link6_f,'Vertices',Link6_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
p7 = patch('Faces',Link7_f,'Vertices',Link7_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);

%%%%%% camera settings %%%%%%
    camlight(-360,0);
    hold on;
    camlight(0,0);
    hold on;
    %     camlight('left');
    hold on;
    camlight('right');
    hold on;
    %     camlight(0,0);
    %     hold on;
    %     camlight(0,0);

h0 = hgtransform;
set(p0,'parent', h0);
h1 = hgtransform;
set(p1,'parent', h1);
h2 = hgtransform;
set(p2,'parent', h2);
h3 = hgtransform;
set(p3,'parent', h3);
h4 = hgtransform;
set(p4,'parent', h4);
h5 = hgtransform;
set(p5,'parent', h5);
h6 = hgtransform;
set(p6,'parent', h6);
h7 = hgtransform;
set(p7,'parent', h7);
    
for i=1:size(joint_angles,1)
FK_T = get_iiwa_FK_all_joints_mex(joint_angles(i,:),eye(4));
FK_T(1:3,4) = FK_T(1:3,4).*1000;
% Link0_v_transformed = apply_transformation(Link0_v,FK_T(1:4,:));
% hold on;
FK_T(5:7,4) = FK_T(5:7,4).*1000;
% Link1_v_transformed = apply_transformation(Link1_v,FK_T(5:8,:));
% hold on;
FK_T(9:11,4) = FK_T(9:11,4).*1000;
% Link2_v_transformed = apply_transformation(Link2_v,FK_T(9:12,:));
% hold on;
FK_T(13:15,4) = FK_T(13:15,4).*1000;
% Link3_v_transformed = apply_transformation(Link3_v,FK_T(13:16,:));
% hold on;
FK_T(17:19,4) = FK_T(17:19,4).*1000;
% Link4_v_transformed = apply_transformation(Link4_v,FK_T(17:20,:));
% hold on;
FK_T(21:23,4) = FK_T(21:23,4).*1000;
% Link5_v_transformed = apply_transformation(Link5_v,FK_T(21:24,:));
% hold on;
FK_T(25:27,4) = FK_T(25:27,4).*1000;
% Link6_v_transformed = apply_transformation(Link6_v,FK_T(25:28,:));
% hold on;
FK_T(29:31,4) = FK_T(29:31,4).*1000;
% Link7_v_transformed = apply_transformation(Link7_v,FK_T(29:32,:));
% hold on;
    

set(h0, 'matrix', FK_T(1:4,:));
set(h1, 'matrix', FK_T(5:8,:));
set(h2, 'matrix', FK_T(9:12,:));
set(h3, 'matrix', FK_T(13:16,:));
set(h4, 'matrix', FK_T(17:20,:));
set(h5, 'matrix', FK_T(21:24,:));
set(h6, 'matrix', FK_T(25:28,:));
set(h7, 'matrix', FK_T(29:32,:));

pause(0.005);
end