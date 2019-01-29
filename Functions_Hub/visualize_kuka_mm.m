function visualize_kuka_mm(joint_angles)

global rob_T_part;
%%%%%%%%%%         <<<<< KUKA visualizer >>>>>         %%%%%%%%%%

% NOTE: UNCOMMENT THIS ONLY WHEN YOU WANT TO VISUALIZE DATA FROM STL FILES
% [Link0_v, Link0_f, Link0_n, ~] = stlRead('Link0.STL');
% [Link1_v, Link1_f, Link1_n, ~] = stlRead('Link1.STL');
% [Link2_v, Link2_f, Link2_n, ~] = stlRead('Link2.STL');
% [Link3_v, Link3_f, Link3_n, ~] = stlRead('Link3.STL');
% [Link4_v, Link4_f, Link4_n, ~] = stlRead('Link4.STL');
% [Link5_v, Link5_f, Link5_n, ~] = stlRead('Link5.STL');
% [Link6_v, Link6_f, Link6_n, ~] = stlRead('Link6.STL');
% [Link7_v, Link7_f, Link7_n, ~] = stlRead('Link7.STL');
% [tool_v, tool_f, tool_n, ~] = stlRead('Concave_Dome.STL');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% getting data from .mat file is faster
load STL_COARSE_DATA_mm.mat;
% load tool_roller_STL_data_mm.mat;
load tool_roller_STL_data_mm.mat;

take_video = false;
show_tool = true;
show_mold = true;

if show_mold
[mold_v,mold_f,~,~] = stlRead('CAD_stl/Composite_Mold.STL');
mold_v = apply_transformation(mold_v,rob_T_part);
end

if take_video
    vid = VideoWriter('video1.mp4');
    open(vid);
end

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

daspect([1,1,1]);

for i= 1:size(joint_angles,1)
    cla;
    if show_mold
    view_stl_with_VF(mold_v,mold_f,[],[],[],[]);
    hold on;
    end
    FK_T = get_iiwa_FK_all_joints_mex(joint_angles(i,:),eye(4));
    FK_T(1:3,4) = FK_T(1:3,4).*1000;
    Link0_v_transformed = apply_transformation(Link0_v,FK_T(1:4,:));
    patch('Faces',Link0_f,'Vertices',Link0_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
    hold on;
    FK_T(5:7,4) = FK_T(5:7,4).*1000;
    Link1_v_transformed = apply_transformation(Link1_v,FK_T(5:8,:));
    patch('Faces',Link1_f,'Vertices',Link1_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
    hold on;
    FK_T(9:11,4) = FK_T(9:11,4).*1000;
    Link2_v_transformed = apply_transformation(Link2_v,FK_T(9:12,:));
    patch('Faces',Link2_f,'Vertices',Link2_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
    hold on;
    FK_T(13:15,4) = FK_T(13:15,4).*1000;
    Link3_v_transformed = apply_transformation(Link3_v,FK_T(13:16,:));
    patch('Faces',Link3_f,'Vertices',Link3_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
    hold on;
    FK_T(17:19,4) = FK_T(17:19,4).*1000;
    Link4_v_transformed = apply_transformation(Link4_v,FK_T(17:20,:));
    patch('Faces',Link4_f,'Vertices',Link4_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
    hold on;
    FK_T(21:23,4) = FK_T(21:23,4).*1000;
    Link5_v_transformed = apply_transformation(Link5_v,FK_T(21:24,:));
    patch('Faces',Link5_f,'Vertices',Link5_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
    hold on;
    FK_T(25:27,4) = FK_T(25:27,4).*1000;
    Link6_v_transformed = apply_transformation(Link6_v,FK_T(25:28,:));
    patch('Faces',Link6_f,'Vertices',Link6_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
    hold on;
    FK_T(29:31,4) = FK_T(29:31,4).*1000;
    Link7_v_transformed = apply_transformation(Link7_v,FK_T(29:32,:));
    patch('Faces',Link7_f,'Vertices',Link7_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
    hold on;
    if show_tool
    FK_T(33:35,4) = FK_T(33:35,4).*1000;
    tool_v_transformed = apply_transformation(tool_v,FK_T(33:36,:));
    patch('Faces',tool_f,'Vertices',tool_v_transformed,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.1,0.1,0.1],'EdgeColor','none','FaceAlpha',1);
    end
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
    if take_video
        frame = getframe(gcf);
        writeVideo(vid,frame);
    end
    pause(0.005);
end

end
