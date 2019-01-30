function visualize_kuka_mm_test(~,~)
% NOTE:
% TO run the code standalone - inputs are : joint angles, figure handle

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

global h0 h1 h2 h3 h4 h5 h6 h7 h_tool;
global take_video show_tool;
global joint_angles_group;

if take_video
    vid = VideoWriter('video1.mp4');
    open(vid);
end

hold on;

if ~isempty(joint_angles_group)
    for i= 1:size(joint_angles_group,1)
        
        FK_T = get_iiwa_FK_all_joints_mex(joint_angles_group(i,:),eye(4));
        FK_T(1:3,4) = FK_T(1:3,4).*1000;
        FK_T(5:7,4) = FK_T(5:7,4).*1000;
        FK_T(9:11,4) = FK_T(9:11,4).*1000;
        FK_T(13:15,4) = FK_T(13:15,4).*1000;
        FK_T(17:19,4) = FK_T(17:19,4).*1000;
        FK_T(21:23,4) = FK_T(21:23,4).*1000;
        FK_T(25:27,4) = FK_T(25:27,4).*1000;
        FK_T(29:31,4) = FK_T(29:31,4).*1000;
        
        set(h0,'matrix',FK_T(1:4,:));
        set(h1,'matrix',FK_T(5:8,:));
        set(h2,'matrix',FK_T(9:12,:));
        set(h3,'matrix',FK_T(13:16,:));
        set(h4,'matrix',FK_T(17:20,:));
        set(h5,'matrix',FK_T(21:24,:));
        set(h6,'matrix',FK_T(25:28,:));
        set(h7,'matrix',FK_T(29:32,:));
        
        if show_tool
            FK_T(33:35,4) = FK_T(33:35,4).*1000;
            set(h_tool, 'matrix', FK_T(33:36,:));
        end
        
        if take_video
            frame = getframe(gcf);
            writeVideo(vid,frame);
        end
        pause(0.01);
    end
else
    fprintf('\n ...No data available for simulation... \n')
end
%move back to home position
FK_T = get_iiwa_FK_all_joints_mex(zeros(7,1),eye(4));
FK_T(1:3,4) = FK_T(1:3,4).*1000;
FK_T(5:7,4) = FK_T(5:7,4).*1000;
FK_T(9:11,4) = FK_T(9:11,4).*1000;
FK_T(13:15,4) = FK_T(13:15,4).*1000;
FK_T(17:19,4) = FK_T(17:19,4).*1000;
FK_T(21:23,4) = FK_T(21:23,4).*1000;
FK_T(25:27,4) = FK_T(25:27,4).*1000;
FK_T(29:31,4) = FK_T(29:31,4).*1000;

set(h0,'matrix',FK_T(1:4,:));
set(h1,'matrix',FK_T(5:8,:));
set(h2,'matrix',FK_T(9:12,:));
set(h3,'matrix',FK_T(13:16,:));
set(h4,'matrix',FK_T(17:20,:));
set(h5,'matrix',FK_T(21:24,:));
set(h6,'matrix',FK_T(25:28,:));
set(h7,'matrix',FK_T(29:32,:));

if show_tool
    FK_T(33:35,4) = FK_T(33:35,4).*1000;
    set(h_tool, 'matrix', FK_T(33:36,:));
end
end