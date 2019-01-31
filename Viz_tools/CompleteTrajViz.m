function CompleteTrajViz(~,~)

global h0 h1 h2 h3 h4 h5 h6 h7 h_tool;
global take_video;
global show_tool;
global joint_angles_group;
global home_pos;

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
        pause(0.001);
    end
else
    fprintf('\n ...No data available for simulation... \n')
end

%move back to home position
FK_T = get_iiwa_FK_all_joints_mex(home_pos,eye(4));
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