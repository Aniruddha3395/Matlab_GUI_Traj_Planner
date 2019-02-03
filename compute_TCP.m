%% Tool TCP Computing Function

% Function takes in the Points matrix and Group indices matrix and computes
% the Tool Frame Orientation for each Point.

function [bx,by,bz,strt_idx,end_idx] = compute_TCP(points,range,normals)
% Compute Direction Vector for Y axis
strt_idx = range(1,1);
end_idx = range(1,2);
by = [];
bz = [];
bx = [];
for j=strt_idx:end_idx
    if (j~=end_idx)
        direction = points(j+1,:) - points(j,:);
        dir_vec = -direction / norm(direction);   %unit direction vector
    end
    tool_z = -normals(j,:);
    tool_y = cross(tool_z,dir_vec);
    tool_y = tool_y / norm(tool_y);
    tool_x = cross(tool_y,tool_z);
    tool_x = dir_vec / norm(dir_vec);
    bx = [bx; tool_x];
    by = [by; tool_y];
    bz = [bz; tool_z];
end
end