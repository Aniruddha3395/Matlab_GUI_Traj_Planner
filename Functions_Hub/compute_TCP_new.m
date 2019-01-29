function [bx,by,bz] = compute_TCP_new(points,normals)

% tcp computating...bx,by,bz
by = zeros(size(points,1),3);
bz = zeros(size(points,1),3);
bx = zeros(size(points,1),3);

for i=1:size(points,1)
    if i~=size(points,1)
        % direction = points(i+1,:) - points(i,:);
        direction = [0 1 0];
        dir_vec = direction / norm(direction);   %unit direction vector
    end
    tool_z = -normals(i,:);
    tool_x = cross(dir_vec,tool_z);
    tool_x = tool_x / norm(tool_x);
    tool_y = cross(tool_z,tool_x);
    tool_y = tool_y / norm(tool_y);
    
    bx(i,:) = tool_x;
    by(i,:) = tool_y;
    bz(i,:) = tool_z;
end
end