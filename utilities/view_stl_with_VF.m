function [] = view_stl_with_VF(v_part,f_part,face_vertex_data,face_color,opacity,edge_color)

if isempty(face_vertex_data)==1
    face_vertex_data = [0.8,0.8,0.8];
end

if isempty(face_color)==1
    face_color = [0.4,0.4,0.4];
end

if isempty(opacity)==1
    opacity = 1;
end

if isempty(edge_color)==1
edge_color = [0.4, 0.4, 0.4];
end

patch('Faces',f_part,'Vertices',v_part,'FaceVertexCData',face_vertex_data,'FaceColor',face_color,'EdgeColor',edge_color,'FaceAlpha',opacity);
daspect([1,1,1]);
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
end