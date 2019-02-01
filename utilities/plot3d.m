function [] = plot3d(data,line_color)

if isempty(line_color)==1
    line_color = 'b';
end
plot3(data(:,1),data(:,2),data(:,3),'Color',line_color);
daspect([1,1,1]);
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');

end