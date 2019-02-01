function [] = scatter3d(data,scatter_style)

if isempty(scatter_style)==1
    scatter_style = 'filled';
end
scatter3(data(:,1),data(:,2),data(:,3),scatter_style);
daspect([1,1,1]);
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');

end