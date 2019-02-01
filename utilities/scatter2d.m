function [] = scatter2d(data,scatter_style)

if isempty(scatter_style)==1
    scatter_style = 'filled';
end
scatter(data(:,1),data(:,2),scatter_style);
xlabel('x-axis');
ylabel('y-axis');

end