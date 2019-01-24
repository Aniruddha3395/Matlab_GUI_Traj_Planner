function [] = plot2d(data,line_color)

if isempty(line_color)==1
    line_color = 'b';
end

plot(data(:,1),data(:,2),'Color',line_color);
xlabel('x-axis');
ylabel('y-axis');

end

