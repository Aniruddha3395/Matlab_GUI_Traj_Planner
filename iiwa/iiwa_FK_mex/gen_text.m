output = "double ";
for i = 2:180
    output = strcat(output, "t", num2str(i), ", ");
end
output = strcat(output, ";");

% dlmwrite('double_txt.txt', output)