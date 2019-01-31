text = fileread('DFT.txt');

C = strsplit(text,'\r\n');

idx = [];
for r = 1:36
    idx = [idx; 0+(r-1), 36+(r-1), 73+(r-1), 109+(r-1) ];
end

new_text = '';

for bigR = 0:1:35
    for bigC = 0:1:3
        for line = 1:size(C,2)

            currentline = C(line);
            row = strsplit (currentline{1}, ' = ');
            rhs = row(2);
            lhs = row(1);
            B = regexp(lhs,'\d*','Match');
            r = B{1}{2};
            c = B{1}{3};
            r = str2num(r);
            c = str2num(c);
            
            if(r==bigR && c == bigC)
                disp(lhs)
                disp([r,c,idx(r+1,c+1)])
%                 newline = strcat( 'T_WE[', num2str( idx(r+1,c+1) ), '] = ', rhs);
                newline = strcat( 'T_WE[', num2str( r + c*36 ), '] = ', rhs);
                new_text = strcat(new_text, newline, '\r');
            end
            
         end        
    end
end

% for line = 1:size(C,2)
%     
%     currentline = C(line);
%     row = strsplit (currentline{1}, ' = ');
%     rhs = row(2);
%     lhs = row(1);
%     B = regexp(lhs,'\d*','Match');
%     r = B{1}{2};
%     c = B{1}{3};
%     r = str2num(r);
%     c = str2num(c);
%     disp([r,c,idx(r+1,c+1)])
%     newline = strcat( 'T_WE[', num2str( idx(r+1,c+1) ), '] = ', rhs);
%     new_text = strcat(new_text, newline, '\r');
%     
%     
% end

fid = fopen('DFT_converted.txt','wt');
fprintf(fid, new_text{1});
fclose(fid);