function SetReplicaOffset(src,~)

global offset_val;
global replica_off_red;
str = get(src,'String');

if ~isempty(str)
    numval = str2double(str);
    if ~isnan(numval)
        offset_val = numval;
        replica_off_red = false;
    else
        replica_off_red = true;
    end
end

end