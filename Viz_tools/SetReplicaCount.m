function SetReplicaCount(src,~)

global num_of_replica;
global replica_count_red;

str = get(src,'String');

if ~isempty(str)
    numval = str2double(str);
    if ~isnan(numval)
        num_of_replica = floor(numval);
        replica_count_red = false;
    else
        replica_count_red = true;
    end
end

end