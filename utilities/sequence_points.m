function [waypoints] = sequence_points(data)
%input as xyz data

data = unique(data,'rows');

waypoints = [];
current_pt = data(1,:);
iter = 1;
while(1)
    waypoints = [waypoints; current_pt];
    sorted_data = sort_neighbors(data, current_pt);
    current_pt = get_next_pt(sorted_data,current_pt,waypoints);
    iter = iter+1;
    if norm(current_pt - waypoints(end,:))>100
        break;
    end
end

function npt = get_next_pt(sorted_data,pt,waypoints)
    for i = 1:size(sorted_data,1)
        npt = sorted_data(i,:);
        [~,index] = ismember(waypoints,npt,'rows');
        if(sum(index>0))
            continue;
        end
        if(norm(pt-npt)>0)
            break;
        end
    end
end



function sorted_data = sort_neighbors(data, pt)
    distance = data - pt;
    distance = vecnorm(distance,2,2);
    augmented_data = [distance,data];
    temp_sorted_data = sortrows(augmented_data,1, 'ascend');
    sorted_data = temp_sorted_data(:,2:4);
end

end