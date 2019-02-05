function show_origin()

% code to show origin in the figure

dir_vec_len = 50;
hold on;
scatter3(0,0,0,50,'d','filled','k');
hold on;
quiver3(0,0,0,1,0,0,dir_vec_len,'r');
hold on;
quiver3(0,0,0,0,1,0,dir_vec_len,'g');
hold on;
quiver3(0,0,0,0,0,1,dir_vec_len,'b');
hold on;
end