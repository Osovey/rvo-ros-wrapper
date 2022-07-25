close all;
clear all;
map = Map.FromYAML('~/git/scarab/upenn_maps/maps/levine_towne_nav.yaml');

%%
inflated_map = map.InflateObstacles(1.0);
plot(inflated_map)
polygons = fit_polygons(inflated_map, 120, 100);
% Change coordinates of outside boundary to be counterclockwise
polygons{1} = flipud(polygons{1});

%%
min_sep = 0.3;
max_sep = 2.0;
smoothed = cellfun(@(x) poly_smooth(x, min_sep, max_sep), polygons, 'UniformOutput', false);
clf(gcf);
figure(gcf);
plot_poly(smoothed)
%%
dt = delauncy_decomp(smoothed);
[xy, neighbors] = make_graph(map, dt);
assert(~any(map.collide(xy)));
%% Visualize results
plot_delaunay(map, dt, xy, neighbors)

%% Save the neighbors to disk
fid = fopen('levine_towne_waypoints.txt', 'w');
strs = cellfun(@(a) sprintf(' %i', a), neighbors, 'UniformOutput', false);
for k = 1:size(xy, 1)
    fprintf(fid, '%0.5f %0.5f%s\n', xy(k, :), strs{k});
end
fclose(fid);

%% Save the polygons to disk
fid = fopen('levine_towne_polygons.txt', 'w');
for k = 1:length(polygons)
    fprintf(fid, '===\n');
    fprintf(fid, '% 7.3f % 7.3f\n', polygons{k}.');
end
fprintf(fid, '===');
fclose(fid);
