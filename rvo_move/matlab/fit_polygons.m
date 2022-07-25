function [polygons] = fit_polygons(map, occ_cutoff, pixel_cutoff)
% FIT_POLYGONS Fit polygons to obstacles in occupancy grid
%  Finds boundary pixels of connected components of pixels that represent
%  occupied space and then fits straight line segments to them.  polygons is
%  a cell array of xy-coordinates arranged in counterclockwise order.
%
%  map: Map() object.
%  occ_cutoff: Pixels in map.occgrid above this value are considered as occupied.
%  pixel_cutoff: Minimum number of boundary pixels for a polygon to be returned.
occgrid = map.occgrid;
plot(map);
hold on;
bin_occ = occgrid > occ_cutoff;
% Coordinates are ordered clockwise, but because occgrid is flipped along 
% y-axis, the result will be clockwise in xy coordinates.
bound = bwboundaries(bin_occ);
valid = cellfun(@length, bound) > pixel_cutoff;
bound = bound(valid);
vc = varycolor(length(bound));
set(gca, 'ColorOrder', vc);
hold all;
polygons = cell(length(bound), 1);
for k = 1:length(bound)
    pixels = bound{k};
    xys = map.subToXY(pixels);
    polygons{k} = polyOutline(xys, inf);
    plot(polygons{k}(:, 1), polygons{k}(:, 2), '.-', 'LineWidth', 1);
end
hold off;
end

function [xys] = polyOutline(xys, dist)
prev_xy1 = xys(1, :);
prev_xy2 = xys(2, :);
new_xys = prev_xy1;
for k = 3:size(xys, 1)
    curr = xys(k, :);
    diff1 = sign(prev_xy1 - prev_xy2);
    diff2 = sign(prev_xy2 - curr);
    same_dir = all(diff1 == diff2) || (norm(diff1)^2 == 2 && norm(diff2)^2 == 2);
    if ~same_dir || norm(prev_xy2 - new_xys(end, :)) > dist
        new_xys = [new_xys; prev_xy2];
    else
        ;
    end
    
    prev_xy1 = prev_xy2;
    prev_xy2 = curr;
end
xys = new_xys;
end