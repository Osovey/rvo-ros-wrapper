function [dt] = delauncy_decomp(polys)

profile = cell2mat(polys);
constraints = [];
start = 1;
for k = 1:length(polys)
    if k == 1
        xys = flipud(polys{k});
    else
        xys = polys{k};
    end
    n = size(xys, 1);
    inc = (start):(start+n-2);
    constraints(start:(start+n-1), :) = [inc.' (inc + 1).'; start + n - 1, start];
    start = size(constraints, 1) + 1;
end

dt = DelaunayTri(profile, constraints);
end
