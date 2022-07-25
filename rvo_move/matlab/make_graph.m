function [xy, neighbors] = make_graph(map, dt)
inside = inOutStatus(dt);
incent = dt.incenters;
neighbors = cell(size(incent, 1), 1);
for k = 1:size(dt.Triangulation)
    ns = dt.neighbors(k);
    n = ns(~isnan(ns));
    neighbors{k} = n(inside(n));
end

second_neighbors = cell(size(neighbors));
for k = 1:size(neighbors)
    ns = neighbors{k};
    for l = 1:length(ns)
        if ~inside(k) || isnan(ns(l)) || ~inside(ns(l))
            continue
        end
        others = ns(ns ~= ns(l));
        second_neighbors{ns(l)} = unique([others, second_neighbors{ns(l)}]);
    end
end

neighbors = cellfun(@(a, b) unique([a, b]), neighbors, second_neighbors, ...
    'UniformOutput', false);
xy = incent;

% Remove invalid vertices and calculate new neighbor indices
valid = inside & ~map.collide(xy);
new_inds = cumsum(valid);
xy = xy(valid, :);
neighbors = neighbors(valid);
neighbors = cellfun(@(a) new_inds(a), neighbors, 'UniformOutput', false);

end
