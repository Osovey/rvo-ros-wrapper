function [] = plot_graph(m, xys, neighbors)

hold on;
plot(xys(:, 1), xys(:, 2), '.')

for k = 1:size(xys, 1)
    xy = xys(k, :);
    
    for n = 1:length(neighbors{k})
        oxys = xys(neighbors{k}, :);
        for l = 1:n
            plot([xy(1), oxys(l, 1)], [xy(2); oxys(l, 2)], '-');
        end
    end
end
xl = xlim;
yl = ylim;
for k = 1:size(xys, 1)
    if (xl(1) < xys(k, 1) && xys(k, 1) < xl(2) && ...
        yl(1) < xys(k, 2) && xys(k, 2) < yl(2))
        text(xys(k, 1) + 0.2, xys(k, 2) + 0.2, sprintf('%i', k));
    end
end


end
