function [] = plot_delaunay(map, dt, xy, neighbors)
clf(gcf);
figure(gcf);
inside = inOutStatus(dt);
plot(map);
hold on;
centers = dt(inside, :);
triplot(centers, dt.X(:,1), dt.X(:,2));
plot(xy(:, 1), xy(:, 2), 'r.', 'MarkerSize', 22);
for k = 1:length(neighbors)
    ns = neighbors{k};
    for l = 1:length(ns)
        xs = [xy(k, 1); xy(ns(l), 1)];
        ys = [xy(k, 2); xy(ns(l), 2)];
        plot(xs, ys, 'k-');
    end
end
hold off;
end
