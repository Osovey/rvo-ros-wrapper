function [] = plot_poly(polygons)
hold all
for k = 1:length(polygons)
    xys = polygons{k};    
    xys = [xys; xys(1, :)];
    x = xys(:, 1);
    y = xys(:, 2);
    plot(x, y, '.-')
end
hold off;
end
