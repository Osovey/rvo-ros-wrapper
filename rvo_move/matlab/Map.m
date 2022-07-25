classdef Map < handle
    properties
        % 2D occupancy grid; 0 if free, 1 if occupied
        occgrid = [];
        res_xy = 1.0;
        % 4 element vector of environment lims: [xy]_min, [xz]_max
        bound_xy = [];
        
        wall_ij;
    end
    
    methods(Static)
        function [new_map] = FromYAML(path, pad_xy)
            if nargin < 2
                pad_xy = [0 0];
            end
            dict = read_yaml(path);
            base = fileparts(path);
            if base
                map_path = [base filesep dict('image')];
            else
                map_path = dict('image');
            end
            img = imread(map_path);
            origin = dict('origin');
            new_map = Map(img, dict('resolution'), origin(1:2), pad_xy);
        end
    end

    methods
        function [obj] = Map(grid, res_xy, offset, pad_xy)
            if nargin < 4
                pad_xy = [0 0];
            end
            obj.res_xy = res_xy;
            obj.bound_xy = [offset, offset + res_xy * [size(grid, 2), size(grid, 1)]];
            
            grid_sz = size(grid);
            pad_ij = fliplr(round(pad_xy / obj.res_xy));
            obj.occgrid = 255 * ones(size(grid) + pad_ij * 2, 'uint8');
            obj.occgrid(pad_ij(1) + (1:grid_sz(1)), pad_ij(2) + (1:grid_sz(2))) = flipud(grid);
            
            obj.bound_xy(1:2) = obj.bound_xy(1:2) - pad_xy;
            obj.bound_xy(3:4) = obj.bound_xy(3:4) + pad_xy;

            [wall_i, wall_j] = find(obj.occgrid < 76);
            obj.wall_ij = [wall_i, wall_j];
        end
        
        function [new_map] = InflateObstacles(map, amount)
            grid = double(map.occgrid < 120);
            grid = conv2(grid, ones(round(amount / map.res_xy)), 'same');
            grid = 255 * (grid < 1); 
            new_map = Map(flipud(grid), map.res_xy, map.bound_xy(1:2), [0 0]);
        end
        
        
        function [] = wipe(obj, x_lim, y_lim)
            xys = obj.indToXY(find(double(obj.occgrid) < 78));
            valid = xys(:, 1) >= x_lim(1) & xys(:, 1) <= x_lim(2) ...
                  & xys(:, 2) >= y_lim(1) & xys(:, 2) <= y_lim(2);
            obj.occgrid(obj.xyToInd(xys(valid, :))) = 255;
        end
        
        function [] = fill(obj, x_lim, y_lim)
            xys = obj.indToXY(find(double(obj.occgrid) > 78));
            valid = xys(:, 1) >= x_lim(1) & xys(:, 1) <= x_lim(2) ...
                  & xys(:, 2) >= y_lim(1) & xys(:, 2) <= y_lim(2);
            obj.occgrid(obj.xyToInd(xys(valid, :))) = 77;
        end
        
%         function [xy] = wallCoords(obj, x_lim, y_lim)
%             % Get all xy coordinates of walls within givin box
%             upleft_ij = obj.xyToSub([x_lim(1), y_lim(2)]);
%             lowright_ij = obj.xyToSub([x_lim(2), y_lim(1)]);
%             
%             max_i = upleft_ij(1);
%             min_j = upleft_ij(2);
%             min_i = lowright_ij(1);
%             max_j = lowright_ij(2);
%             
% %             valid = (min_i <= obj.wall_ij(:, 1) & obj.wall_ij(:, 1) <= max_i & ...
% %                      min_j <= obj.wall_ij(:, 2) & obj.wall_ij(:, 2) <= max_j);
%             xy = obj.subToXY(obj.wall_ij(:, :));
%         end
        
        function [xy] = subToXY(map, ij)
            ji = [ij(:, 2), ij(:, 1)];
            scaled = bsxfun(@(a, b) a .* b, ji - 1, map.res_xy);
            xy = bsxfun(@plus, scaled, map.bound_xy(1:2) + map.res_xy / 2);
        end
        
        function [ij] = xyToSub(map, xy)
            diff = bsxfun(@minus, xy, map.bound_xy(1:2));
            
            % Make origin of map (i.e. bound_xy(1:2)) have valid subscript
            diff(bsxfun(@lt, diff, map.res_xy)) = eps;
            
            jik = bsxfun(@(a, b) a ./ b, diff, map.res_xy);
            inds = jik ~= floor(jik);
            jik(inds) = floor(jik(inds)) + 1;
            
            % Points outside boundary get negative indicies
            valid = all(bsxfun(@le, map.bound_xy(1:2), xy) & ...
                        bsxfun(@ge, map.bound_xy(3:4), xy), 2);
            jik(~valid, :) = -1;
            ij = [jik(:, 2), jik(:, 1)];
        end
        
        function [inds] = xyToInd(map, xy)
            ij = map.xyToSub(xy);
            inds = round(sub2ind(size(map.occgrid), ij(:, 1), ij(:, 2)));
        end
        
        function [xy] = indToXY(map, ind)
            [i, j] = ind2sub(size(map.occgrid), ind);
            xy = map.subToXY([i, j]);
        end
        
        function [collide] = collide(map, xys)
            inds = map.xyToInd(xys);
            collide = map.occgrid(inds) < 100;
        end
        
        
        function plot(obj, fh, min_xy, max_xy)
            if nargin < 2
                fh = gcf;
                clf(fh);
            end
            if nargin < 4
                min_xy = obj.bound_xy(1:2);
                max_xy = obj.bound_xy(3:4);
            end
            if isempty(min_xy)
                    min_xy = obj.bound_xy(1:2);
            end
            if isempty(max_xy)
                    max_xy = obj.bound_xy(3:4);
            end
            
            min_ij = obj.xyToSub(min_xy);
            max_ij = obj.xyToSub(max_xy);
            if any(max_ij == -1 | min_ij == -1)
                warning('map:limits', 'Requested x_lim or y_lim is bigger than map');
            end
            map = obj.occgrid(min_ij(1):max_ij(1), min_ij(2):max_ij(2));
            
            % Make figure pretty + informative
            imshow(map, 'XData', [min_xy(1), max_xy(1)], ...
                        'YData', [min_xy(2), max_xy(2)], ...
                        'Parent', gca);
            ax = get(fh, 'CurrentAxes');
            hold(ax, 'on');
            axis(ax, 'on');
            axis(ax, 'xy');
            xlabel(ax, 'X (m)');
            ylabel(ax, 'Y (m)');
            axis(ax, 'equal');
            xlim(ax, [min_xy(1), max_xy(1)]);
            ylim(ax, [min_xy(2), max_xy(2)]);
            hold(ax, 'off');
        end
        
        function display(map)
            s = char('', sprintf('Map:'));
            [i, j] = size(map.occgrid);
            s = char(s, sprintf('  %i-by-%i', i, j));
            s = char(s, ...
                     sprintf('  X:(% 5.1f,% 5.1f) Y:(% 5.1f,% 5.1f)', ...
                             map.bound_xy([1, 3, 2, 4])));
            s = char(s, sprintf('  XY-Res: %6.2f', map.res_xy));
            disp(s);
        end
    end    
end
