function [smoothed] = poly_smooth(xy, min_sep, max_sep)
xy = [xy; xy(1, :)];

smoothed = xy(1, :);
for k = 2:(size(xy, 1)-1)
    prev = smoothed(end, :);
    curr = xy(k, :);
    next = xy(k + 1, :);
    prev_dist = norm(curr - prev);
    next_dist = norm(prev - next);
%     plot(gca, smoothed(:, 1), smoothed(:, 2), '.');
%     title(gca, k);
%     hold on;
%     text(0.2 + smoothed(end, 1), smoothed(end, 2), sprintf('%i', k));
%     hold off;
%     pause(0.25);
    if next_dist < max_sep
        smoothed(end + 1, :) = curr;
    elseif prev_dist < min_sep
        continue
    else
        while norm(curr - prev) > max_sep
            dir = (curr - prev) / norm(curr - prev);
            prev = prev + dir * min(max_sep, norm(curr - prev));
            smoothed(end + 1, :) = prev;
        end
        if (norm(smoothed(end, :) - curr) > min_sep)
            dir = (curr - prev) / norm(curr - prev);
            prev = prev + dir * min(max_sep, norm(curr - prev));
            smoothed(end + 1, :) = prev;
        end
    end
end

if norm(smoothed(end, :) - smoothed(1, :)) < min_sep
    smoothed(end, :) = [];
elseif norm(smoothed(end, :) - smoothed(1, :)) > max_sep
    prev = smoothed(end, :);
    curr = smoothed(1, :);
    while norm(curr - prev) > max_sep
        dir = (curr - prev) / norm(curr - prev);
        prev = prev + dir * min(max_sep, norm(curr - prev));
        smoothed(end + 1, :) = prev;
    end
end
end
