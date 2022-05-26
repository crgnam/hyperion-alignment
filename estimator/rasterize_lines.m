function [outline] = rasterize_lines(connected_points)
    N = size(connected_points,2);
    outline = nan(2, 10*N);
    idx = 1;
    for ii = 1:N-1
        p1 = connected_points(:,ii);
        p2 = connected_points(:,ii+1);
        dist = norm(p2-p1);
        CHECK = sqrt(2);
        CHECK = 1.3;
        if dist > CHECK
            % Interpoalte between the points:
            num_insert = ceil(dist/CHECK);
            x = linspace(p1(1), p2(1), num_insert);
            y = linspace(p1(2), p2(2), num_insert);
            new_points = [x;y];
            num_new = size(new_points,2);

            % Insert the new points:
            outline(:,idx:(idx+num_new-1)) = new_points;
            idx = idx + num_new-1;
        else
            outline(:,idx:(idx+1)) = [p1, p2];
            idx = idx+1;
        end
    end
    
    % Remove unused points:
    outline(:,isnan(outline(1,:))) = [];
end