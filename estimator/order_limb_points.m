function [x_r,y_r] = order_limb_points(x_u, y_u)
    N = numel(x_u);
    
    x_r = nan(N,1);
    y_r = nan(N,1);
    
    % This assumes the sun-vector is coming in perfectly horizontally
    start_ind = find(y_u == min(y_u),1,'first');
    x_r(1) = x_u(start_ind);
    y_r(1) = y_u(start_ind);
    
    % Remove the used point from points to be considered:
    remaining_pts = 1:N;
    remaining_pts(remaining_pts == start_ind) = [];
    
    % Loop through:
    pts = [x_u, y_u]';
    tol = 5;
    
    for ii = 1:N-1
        % Identify the remaining points:
        pts_consider = pts(:,remaining_pts);
        
        % Identify the closest point:
        [~,dist] = normc([x_r(ii); y_r(ii)] - pts_consider);
        if min(dist) > tol
            break
        end
        used_point = remaining_pts(find(dist==min(dist),1,'first'));
        
        % Assign the newest point:
        x_r(ii+1) = x_u(used_point);
        y_r(ii+1) = y_u(used_point);
        
        % Remove the currently used point:
        remaining_pts(remaining_pts == used_point) = [];
    end
    
    x_r(isnan(x_r)) = [];
    y_r(isnan(y_r)) = [];
end