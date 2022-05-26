function [new_points] = subdivide_line(line, D, follow_path)
    % Subdivide a line defined by a series of discrete points given a
    % specific target average distancebetween points
    
    if nargin == 2
        follow_path = true;
    end
    
    % Obtain all distances:
    [~,dists] = normc(line(:,2:end) - line(:,1:end-1));
    perimeter = sum(dists);
    
    % Determine size of output points:
    M = size(line,2);
    N = floor(perimeter/D);
    new_points = nan(2,2*N);
    new_points(:,1) = line(:,1);
    kk = 2;
    for ii = 2:2*N
        % Check the distance until the next point:
        dir = line(:,kk) - new_points(:,ii-1);
        d = norm(dir);
        dir = dir/d;
        
        % If current distance is greater than target distance, step in
        % direction towards next point
        if d > D
            new_points(:,ii) = new_points(:,ii-1) + D*dir;
            next_point = false;
            
        % If current distance is less than target distance, jump to next
        % point, and step in direction in the point that follows
        elseif d < D
            if follow_path
                new_dir = line(:,kk+1) - line(:,kk);
                new_dir = new_dir/norm(new_dir);
                new_points(:,ii) = line(:,kk) + (D-d)*(new_dir);
            else
                new_dir = line(:,kk+1) - new_points(:,ii-1);
                new_dir = new_dir/norm(new_dir);
                new_points(:,ii) = new_points(:,ii-1) + D*new_dir;
            end
            next_point = true;
        
        % If current distance is equal to the target distance, simply store
        % the next point.
        else
            new_points(:,ii) = line(:,kk);
            next_point = true;
        end
        
        % Get the next point to interpolate with:
        if next_point
            kk = kk+1;
        end
        
        % Exit if the last point has been reached:
        if kk == M
            break
        end
    end
    new_points(:,isnan(new_points(1,:))) = [];
end