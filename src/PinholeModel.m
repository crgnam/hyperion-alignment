classdef PinholeModel < handle
    properties
        % Defined values
        f (1,1) double
        resolution (1,2) double
        sensor_size (1,2) double
        cx (1,1) double
        cy (1,1) double
        
        % Calculated values
        K  (3,3) double
        sx (1,1) double
        sy (1,1) double
    end
    
    methods 
        function [self] = PinholeModel(varargin)
            p = inputParser;
                validScalar = @(x) isnumeric(x) && numel(x) == 1;
                validSize = @(x) numel(x) == 2 && isnumeric(x);
                validVec = @(x) validSize(x) || validScalar(x);
                addParameter(p,'FocalLength',35/1000,validScalar);
                addParameter(p,'Resolution',[1920 1080],validSize);
                addParameter(p,'SensorSize',35/1000,validVec);
                addParameter(p,'CenterCoordinates',nan(1,2),validSize);
            parse(p,varargin{:});
            
            % Store the data:
            self.f = p.Results.FocalLength;
            self.resolution = p.Results.Resolution;
            if numel(p.Results.SensorSize) == 1
                self.sensor_size = p.Results.SensorSize*[1, self.resolution(2)/self.resolution(1)];
            else
                self.sensor_size = p.Results.SensorSize;
            end
            
            % Calculate the center coordinates if none are provided:
            if all(isnan(p.Results.CenterCoordinates))
                self.cx = self.resolution(1)/2;
                self.cy = self.resolution(2)/2;
            else
                self.cx = p.Results.CenterCoordinates(1);
                self.cy = p.Results.CenterCoordinates(2);
            end
            
            % Calculate the camera projection matrix:
            self.K = [self.f    0     self.cx;
                         0    self.f  self.cy;
                         0       0        1];
                  
            % Calculate scale from meters to pixel:
            self.sx = self.resolution(1)/self.sensor_size(1);
            self.sy = self.resolution(2)/self.sensor_size(2);
        end
    end
    
    methods (Access = public)
        function [rays_camera_frame] = pixels_to_rays(self, pixels)
            % Convert pixels to image points:
            image_points(1,:) = (-self.cx + pixels(1,:))/self.sx;
            image_points(2,:) = (self.cy - pixels(2,:))/self.sy;
            
            rays_camera_frame = [image_points; -self.K(1,1)*ones(1,size(image_points,2))];
        end
        
        function [pixel_pts,in_fov] = points_to_pixels(self, points_camera_frame)
            % Calculate homogenous coordinates:
            homogeneous = (self.K*points_camera_frame)';

            % Normalize the points into focal length coordinates
            image_pts(:,1) = self.cx + self.sx*(self.cx - homogeneous(:,1)./homogeneous(:,3));
            image_pts(:,2) = self.cy - self.sy*(self.cy - homogeneous(:,2)./homogeneous(:,3));
            
            % Convert from meters to pixel coordinates:
            pixel_pts(:,1) = image_pts(:,1);
            pixel_pts(:,2) = image_pts(:,2);
            
            % Determine which points are in the fov:
            remove1 = pixel_pts(:,1) > self.resolution(1);
            remove2 = pixel_pts(:,2) > self.resolution(2);
            remove3 = pixel_pts(:,1) < 0;
            remove4 = pixel_pts(:,2) < 0;
            remove5 = homogeneous(:,3) > 0;
            remove = (remove1 | remove2 | remove3 | remove4 | remove5);
            in_fov = ~remove;
            
            pixel_pts = pixel_pts'; % TODO: Adjust the math above to be native column vectors
        end
    end
end