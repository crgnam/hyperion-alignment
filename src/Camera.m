classdef Camera < handle
    % Camera A class for a camera object
    
    properties
        camera_model
        
        position_on_parent
        parent2self
        
        inertial2self
        position
        
        % Visualization parameters:
        vis
    end
    
    methods
        function [self] = Camera(varargin)
%             validCameraModel = @(x) isa(x,'CameraModel');
            validVec = @(x) numel(x) == 3;
            validMatrix = @(x) all(size(x) == [3,3]);
            validAttitude = @(x) isa(x,'Attitude') || validMatrix(x);
            p = inputParser;
            p.KeepUnmatched = true;
                addParameter(p,'CameraModel',PinholeModel());
                addParameter(p,'PositionOnParent',zeros(3,1),validVec);
                addParameter(p,'Parent2Self',eye(3),validAttitude);
            parse(p,varargin{:});
            
            self.camera_model = p.Results.CameraModel;
            self.position_on_parent = p.Results.PositionOnParent;
            self.parent2self = p.Results.Parent2Self;
            
            self.inertial2self = eye(3);
            self.position = zeros(3,1);
                  
            % Preallocate visualization settings:
            self.vis.view = [];
            self.vis.rays = [];
        end
    end
    
    methods (Access = public)       
        function [] = set_pose(self,parent_position, inertial2parent)
            self.position = parent_position + inertial2parent'*self.position_on_parent;
            self.inertial2self = self.parent2self*inertial2parent;
        end
        
        function [pixels, in_fov] = points_to_pixels(self,world_points)           
            % Transform the points to the camera frame:
            camera_points = self.inertial2self*(world_points - self.position);

            % Use the defined camera model to project the points:
            [pixels,in_fov] = self.camera_model.points_to_pixels(camera_points);
        end
        
        function [rays] = pixels_to_rays(self,pixels)   
            % Generate the list of pixels to generate rays for if none was
            % provided:
            if nargin == 1
               u = 1:self.camera_model.resolution(1);
               v = 1:self.camera_model.resolution(2);
               [U,V] = meshgrid(u,v);
               pixels = [U(:)';V(:)'];
            end
            
            % Use the defined camera model to generate rays:
            rays_camera_frame = self.camera_model.pixels_to_rays(pixels);

            % Transform the rays into the inertial space:
            rays = normc(self.inertial2self'*rays_camera_frame);
        end
        
        function [image] = rasterize(self,model)
            % Identify camera boresight:
            camera_bore = -self.inertial2self(3,:)';
            
            % Identify visible triangles:
            visible = sum(camera_bore.*model.face_normals') < 0;
            visible_faces = model.faces(visible,:);
            visible_vec = unique(visible_faces(:));
            
            % Project vertices into image plane:
            pixels_vis = self.points_to_pixels(model.vertices(visible_vec,:)');
            
            % Project on the visible points:
            pixels = zeros(2,size(model.vertices,1));
            pixels(:,visible_vec) = pixels_vis;
           
            image = zeros(self.camera_model.resolution(1),self.camera_model.resolution(2));
            sz = size(image);
            [r,c] = ind2sub(sz,1:numel(image));
            query = [r;c];
            for ii = 1:size(visible_faces,1)
                face = visible_faces(ii,:);
                tri_coords = pixels(:,face);
                [in,~] = inpolygon(query(1,:),query(2,:), tri_coords(1,:),tri_coords(2,:));
                activated = query(:,in);
                ind = sub2ind(sz,activated(1,:),activated(2,:));
                image(ind) = 255;                
            end
        end
    end
    
    methods (Access = public)
        function [] = reset(self)
            self.vis.view = [];
            self.vis.rays = [];
        end
        
        function [] = draw(self,scale,varargin)    
            hx = (self.camera_model.sensor_size(1)/2)/self.camera_model.f;
            hy = (self.camera_model.sensor_size(2)/2)/self.camera_model.f;
            
            x = self.inertial2self(1,:)*hx;
            y = self.inertial2self(2,:)*hy;
            z = self.inertial2self(3,:);
            
            pt2 = scale*(-x + -y + -z) + self.position';
            pt3 = scale*(-x +  y + -z) + self.position';
            pt4 = scale*( x +  y + -z) + self.position';
            pt5 = scale*( x + -y + -z) + self.position';
            pt6 = scale*(-x +  y + -z) + scale*0.3*y + self.position';
            pt7 = scale*( x +  y + -z) + scale*0.3*y + self.position';
            pt8 = scale*( 0 +  y + -z) + scale*y + self.position';
            
            verts = [self.position'; pt2; pt3; pt4; pt5; pt6; pt7; pt8];
            faces1 = [1 2 3; 1 3 4; 1 4 5; 1 5 2];
            faces2 = [2 3 4 5];
            faces3 = [6 7 8];
            if isempty(self.vis.view)
                self.vis.view(1) = patch('Faces',faces1,'Vertices',verts,varargin{:}); hold on
                self.vis.view(2) = patch('Faces',faces2,'Vertices',verts,varargin{:});
                self.vis.view(3) = patch('Faces',faces3,'Vertices',verts,varargin{:});
                axis equal
                rotate3d on
            else
                set(self.vis.view(1),'Vertices',verts);
                set(self.vis.view(2),'Vertices',verts);
                set(self.vis.view(3),'Vertices',verts);
            end
        end
        
        function [] = drawRays(self, scale, rays, varargin)
            rays = rays';
            origin = repmat(self.position',size(rays,1),1);
            
            if isempty(self.vis.rays)
                self.vis.rays = quiver3(origin(:,1),origin(:,2),origin(:,3),...
                                        rays(:,1),rays(:,2),rays(:,3),scale,varargin{:});
            else
                % TODO: Finish this... I'm too lazy right now...
                self.reset();
            end
        end
    end
end