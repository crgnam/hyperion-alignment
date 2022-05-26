classdef ShapeModel < handle
    % ShapeModel A class for handling computations involving shape models.
    % TODO: Transpose everything to be column vectors
    
    properties
        % Definition of face vertices:
        faces
        
        % Initial definitions of data:
        vertices_init
        face_centers_init
        face_normals_init
        vertex_normals_init
        
        % Pose information:
        inertial2self
        position
        
        % Vertices based on pose of the parent RigidBody object:
        vertices
        face_centers
        face_normals
        vertex_normals
        
        % Boundary Volume Heirarchy acceleration structure:
        bvh
        
        % Visualization:
        vis
    end
    
    %% Constructor
    methods
        function [self] = ShapeModel(geometry)
            % Check if an object file has been provided or a patch struct:
            is_file = false;
            if ischar(geometry)
                if exist(geometry,'file')
                    filename = geometry;
                    is_file = true;
                else
                    error(['Input object must be either a valid file path to a .OBJ or .PLY file,\n',...
                           'or it must be a valid struct containing both a faces and vertices field'])
                end
            elseif isa(geometry,'struct')
                assert(all(isfield(geometry,{'faces','vertices'})),...
                       'Input struct must have both a faces and vertices field')
                f = fields(geometry);
                for ii = 1:length(f)
                    self.(f{ii}) = geometry.(f{ii});
                end
            else
                error(['Input object must be either a valid file path to a .OBJ or .PLY file,\n',...
                       'or it must be a valid struct containing both a faces and vertices field'])
            end
            
            % If it is a path to a file, check if it is a .obj or .ply:
            if is_file
                [~,~,ext] = fileparts(filename);
                switch lower(ext)
                    case '.obj'
                        obj = readObj(filename);
                        self.faces    = obj.f.v;
                        self.vertices = obj.v;
                        if all(size(obj.vn) == size(obj.f.v))
                            self.face_normals = obj.vn;
                        end
                    case '.ply'
                        [f,v] = plyread(filename,'tri');
                        self.faces = f;
                        self.vertices = v;
                    otherwise
                        error('Input file must be a valid .OBJ or .PLY file')
                end               
            end
            
            % Calculate the face centers:
            if isempty(self.face_centers)
                self.calculateFaceCenters();
            end
            
            % Calculate the vertex normals:
            if isempty(self.vertex_normals)
                self.calculateNormals();
            end
            
            % Set current values to the initial:
            self.vertices_init       = self.vertices;
            self.face_normals_init   = self.face_normals;
            self.vertex_normals_init = self.vertex_normals;
            self.face_centers_init   = self.face_centers;
        end
    end
    
    methods (Access = public)
        % Generic methods:
        function [] = buildBVH(self,varargin)
            self.bvh = KDTree(self,varargin{:});
        end
        
        function [] = set_pose(self,position,inertial2self)
            self.position = position;
            self.inertial2self = inertial2self;
            
            % Adjust positions:
            self.face_centers = (inertial2self'*self.face_centers_init')' + position';
            self.vertices = (inertial2self'*self.vertices_init')' + position';
            
            % Adjust normals:
            self.vertex_normals = (inertial2self'*self.vertex_normals_init')';
            self.face_normals = (inertial2self'*self.face_normals_init')';
        end
        
        function [spheres] = densePackUniformSpheres(self,radius)
            % Parse the inputs:
            if nargin == 1
                radius = 1; % Default radius value
            end

            % Get a padded range of x,y,z values to use:
            xyz_max = max(self.vertices_init)+radius;
            xyz_min = min(self.vertices_init)-radius;

            % Generate the lattice:
            Ax = xyz_min(1):2*radius:xyz_max(1);
            Ay = xyz_min(2):(radius*sqrt(3)):xyz_max(2);
            Az = xyz_min(3):(radius*2*sqrt(6)/3):xyz_max(3);
            [X,Y,Z] = meshgrid(Ax,Ay,Az);
            X(2:2:end,:) = X(2:2:end,:) + radius;
            X(:,:,2:2:end) = X(:,:,2:2:end) + radius;
            Y(:,:,2:2:end) = Y(:,:,2:2:end) + radius*sqrt(3)/3;

            % Vectorize the lattice:
            X = X(:);
            Y = Y(:);
            Z = Z(:);

            % Check which cubes are inside the defined polygon:
            in = inpolyhedron(self.faces,self.vertices_init, [X,Y,Z]);

            % Rescale and retest to find the shell:
            scale = (max(max(self.vertices_init))-4*radius)/max(max(self.vertices_init));
            in2   = inpolyhedron(self.faces,scale*self.vertices_init, [X,Y,Z]);
            shell = ~(in&in2);

            % Calculate the data required to represent the spheres:
            origins  = [X(in),Y(in),Z(in)];
            spheres = Spheres(origins,radius,'Shell',shell(in));
        end
        
        function [cubes] = densePackUniformCubes(self,varargin)
            % Parse the inputs:
            defaultDimension = 1;
            validDimension = @(x) isnumeric(x) && numel(x) == 1 && x>0;
            p = inputParser;
                addParameter(p,'Dimension',defaultDimension,validDimension);
            parse(p,varargin{:});
            dimension = p.Results.Dimension;
            
            % Get a padded range of x,y,z values to use:
            xyz_max = max(self.vertices_init)+dimension;
            xyz_min = min(self.vertices_init)-dimension;

            % Generate grid of cubes:
            gridX = xyz_min(1):dimension:xyz_max(1);
            gridY = xyz_min(2):dimension:xyz_max(2);
            gridZ = xyz_min(3):dimension:xyz_max(3);
            [X,Y,Z] = meshgrid(gridX,gridY,gridZ);
            X = X(:);
            Y = Y(:);
            Z = Z(:);

            % Check which cubes are inside the defined polygon:
            in = inpolyhedron(self.faces,self.vertices_init, [X,Y,Z]);

            % Rescale and retest to find the shell:
            scale = (max(max(self.vertices_init))-2*dimension)/max(max(self.vertices_init));
            in2   = inpolyhedron(self.faces,scale*self.vertices_init, [X,Y,Z]);
            shell = ~(in&in2);

            % Generate the output cubes:
            origins  = [X(in),Y(in),Z(in)];
            cubes = Cubes(origins,dimension,'Shell',shell(in));
        end
    end

    methods (Access = public)           
        function [hit,intersection,normal] = rayTrace(self,ray)
            hit = false;
            if ~isempty(self.bvh)
                % Rotate the ray into the body frame (for use with BVH):
                ray(1:3,:) = self.inertial2self*ray(1:3,:);
                ray(4:6,:) = self.inertial2self*ray(4:6,:);

                % Get the set of triangles to test the ray against:
                triangles = self.bvh.rayCast(ray);
                normal = nan;
                if ~isempty(triangles)
                    % Obtain the definition for the remaining triangles:
                    f = self.faces(triangles,:);
                    scale = 1 + 1e-1;
                    vert1 = scale*(self.vertices(f(:,1),:) - self.face_centers(triangles,:)) + self.face_centers(triangles,:);
                    vert2 = scale*(self.vertices(f(:,2),:) - self.face_centers(triangles,:)) + self.face_centers(triangles,:);
                    vert3 = scale*(self.vertices(f(:,3),:) - self.face_centers(triangles,:)) + self.face_centers(triangles,:);

                    % Trace ray with remaining triangles:
                    [intersect,t,u,v,xcoor] = TriangleRayIntersection(ray(1:3), ray(4:6), vert1, vert2, vert3);

                    % Get the closest ray intersection:
                    if any(intersect)
                        hit = true;
                        tri = triangles(intersect);
                        dist = t(intersect);
                        u = u(intersect);
                        v = v(intersect);
                        coord = xcoor(intersect,:);
                        closest = dist == min(dist);
                        intersection = coord(closest,:);
                        u = u(closest);
                        v = v(closest);
                        f = self.faces(tri(closest),:);

                        if sum(closest) > 1
                            intersection = intersection(1,:);
                            u = u(1);
                            v = v(1);
                            f = f(1,:);
                        end

                        % Get the vertices and vertex normals:
                        normals = self.vertex_normals(f,:);

                        % Interpolate to get normal at intersection point:
                        normal = (1-u-v)*normals(1,:) + u*normals(2,:) + v*normals(3,:); 

                        % Transpose for consistency:
                        intersection = intersection';
                        normal = normal';
                    else
                        intersection = nan;
                    end
                else
                    intersection = nan(1,3);
                end

                % Rotate back into inertial space:
                intersection = self.inertial2self'*intersection;
                normal = self.inertial2self'*normal;
            else
                % Trace without the BVH:
                vert1 = self.vertices(self.faces(:,1),:);
                vert2 = self.vertices(self.faces(:,2),:);
                vert3 = self.vertices(self.faces(:,3),:);
                [intersect,t,~,~,xcoor] = TriangleRayIntersection(ray(1:3), ray(4:6), vert1, vert2, vert3);

                % Get the closest ray intersection:
                if any(intersect)
                    hit = true;
                    normal = self.face_normals(intersect,:);
                    dist = t(intersect);
                    coord = xcoor(intersect,:);
                    closest = dist == min(dist);
                    intersection = coord(closest,:);
                    normal = normal(closest,:);
                    if sum(closest) > 1
                        intersection = intersection(1,:);
                    end

                    % Transpose for consistency:
                    intersection = intersection';
                    normal = normal';
                else
                    intersection = nan;
                end
            end
        end
    end
    
    methods (Access = public)
        % Methods for visualization:
        function [] = reset(self)
            self.vis = [];
        end
        
        function [] = draw(self,varargin)
            if isempty(self.vis)
                self.vis.handle = patch('Faces',self.faces,'Vertices',self.vertices,varargin{:});  hold on
            else
                set(self.vis.handle,'Vertices',self.vertices);
            end
            axis equal
            rotate3d on
        end
        
        function [h] = drawFaceNormals(self,varargin) 
            h = quiver3(self.face_centers(:,1),self.face_centers(:,2),self.face_centers(:,3),...
                        self.face_normals(:,1),self.face_normals(:,2),self.face_normals(:,3),varargin{:}); hold on
            axis equal
        end
        
        function [h] = drawVertexNormals(self,varargin)
            h = quiver3(self.vertices(:,1),self.vertices(:,2),self.vertices(:,3),...
                        self.vertex_normals(:,1),self.vertex_normals(:,2),self.vertex_normals(:,3),varargin{:}); hold on
            axis equal
        end
    end
    
    methods (Access = private)
        function [] = calculateNormals(self)
            if isempty(self.face_normals)
                mesh_struct.faces = self.faces;
                mesh_struct.vertices = self.vertices;
                self.face_normals = COMPUTE_mesh_normals(mesh_struct);
            end
            self.vertex_normals = STLVertexNormals(self.faces, self.vertices, self.face_normals);
        end
        
        function [] = calculateFaceCenters(self)
            for ii = 1:size(self.faces,1)
                v = zeros(1,3);
                for jj = 1:size(self.faces,2)
                    v = v+self.vertices(self.faces(ii,jj),:);
                end
                self.face_centers(ii,:) = v/size(self.faces,2);
            end
        end
    end
end