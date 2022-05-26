classdef (Abstract) CameraModel < handle
    methods (Abstract)
        [rays]   = pixels_to_rays(self,pixels)
        [pixels] = points_to_pixels(self,points)
    end
end