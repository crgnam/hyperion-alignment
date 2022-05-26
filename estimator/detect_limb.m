function [limb_out] = detect_limb(img,limb_width,output_width)
    mask = img(:,:,1)>1;
    
    if nargin == 1
        limb_width = 1;
        output_width = 1;
    elseif nargin == 2
        output_width = limb_width;
    end
    
    % For now, it is assuming the vec_to_sun is along the x-axis of the
    limb = zeros(size(mask));
    for row = 1:size(mask,1)
        if any(mask(row,:))
            limb(row, find(mask(row,:),limb_width,'last')) = 1;
        end
    end
    
    % Extract only the connected parts of the limb:
    limb = ExtractNLargestBlobs(limb, 1);
    
    if output_width ~= limb_width
        limb_out = zeros(size(limb));
        for row = 1:size(limb,1)
            if any(limb(row,:))
                limb_out(row, find(limb(row,:),output_width,'last')) = 1;
            end
        end
    else
        limb_out = limb;
    end
end