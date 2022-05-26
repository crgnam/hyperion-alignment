function [aa_limb] = anti_alias_outline(outline)
    tp = outline(:,1);
    remove = nan(size(outline,2),1);
    kk = 1;
    on_line = false;
    for ii = 2:size(outline,2)
        if any(outline(:,ii) == tp)
            remove(kk) = ii;
            kk = kk+1;
            on_line = true;
        else
            tp = outline(:,ii);
            if on_line
                remove(kk-1) = [];
            end
            on_line = false;
        end
    end
    remove(isnan(remove)) = [];
    aa_limb = outline;
    aa_limb(:,remove) = [];
end