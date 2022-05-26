function [aa_limb] = anti_alias_limb(limb)
    tp = limb(:,1);
    remove = nan(size(limb,2),1);
    kk = 1;
    for ii = 2:size(limb,2)
        if any(limb(:,ii) == tp)
            remove(kk) = ii;
            kk = kk+1;
        else
            tp = limb(:,ii);
        end
    end
    remove(isnan(remove)) = [];
    aa_limb = limb;
    aa_limb(:,remove) = [];
end