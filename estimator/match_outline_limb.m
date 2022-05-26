function [rms,R,t] = match_outline_limb(outline, limb)
    % Initialize:
    N = size(limb,2);
    rms = inf;
    R = eye(3);
    t = zeros(2,1);
    for ii = 1:size(outline,2)
        % Get the current hypothesis arc:
        outline_arc = get_arc(outline, ii, N);

        % Perform the alignment:
        [regParams,~,ErrorStats] = absor(outline_arc,limb, 'doScale',0);
        lsq_error = ErrorStats.errlsq;
        if lsq_error < rms
            rms = lsq_error;
            R = regParams.R;
            t = regParams.t;
        end
    end
end