function [rms,R,t] = match_outline_limb(outline, limb)
    % Initialize:
    N = size(limb,2);
    rms = inf;
    R = eye(3);
    t = zeros(2,1);
    M = size(outline,2);
    span = 1:floor(M/(360/3)):M;
    for ii = span
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