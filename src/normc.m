function [M,n] = normc(M)
    %@code{true}
    % Highly vectorized normalization of columns
    n = sqrt(sum(M.^2,1));
    M = bsxfun(@rdivide,M,n);
end