function [x_resamp,w_resamp,index] = resample_particles(x_particle, w_particle, jitter, n)
    % Get dimensionality of particles:
    dim = size(x_particle,1);
    if nargin == 3
        n = size(x_particle,2);
    end
        
    % Cumulative Sum of Particles
    w_particle = w_particle(:);
    c = cumsum(w_particle);

    % Compute u Vector
    u = zeros(n,1);
    u(1) = n\rand(1);
    u(2:n) = u(1) + n\(1:n-1)';

    % Pre-allocate Index
    index = zeros(n,1);

    % Compute Index for Resampling 
    ii = 1;
    for jj = 1:n
        while u(jj) > c(ii)
            ii = ii+1;
        end
        index(jj) = ii;
    end

    % Resampled Data
    x_resamp = x_particle(:,index);
    w_resamp = n\ones(1,n);

    % Jitter the particles:
    x_resamp = x_resamp + jitter*randn(dim, size(x_resamp,2));
end