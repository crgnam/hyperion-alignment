function [] = draw_density(x,y, x_range, y_range)
    % Bin the data:
    N = histcounts2(y(:), x(:), y_range, x_range);

    % Create Gaussian filter matrix:
    [xG, yG] = meshgrid(-5:5);
    sigma = 2.5;
    g = exp(-xG.^2./(2.*sigma.^2)-yG.^2./(2.*sigma.^2));
    g = g./sum(g(:));

    % Plot heatmap:
    imagesc(x_range, y_range, conv2(N, g, 'same')); hold on
%     scatter(x,y, '.k');
    axis equal;
    set(gca,'YDir', 'normal');
    title('Distribution of Particles')
end