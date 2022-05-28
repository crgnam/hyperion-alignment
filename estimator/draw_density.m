function [] = draw_density(particles, step_size)
    % Bin the data:
    yaw   = rad2deg(particles(1,:));
    pitch = rad2deg(particles(2,:));
    pitch_range = (-90-step_size/2):step_size:(90+step_size/2);
    yaw_range = (-step_size/2):step_size:(360+step_size/2);
    N = histcounts2(pitch, yaw, pitch_range, yaw_range);

    % Create Gaussian filter matrix:
    [xG, yG] = meshgrid(-5:5);
    sigma = 2.5;
    g = exp(-xG.^2./(2.*sigma.^2)-yG.^2./(2.*sigma.^2));
    g = g./sum(g(:));

    % Plot heatmap:
    imagesc(yaw_range, pitch_range, conv2(N, g, 'same')); hold on
    set(gca,'YDir', 'normal');
    title('Estimate Distribution')
    axis equal
    xlim([0 360])
    ylim([-90 90])
    xlabel('Yaw (deg)')
    ylabel('Pitch (deg)')
end