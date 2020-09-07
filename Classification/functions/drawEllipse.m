function rotated_ellipse = drawEllipse(parameters,N, visualize)
    cos_phi = cos(parameters.phi);
    sin_phi = sin(parameters.phi);
    a = parameters.a;
    b = parameters.b;
    X0 = parameters.X0;
    Y0 = parameters.Y0;
    % rotation matrix to rotate the axes with respect to an angle phi
    R = [ cos_phi sin_phi; -sin_phi cos_phi ];
    % the ellipse
    theta_r         = linspace(0,2*pi,N);
    ellipse_x_r     = X0 + a*cos( theta_r );
    ellipse_y_r     = Y0 + b*sin( theta_r );
    rotated_ellipse = R * [ellipse_x_r;ellipse_y_r];
    
    if visualize
        plot( rotated_ellipse(1,:),rotated_ellipse(2,:),'r' );
    end
end