
% This function applies an inverse polar transformation to an image with
% circular structures. [Cx, Cy] is the center location of both of the
% circles, while InnerRadius is the radius for the small circular arc,
% while OuterRadius is for the big one. The transformation is applied
% starting from angle Phi0 to PhiEnd (in radians). 
% The result IWarp is a linearized image, in which the circular elements
% are mapped onto linear ones. If the parameter Visualize is set to true,
% then the image with polar region as well as the output are plotted.
% The function only warps the region remaining within the torus lying
% within the two circles.
% Note that I is expected to be a gray valued image. The coordinate axis
% respects the mathematical convention. This is why angular mappings are
% also defind in the similar fashion. Yet, [Cx, Cy] are in pixel
% coordinates.
% Usage (Parameters are illustrative only) : 
% fromRadius = 205.0;
% toRadius = 676.0;
% centerX = 802.0;
% centerY = 550.0;
% fromAngle = pi/2;
% toAngle = pi+pi/2;
% transImageInvPolar(I, centerX, centerY, fromRadius, toRadius, fromAngle, toAngle, 1)
% Author: Tolga Birdal

function [IWarp] = transImageInvPolar(I, Cx, Cy, InnerRadius, OuterRadius, Phi0, PhiEnd, Visualize)

% copy parameters
cx = Cx; cy = Cy;
out = OuterRadius;
in = InnerRadius;
phi0 = Phi0;
phiEnd = PhiEnd;

% compute boundaries and dimensions
M=out-in+1; %R
rMin=in/out;
rMax=1;

% sampling for the angle
s=out;
r=linspace(rMin, rMax, M);
N=round(out*(phiEnd-phi0));

th=linspace(phi0, phiEnd, N);

% meshgrid in polar space
[TH, R]=meshgrid(-th,r);

% compute the correspondent coordinates
X=R.*cos(TH);
Y=R.*sin(TH);
X=X*s+cx;
Y=Y*s+cy;

% warp
IWarp = interp2(I, X,Y, 'nearest', 0);

if (Visualize)
    figure, imshow(I, []);
    hold on, viscircles([cx cy; cx cy], [in out]);
    figure, imshow(IWarp,[]);
end


end
