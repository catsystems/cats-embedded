%% 3D Model Demo
% This is short demo that loads and renders a 3D model of a human femur. It
% showcases some of MATLAB's advanced graphics features, including lighting and
% specular reflectance.

% Copyright 2011 The MathWorks, Inc.


%% Load STL mesh
% Stereolithography (STL) files are a common format for storing mesh data. STL
% meshes are simply a collection of triangular faces. This type of model is very
% suitable for use with MATLAB's PATCH graphics object.

% Import an STL mesh, returning a PATCH-compatible face-vertex structure
fv = stlread('sat_model.stl');

% fv.vertices = 5*fv.vertices;

fv.vertices = fv.vertices*5;
fv.vertices(:, 1) = fv.vertices(:, 1)-145;
fv.vertices(:, 2) = fv.vertices(:, 2)-2;
fv.vertices(:, 3) = fv.vertices(:, 3)-1;

% rot_axis = [1;0;0];
% rot_angle = 0;
% rot_quat = quaternion(cos(rot_angle*0.5), rot_axis(1)*sin(rot_angle*0.5), rot_axis(2)*sin(rot_angle*0.5), rot_axis(3)*sin(rot_angle*0.5));
% rotation_matrix = rotmat(rot_quat, 'point');
% homogenous_transform = [rotation_matrix, [0;0;0]; zeros(1,3), 1];
% vertices = [fv.vertices, ones(length(fv.vertices(:,1)), 1)]';
% transformed_vertices = [homogenous_transform*vertices]';
% fv.vertices = transformed_vertices(:,1:3);



%% Render
% The model is rendered with a PATCH graphics object. We also add some dynamic
% lighting, and adjust the material properties to change the specular
% highlighting.

sat_patch = patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
     


% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([-135 35]);

% for i = 1:90
%     rotate(sat_patch, [1,0,0], 90);
%     sat_patch.Vertices(:,1) = sat_patch.Vertices(:,1)+0.1;
%     pause(1);
% end