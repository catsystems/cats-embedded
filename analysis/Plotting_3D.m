%% Plot 3D
% curve = animatedline;
Ts = 0.01;
tp = theaterPlot('XLimit',[-5 15],'YLimit',[-5 10],'ZLimit',[-5 15]);
traject_plotter = cell(1, 1);
time_string = strcat("Time : 0.00 s");
time_handle = text(-5,5,-5,time_string,'FontSize',14);
sat_name = cell(1, 1);
sat_name_handle = cell(1, 1);
for i = 1:1
    sat_name{i} = strcat("#", num2str(i));
    sat_name_handle{i} = text(6,-6,10,sat_name{i},'FontSize',14);
    traject_plotter{i} = trajectoryPlotter(tp);
    grid on;
end
sat_num = 2;
hold on;

% Load Models
satellite_stl = stlread('Recovery_FC.stl');
% satellite_stl = stlread('sat_model.stl');
[v,f] = patchslim(satellite_stl.vertices, satellite_stl.faces);
% sat2.faces = f;
% sat2.vertices = v;
% satellite_stl = sat2;

% Scale Models %
% asteroid_stl.vertices = ref_obj.asteroid_radius*4*asteroid_stl.vertices;
% satellite_stl.vertices = 5*satellite_stl.vertices;
satellite_stl.vertices = satellite_stl.vertices/10;

% Move Models to origin %
% asteroid_verteces_origin = asteroid_stl.vertices;

% asteroid_stl.vertices(:, 1) = asteroid_verteces_origin(:, 1) + 0;     % Move X Axis
% asteroid_stl.vertices(:, 2) = asteroid_verteces_origin(:, 2) + 0;     % Move Y Axis
% asteroid_stl.vertices(:, 3) = asteroid_verteces_origin(:, 3) + 0;     % Move Z Axis
% satellite_stl.vertices(:, 1) = satellite_stl.vertices(:, 1) - 145;
satellite_stl.vertices(:, 1) = satellite_stl.vertices(:, 1) - 5.5;
satellite_stl.vertices(:, 2) = satellite_stl.vertices(:, 2) - 2;
satellite_stl.vertices(:, 3) = satellite_stl.vertices(:, 3) - 1;

% Turn Satellites
rot_axis = [0;0;1];
rot_angle = pi/2;
rot_quat = quaternion(cos(rot_angle*0.5), rot_axis(1)*sin(rot_angle*0.5), rot_axis(2)*sin(rot_angle*0.5), rot_axis(3)*sin(rot_angle*0.5));
rotation_matrix = rotmat(rot_quat, 'point');
homogenous_transform = [rotation_matrix, [0;0;0]; zeros(1,3), 1];
vertices = [satellite_stl.vertices, ones(length(satellite_stl.vertices(:,1)), 1)]';
transformed_vertices = [homogenous_transform*vertices]';
satellite_stl.vertices = transformed_vertices(:,1:3);

% Save Initial positions %
% asteroid_fixed_vertices = asteroid_stl.vertices;
satellite_fixed_vertices = satellite_stl.vertices;



% Render Models
% asteroid_patch = patch(asteroid_stl,'FaceColor',       [0.8 0.8 1.0], ...
%     'EdgeColor',       'none',        ...
%     'FaceLighting',    'gouraud',     ...
%     'AmbientStrength', 0.15);

sat_patch = patch(satellite_stl,'FaceColor',       [0.8 0.8 1.0], ...
        'EdgeColor',       'none',        ...
        'FaceLighting',    'gouraud',     ...
        'AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
view([-135 20]);

buttonH = uicontrol('Style', 'togglebutton', ...
             'Units',    'pixels', ...
             'Position', [5, 5, 60, 20], ...
             'String',   'Pause', ...
             'Value',    1);

% Set Limits for theaterplot %
tp.XLimits = [-5, 5];
tp.YLimits = [-5, 5];
tp.ZLimits = [-5, 5];

position_array = [teraterm.VarName13, teraterm.VarName15, teraterm.VarName17, teraterm.VarName19]/1000;
iterations = length(position_array);
orientation = cell(1, iterations);
% Prepare Measurements %
for t = 1:iterations
        holder = normalize_q(position_array(t, :));
        orientation{1, t} = quaternion(holder(1), holder(2), holder(3), holder(4));
end

% Do the Actual Plotting %
for t = 500:20:iterations
    
        % Move and Rotate the Satellite Vertices 
        rotation_matrix = rotmat(orientation{1,t}, 'point');
        homogenous_transform = [rotation_matrix, zeros(3,1); zeros(1,3), 1];
        vertices = [satellite_fixed_vertices, ones(length(satellite_fixed_vertices(:,1)), 1)]';
        transformed_vertices = [homogenous_transform*vertices]';
        sat_patch.Vertices = transformed_vertices(:,1:3);
        
%         % Plot the trajectory
%         plotTrajectory(traject_plotter{i},  {position_traj{i}(1:t, :)})
        drawnow
        
        % Move the Names of thje satellites
%         handle_pos = position{i, t};
%         handle_pos(3) = handle_pos(3)+0.5;
%         sat_name_handle{i}.Position = handle_pos;
    
    % Update the Time string 
    time_string = strcat("Time : ", num2str(t*Ts), " s");
    time_handle.String = time_string;
%     pause(0.01);
    
        % Check if the animation is paused %
    if get(buttonH, 'Value') == 1
        while(1)
            pause(1);
            if get(buttonH, 'Value') == 0
                break;
            end
        end
    end
    
end