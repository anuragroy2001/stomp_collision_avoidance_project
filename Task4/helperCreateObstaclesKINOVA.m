%% Construct voxel obstacle representation for STOMP planner
% Define the workspace (the voxel world limits). Depending on the robot's reach range.
Env_size = [-1, -1, -1; 2, 2, 2];  % [xmin, ymin, zmin] for 1st row, xyz-lengths for 2nd row
voxel_size = [0.02, 0.02, 0.02];  % unit: m
% Binary map: all free space initially, 
binary_world = zeros(Env_size(2, 1) / voxel_size(1), Env_size(2, 2) / voxel_size(2), Env_size(2, 3) / voxel_size(3));
% binary_world_offset = Env_size(1, :)./ voxel_size;
%% XYZ metric representation (in meter) for each voxel 
% !!!!Watch out for the useage of meshgrid: 
% [X,Y,Z] = meshgrid(x,y,z) returns 3-D grid coordinates defined by the 
% vectors x, y, and z. The grid represented by X, Y, and Z has size 
% length(y)-by-length(x)-by-length(z).
% The 3D coordinate is of the center of the voxels
[Xw, Yw, Zw] = meshgrid(Env_size(1, 1) + 0.5 * voxel_size(1) : voxel_size(1) : Env_size(1, 1) + Env_size(2, 1) - 0.5 * voxel_size(1), ...
       Env_size(1, 2) + 0.5 * voxel_size(2) : voxel_size(2) : Env_size(1, 2) + Env_size(2, 2) - 0.5 * voxel_size(2), ...
    Env_size(1, 3) + 0.5 * voxel_size(3) : voxel_size(3) : Env_size(1, 3) + Env_size(2, 3) - 0.5 * voxel_size(3));

%% Static obstacles
%%%%%%%%%%%%%%Scene 1$%%%%%%%%%%%%%
% box_center = [0.4 0.5 0.6]; % (metric) world coordinates of the box center
% aObs = collisionBox(0.6,0.3,0.04);  
% aObs.Pose = trvec2tform(box_center);   
% box_center = [0.4 0.5 0.2]; % (metric) world coordinates of the box center
% aObs2 = collisionBox(0.6,0.3,0.04);  
% aObs2.Pose = trvec2tform(box_center);
% box_center = [0.5 0.0 0.0]; % (metric) world coordinates of the box center
% aObs3 = collisionBox(0.3,0.3,0.3);  
% aObs3.Pose = trvec2tform(box_center);
% box_center = [0.3 -0.3 0.0]; % (metric) world coordinates of the box center
% aObs4 = collisionBox(0.5,0.04,1.0);  
% aObs4.Pose = trvec2tform(box_center);   
% world = {aObs, aObs2, aObs3, aObs4};
%%%%%%%%%%%%%%Scene 2$%%%%%%%%%%%%%

%%%%%%%%%%%%%%Scene 2$%%%%%%%%%%%%%
% box_center = [0.0 0.5 0.4]; % (metric) world coordinates of the box center
% aObs = collisionBox(0.04,0.5,0.4);  
% aObs.Pose = trvec2tform(box_center);   
% box_center = [0.0 -0.3 0.0]; % (metric) world coordinates of the box center
% aObs2 = collisionBox(0.5,0.04,1.0);  
% aObs2.Pose = trvec2tform(box_center);   
% box_center = [-0.5 0.5 0.4]; % (metric) world coordinates of the box center
% aObs3 = collisionBox(0.04,0.5,0.4);  
% aObs3.Pose = trvec2tform(box_center);   
% box_center = [0.5 0.5 0.4]; % (metric) world coordinates of the box center
% aObs4 = collisionBox(0.04,0.5,0.4);  
% aObs4.Pose = trvec2tform(box_center);   
% world = {aObs, aObs2, aObs3, aObs4};
%%%%%%%%%%%%%%Scene 2$%%%%%%%%%%%%%

%%%%%%%%%%%%%%Scene 3$%%%%%%%%%%%%%
box_center = [0.0 -0.3 0.0]; % (metric) world coordinates of the box center
aObs = collisionBox(0.5,0.04,1.0);  
aObs.Pose = trvec2tform(box_center);   
box_center = [-0.15 0.3 0.1]; % (metric) world coordinates of the box center
aObs2 = collisionBox(0.3,0.3,0.2);  
aObs2.Pose = trvec2tform(box_center);   
box_center = [0.15 0.3 0.6]; % (metric) world coordinates of the box center
aObs3 = collisionBox(0.3,0.3,0.2);  
aObs3.Pose = trvec2tform(box_center);   
world = {aObs, aObs2, aObs3};
%%%%%%%%%%%%%%Scene 1$%%%%%%%%%%%%%


%% Visulaization the obstacle
for i=1: length(world)
    show(world{i})
end

%% voxelize all obstacles in 'world'
binary_world = zeros(Env_size(2, 1) / voxel_size(1), ...
                     Env_size(2, 2) / voxel_size(2), ...
                     Env_size(2, 3) / voxel_size(3));

for i = 1:length(world)
    % Extract current obstacle
    obs = world{i};

    % Assume each obstacle is a collisionBox(lx, ly, lz)
    dims = [obs.X, obs.Y, obs.Z];  % or [obs.X, obs.Y, obs.Z] depending on your MATLAB version
    if isa(obs, 'collisionBox')
        dims = [obs.X, obs.Y, obs.Z];
    else
        error('Currently only handles collisionBox');
    end

    % Get obstacle center position in world frame
    center = tform2trvec(obs.Pose);

    % Compute metric bounds of box
    metric_min = center - dims/2;
    metric_max = center + dims/2;

    % Convert to voxel indices (ensure positive & within bounds)
    voxel_min = max( ceil((metric_min - Env_size(1,:)) ./ voxel_size), 1 );
    voxel_max = min( ceil((metric_max - Env_size(1,:)) ./ voxel_size), size(binary_world) );

    % Fill occupancy
    [xc, yc, zc] = meshgrid(voxel_min(1):voxel_max(1), ...
                             voxel_min(2):voxel_max(2), ...
                             voxel_min(3):voxel_max(3));
    ind = sub2ind(size(binary_world), xc(:), yc(:), zc(:));
    binary_world(ind) = 1;
end

% Construct signed Euclidean distance field
voxel_world_sEDT = prod(voxel_size)^(1/3) * sEDT_3d(binary_world);

voxel_world.voxel_size = voxel_size;
voxel_world.voxel = binary_world;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size = Env_size;
voxel_world.sEDT = voxel_world_sEDT;






