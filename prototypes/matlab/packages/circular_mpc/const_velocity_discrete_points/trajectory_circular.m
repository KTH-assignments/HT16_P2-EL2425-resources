%% Creates the Trajectory to Follow
function trajectory_circular = trajectory_circular(trajectory_params)
% Center of the circular trajectory 
  x_o = trajectory_params.x_o;
  y_o = trajectory_params.y_o;
  r = trajectory_params.r;

% Creates the circular trajectory and desired corresponding angle
  for i = 0:1:360
    t = pi * i / 180;
    
    x = x_o + r*cos(t);
    y = y_o + r*sin(t);
    theta = pi/2 + t;

    trajectory_circular(i+1, 1) = x;
    trajectory_circular(i+1, 2) = y;
    trajectory_circular(i+1, 3) = theta;
  end  
end