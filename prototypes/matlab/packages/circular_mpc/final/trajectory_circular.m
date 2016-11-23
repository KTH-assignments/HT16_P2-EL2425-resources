%% Creates the Trajectory to Follow
function trajectory_circular = trajectory_circular(trajectory_params)
% Center of the circular trajectory 
  x_o = trajectory_params.x_o;
  y_o = trajectory_params.y_o;
  r = trajectory_params.r;
  v_ref = trajectory_params.v_ref;

% Creates the circular trajectory and desired corresponding angle
  for i = 0:1:360
      x = x_o + r*cos(pi * i / 180);
      y = y_o + r*sin(pi * i / 180);
      theta = pi - atan2(x - x_o, y - y_o);

      trajectory_circular(i+1, 1) = x;
      trajectory_circular(i+1, 2) = y;
      trajectory_circular(i+1, 3) = v_ref;
      trajectory_circular(i+1, 4) = theta;
  end



  trajectory_circular(272:end,4) = trajectory_circular(272:end,4) + 2*pi;
  
  % Plotting the trajectory and desired corresponding angle
%   plot(trajectory_circular(:,1), trajectory_circular(:,2))
%   axis equal
%   figure
%   plot(trajectory_circular(:,3))

end