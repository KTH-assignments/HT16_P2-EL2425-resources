function trajectory = trajectory()

  xc = 0;
  yc = 0;
  r = 1.5;

  trajectory = zeros(360,3);

  for i = 0:1:360
      x = xc + r*cos(pi * i / 180);
      y = yc + r*sin(pi * i / 180);
      theta = pi - atan2(x - xc, y - yc);

      trajectory(i+1, 1) = x;
      trajectory(i+1, 2) = y;
      trajectory(i+1, 3) = mod(theta, 2*pi);

  end
  
%   plot(trajectory(:,1), trajectory(:,2))
%   axis equal

end