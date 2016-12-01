function z_new = car_sim_circular(z_curr, u_curr, params)
    
  delta = u_curr;
  
  % vehicle dynamics equations
  z_new(1) = z_curr(1) + (params.v0*cos(z_curr(3)+atan((params.l_r/(params.l_r+params.l_f))*tan(delta))))*params.Ts;
  z_new(2) = z_curr(2) + (params.v0*sin(z_curr(3)+atan((params.l_r/(params.l_r+params.l_f))*tan(delta))))*params.Ts;
  z_new(3) = z_curr(3) + (params.v0 / params.l_r * sin(atan((params.l_r/(params.l_r+params.l_f))*tan(delta))))*params.Ts;

end