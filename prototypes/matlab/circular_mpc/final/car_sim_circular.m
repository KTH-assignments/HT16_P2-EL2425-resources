function z_new = car_sim_circular(z_curr, u_curr, params)
    
  Vref = u_curr(1);
  delta = u_curr(2);

  % vehicle dynamics equations
  z_new(1) = z_curr(1) + (z_curr(3)*cos(z_curr(4)+atan((params.l_r/(params.l_r+params.l_f))*tan(delta))))*params.Ts;
  z_new(2) = z_curr(2) + (z_curr(3)*sin(z_curr(4)+atan((params.l_r/(params.l_r+params.l_f))*tan(delta))))*params.Ts;
  z_new(3) = z_curr(3) + (Vref - z_curr(3)) / params.tau * params.Ts;
  z_new(4) = z_curr(4) + (z_curr(3)*sin(atan((params.l_r/(params.l_r+params.l_f))*tan(delta)))/params.l_r)*params.Ts;

end