function z_new = car_sim(z_curr, u_curr, params)
  %% CAR_SIM simulates a car-like vehicle
  %
  % imposing the phisical saturation on the inputs
  v            = params.v0;
  delta        = u_curr;

  beta = atan(params.l_q * tan(delta));

  % vehicle dynamics equations
  z_new(1) = z_curr(1) + (v*cos(z_curr(3)+beta))*params.Ts;
  z_new(2) = z_curr(2) + (v*sin(z_curr(3)+beta))*params.Ts;
  z_new(3) = z_curr(3) + (v*sin(beta)/params.l_r)*params.Ts;


end