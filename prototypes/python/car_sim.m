def car_sim(z_curr, u_curr, params):

  %% CAR_SIM simulates a car-like vehicle

  v = params(0)
  ts = params(1)
  l_r = params(2)
  l_f = params(3)

  beta = np.atan2(l_r/(l_r + l_f) * np.tan(u_curr));

  % vehicle dynamics equations
  z_new(1) = z_curr(1) + (v*cos(z_curr(3)+beta))*params.Ts;
  z_new(2) = z_curr(2) + (v*sin(z_curr(3)+beta))*params.Ts;
  z_new(3) = z_curr(3) + (v*sin(beta)/params.l_r)*params.Ts;

  return np.matrix([[],[],[]])
