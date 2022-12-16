function [stateEstimates] = kalman(A,B,U,H,Xk,Q,R,Pk,tVec,Xmeas)
  %{
  Basic Kalman Filter...
  A = matrix of state functions
  B = matrix of input function coefficients
  U = input functions
  H = matrix mapping measurements to states
  Xk = initial prediction of the states( may use I.C.s or an arbitrary number...
  ...if inaccuracy tolerable at the start
  Q, R = variances of model and measurements, respectively - TUNABLE PARAMETERS
  Pk = initial prediction of error covar mat... identity may be sufficient
  Xmeas = actual measured data... provided if possible
  %}
  
  Xkstore(1,:) = Xk;%using initial guesses for first iteration...
  for k=2:length(tVec)
    Xk = A*Xk + B*U;
    Pk = A*Pk*A.' + Q;
    Zk = Xmeas(k,:).';#note that Zk is a COLUMN vector
    yk = Zk-H*Xk;
    sk = H*Pk*H.' + R;
    Kk = Pk*H.'*sk^-1;
    Xk = Xk + Kk*yk;
    Xkstore(k,:) = Xk;
    Pk = (eye(size(A,1)) - Kk*H )*Pk;
  end
stateEstimates = Xkstore;
