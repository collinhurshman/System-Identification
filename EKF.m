function [stateEstimates] = EKF(models,inputs,Jac, G,Xk,Q,R,Pk,tVec,Xmeas)
  %{
  Basic Extended Kalman Filter...
  *models = 1xn vector of state equations... assign it as below:
  models = @(stateVec,inputVec) [stateVec(1); -2*stateVec(2)^2*inputVec(1) - 3]
  NOTE: inputs can be incorporated into these modeled, and passed as parameters...
  by setting the parameter below...
  *inputs = time series of inputs in their own columns
  *Jac = nxn mat of anonymous fns. jacobian matrix of state models (partial derivatives)
  %may be possible to automate creation of Jac from models
  *G = [n measured x n]matrix mapping known measurements to states
  *Xk = n column vector, initial prediction of the states( may use I.C.s or an arbitrary 
number...
  ...if inaccuracy tolerable at the start
 *Q (nxn), R (nmeasured x nmeasured) = variances of model and measurements, respectively - 
TUNABLE PARAMETERS
  *Pk = nxn, initial prediction of error covar mat... identity may be sufficient
  %}
  Xkpred(1,:) = Xk.';
  for k=2:length(tVec)
    %prediction
    Xk = models(Xkpred(k-1,:),inputs(k-1,:));
    F = Jac(Xkpred(k-1,:));%pass in previous state estimates to the linearizing matrix...
    %correction
    Pk = F*Pk*F.' + Q;%use previous Pk to find new Pk...
    Zk = Xmeas(k,:).';
    Yk = Zk - G*Xk;% OLD VERSION: Yk = Zk - G*Xkpred(k-1,:).';
    Sk = G*Pk*G.' + R;
    Kk = Pk*G.'*Sk^-1;
    Xkpred(k,:) = (Xk + Kk*Yk).';
    Pk = (eye(length(Xk)) - Kk*G)*Pk;
  end
stateEstimates = Xkpred;