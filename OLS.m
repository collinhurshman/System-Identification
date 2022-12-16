function Coeffs = OLS(X,y)
  %where X is a n x p matrix of the p explanatory variables corresponding to...
  %the n datapoints... note X[i,1] = 1 for B0 term...
  %y is a nx1 vector of known output data
  %Coeffs returns a px1 vector
 
  Coeffs = (real(X'*X))^-1*real(X'*y);