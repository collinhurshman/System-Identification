function [xf,zf,rtCoeffs] =  RFT(tVec,wVec,X,Z,FoFac)
j= sqrt(-1);
dt = tVec(2) - tVec(1);
C = exp(-j*wVec*tVec(1));
dC = exp(-j*wVec*dt);
xrt = zeros(size(X));
for i=1:size(X,2)%remove offset bias from regressors
  xrt(:,i) = X(:,i) - mean(X(:,i));
end
zrt = Z - mean(Z);%remove offset from response
xf = zeros(length(wVec),size(X,2)-1);%no bias column in freq domain
zf = 0;
xrt = xrt(:,2:size(X,2));%exclude first column
rtCoeffs = zeros(length(tVec),size(X,2)-1);
for i=1:length(tVec)%iterate through time series
  xCur = xrt(i,:);
  zCur = zrt(i);
  xf = FoFac*xf + C.'*xCur;
  zf = zf + C.'*zCur;
  C = C.*dC;
  rtCoeffs(i,:) = real(xf'*xf)^-1*real(xf'*zf);
end