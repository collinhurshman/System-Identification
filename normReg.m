function [normZ,normR] = normReg(X,Z)
%%normalizing regressors....
normR = X;
angAcc = Z;% just a legacy name...
for j=2:size(X,2)%iterate through the regressor columns
  sjj = 0;
  for i=1:size(X,1)%iterate through all samples
    sjj = sjj + (X(i,j) - mean(X(:,j)))^2;
    normR(i,j) = X(i,j) - mean(X(:,j));
    end
  normR(:,j) = normR(:,j)/sqrt(sjj);
end
%normalizing the dependent variable...
sjDep = 0; %for the dependent variable
normAcc = angAcc;
for i =1:length(angAcc)
    sjDep = sjDep + (angAcc(i) - mean(angAcc))^2;
    normAcc(i) = angAcc(i) - mean(angAcc);
end
normAcc = normAcc/sqrt(sjDep);

%cut out first column, bias drops out when mean subtracted
normR = normR(:,2:size(X,2));
%normalizedModel...
normZ = normAcc;
return;