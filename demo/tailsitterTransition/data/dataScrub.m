load('CM_LUT') 
momentLUT(:,1) = momentLUT(:,1)+7 ; 
momentLUT(:,2) = momentLUT(:,2)-momentLUT(1,2)

newAlpha = [0:1:90]' ; 
newCm = interp1(momentLUT(:,1),momentLUT(:,2),newAlpha) 
%% plot
plot(newAlpha,newCm,'ro'); hold on; grid on; 
% plot(newAlpha,newCm,'k--')

%% fit 
P = polyfit(newAlpha,newCm,1) ; 
Peval = polyval(P,newAlpha) ; 

plot(newAlpha,Peval)

%% 
alpha_LUT = aero_data_0012(:,1) ; 
P = [0.005, 0] ; 
CM_eval = polyval(P,alpha_LUT) ; 