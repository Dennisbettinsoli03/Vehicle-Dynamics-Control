function F = nlcon(x,y)
% NMPC constraint function
% x: state of the system
% y: output of the system (y(1)=X, y(2)=Y)

% Richiama le coordinate dell'ostacolo definite nel main
global Co 

% La consegna richiede: F(x, y) = 2.5 - ||(X, Y) - (Xcon, Ycon)|| <= 0
% Usiamo il teorema di Pitagora (distanza euclidea)
distance = sqrt((y(1) - Co(1))^2 + (y(2) - Co(2))^2);
F(1,1) = 2.5 - distance;

end
