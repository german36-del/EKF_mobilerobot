
Bal = [ -3.9 3.9;    
             3.9 3.9       ]; 
for i=1:1000         
apoloMoveMRobot('Marvin',[0.1, 0.1], 0.1);
apoloUpdate();
ground_truth = apoloGetLocationMRobot('Marvin');
            %Obviamos la tercera componente porque estamos trabajando en el plano,
            %z=0
            %obtenemos ground_truth para posteriores gráficas
Xk_1= [ground_truth(1) ground_truth(2) ground_truth(4)];
res=apoloGetLaserLandMarks('LMS100'); 
angulo(i)=res.angle(2)     
distancia(i)=res.distance(2);
atan2(Bal(2,2)-Xk_1(2),Bal(2,1)-Xk_1(1))-Xk_1(3)
anguloreal=pi/4;
distancia_real=5.5154;
dif_angulo(i)=abs(anguloreal-angulo(i));
dif_distancia(i)=abs(distancia(i)-distancia_real);
 pause;
apoloResetOdometry('Marvin', [Xk_1(1) Xk_1(2) Xk_1(3)]);
end
s=dif_angulo.^2;
t=dif_distancia.^2;
figure(1),
plot(s,'r');
xlabel('Muestras')
ylabel('Magnitud error cuadratico del angulo')
grid;
figure(2);
plot(t,'b');
xlabel('Muestras')
ylabel('Magnitud error cuadratico de la distancia')
grid;


varianza_angulo=sum(dif_angulo.^2)/1000
varianza_distancia=sum(dif_distancia.^2)/1000