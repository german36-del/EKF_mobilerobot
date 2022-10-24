pos=apoloGetOdometry('Marvin')
A=[pos]

velocidadL=0.1 ;
velocidadA=0;




for i = 1:1000
    
 apoloMoveMRobot('Marvin', [velocidadL, velocidadA], 0.1);
 apoloUpdate()
 ground_truth = apoloGetLocationMRobot('Marvin');
 a=apoloGetOdometry('Marvin');
 avance_odom = sqrt((a(1))^2+(a(2))^2);
 giro_odom= a(3);

 avance_real=sqrt((ground_truth(1))^2+(ground_truth(2))^2);
 giro_real=ground_truth(4);
 apoloResetOdometry('Marvin', [ground_truth(1) ground_truth(2) ground_truth(4)]);
 Xrealk=[ground_truth(1) ground_truth(2) ground_truth(4)];
 dif_giro(i)=giro_odom-giro_real;
 dif_avance(i)=avance_real-avance_odom;
 Xreal(:,i) = Xrealk;
 
 A=[A ; a]; 
   pause;     
%%        apoloResetOdometry('Marvin', [Xk(1) Xk(2) Xk(3)]);   Si lo pongo en el filtro de kalman tambien aqui no?

end

varianza_giro=sum(dif_giro.^2)/1000
varianza_avance=sum(dif_avance.^2)/1000
figure(1);
plot(A(:,1), A(:,2),'b');
hold on;
grid;
plot(Xreal(1,:),Xreal(2,:),'r');
legend('Odometría','Ground truth');
xlabel('Eje x');
ylabel('Eje y');
figure(2);
s=dif_giro.^2;
t=dif_avance.^2;
plot(s,'r');
hold on;
plot(t,'b');
grid;
legend('Error cuadratico en giro', 'Error cuadrático en avance')
xlabel('Muestras')
ylabel('Magnitud error cuadratico,Vl=0, Va=0.4')