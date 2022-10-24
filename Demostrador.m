

    
%Declaro el controlador 
controller = controllerPurePursuit;
waypoints=[coordenadas_def(:,1) coordenadas_def(:,2)];
controller.Waypoints = waypoints;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 1;
controller.LookaheadDistance = 0.1;
d=100101;

   % Consigo en primer lugar la estimacion de la odometría
    Xk = apoloGetOdometry('Marvin'); 

    % Este vector habra que ir aumentandolo o meterle ceros en funcion de
    % las balizas que veamos
    Bal = [ 5 -5;    
             5 5;
             13 0;
             15 5;
             10 17;
             5 25;
             13 25;
             13 35;
             5 35;
             5 25;
             -5 25;
             -5 35]; 

    %Varianza que nos dan los encoders
    vavance = 2.2586e-07;
    vgiro = 3.6834e-07;
    Qk = [vavance 0;
        0 vgiro];


   
    
    
    % Inicialización de la matriz P se puede hacer con cualquier valor(probar mas valores)
    Pk = [0.0005 0 0;
          0 6.3e-05 0;
          0 0 8.8e-05];
     

    %Varianza del laser tras calibracion
    vdist = 1.4481e-04;
    vang = 4.7432e-04;
    Vec=[];
    for h=1:length(Bal)
    Vec=[Vec vdist vang];
    end
    %Matriz de varianzas del sensor exteroceptivo que usaremos
    Rk =diag(Vec);
    


%Matrices complementarias para el calculo de mahalnobis, no queremos que
%interfieran los zeros del vector de medidas que vamos usando dependiendo
%de los landmarks que se vean
Hk_m=[];
Zk_m=[];
Zk_m_=[];

i=1
tiempo=0.1;
    while d>0.1
            
    
            [vel,angvel] = controller([Xk(1) Xk(2) Xk(3) ]);
            apoloMoveMRobot('Marvin',[vel, angvel], tiempo);
            apoloUpdate();

            ground_truth = apoloGetLocationMRobot('Marvin');
            %Obviamos la tercera componente porque estamos trabajando en el plano,
            %z=0
            %obtenemos ground_truth para posteriores gráficas
            Xrk = [ground_truth(1) ground_truth(2) ground_truth(4)];
            Xr(i,:) = Xrk;
  
            
            % Actualizacion de variables
            Xk_1 = Xk;
            Pk_1 = Pk;
            %Comienzo el EKF estimando posicion y pose del robot
            encoders = apoloGetOdometry('Marvin');
            avance = sqrt((Xk_1(1)-encoders(1))^2+(Xk_1(2)-encoders(2))^2);
            giro = encoders(3)-Xk_1(3);
            %Incremental del estado
            X_k = [(Xk_1(1) + avance*cos(Xk_1(3)+(giro/2)));
                   (Xk_1(2) + avance*sin(Xk_1(3)+(giro/2)));
                   (Xk_1(3) + giro)];
            
            %Jacobiano del estado respecto a las variables del proceso 
            Gk = [(cos(Xk_1(3)+giro/2)) (-1/2*avance*sin(Xk_1(3)+giro/2));
                  (sin(Xk_1(3)+giro/2)) (1/2*avance*cos(Xk_1(3)+giro/2));
                   0 1];
            %Jacobiano del estado respecto a las variables de estado
            Phik = [1 0 (-avance*sin(Xk_1(3)+giro/2));
                  0 1 (avance*cos(Xk_1(3)+giro/2));
                  0 0 1];
            %Actualizo la matriz de varianzas de la medida
            P_k = Phik*Pk_1*((Phik)') + Gk*Qk*((Gk)');
                
            %Hay dos cambios  en los signos que te matan con los angulos
            %Predicción de la medida que nos dará el laser
            Zk_=[];
            for le=1:length(Bal)
            Zk_ = [ Zk_ ; sqrt((Bal(le,1)-Xk_1(1))^2+(Bal(le,2)-Xk_1(2))^2);
                    atan2(Bal(le,2)-Xk_1(2),Bal(le,1)-Xk_1(1))-Xk_1(3)
                    ];
            end
            laser = apoloGetLaserLandMarks('LMS100');
            if length(laser.id)>=1

 
        
             
             
               
                %%OJO CAMBIAR POSICION DEL LASER EN EL XML, importante
                %%quitarle el offset para no tener que usarlo aqui
                
                 %Derivada de la prediccion respecto a las variables de
                 %estado Repasar esto
                 Hk=[];
                 for w=1:length(Bal)
                    Hk=[Hk ; -1*((Bal(w,1))-X_k(1))/(sqrt((Bal(w,1)-Xk_1(1))^2+ (Bal(w,2)-Xk_1(2))^2)) -1*((Bal(w,2))-X_k(2))/(sqrt((Bal(w,1)-Xk_1(1))^2+ (Bal(w,2)-Xk_1(2))^2)) 0;   
              ((Bal(w,2)-X_k(2))/((Bal(w,1)-X_k(1))^2+(Bal(w,2)-X_k(2))^2)) (-(Bal(w,1)-X_k(1))/((Bal(w,1)-X_k(1))^2+(Bal(w,2)-X_k(2))^2)) (-1)];
                 end
%             Hk = [-1*((Bal(1,1))-X_k(1))/(sqrt((Bal(1,1)-Xk_1(1))^2+ (Bal(1,2)-Xk_1(2))^2)) -1*((Bal(1,2))-X_k(2))/(sqrt((Bal(1,1)-Xk_1(1))^2+ (Bal(1,2)-Xk_1(2))^2)) 0;   
%               ((Bal(1,2)-X_k(2))/((Bal(1,1)-X_k(1))^2+(Bal(1,2)-X_k(2))^2)) (-(Bal(1,1)-X_k(1))/((Bal(1,1)-X_k(1))^2+(Bal(1,2)-X_k(2))^2)) (-1); -1*((Bal(2,1))-X_k(1))/(sqrt((Bal(2,1)-Xk_1(1))^2+ (Bal(2,2)-Xk_1(2))^2)) -1*((Bal(2,2))-X_k(2))/(sqrt((Bal(2,1)-Xk_1(1))^2+ (Bal(2,2)-Xk_1(2))^2)) 0;   
%               ((Bal(2,2)-X_k(2))/((Bal(2,1)-X_k(1))^2+(Bal(2,2)-X_k(2))^2)) (-(Bal(2,1)-X_k(1))/((Bal(2,1)-X_k(1))^2+(Bal(2,2)-X_k(2))^2)) (-1) ];
             %Computamos que landmarks ve el sensor para poner a 0 o a su
             %valor respectivo
            r=0;
            for k=1:length(laser.id)
                
                   if laser.angle(k)>(-pi)
                 r(k)=laser.id(k);
                   end
            end
            %Vector es un vector que debe llevar los indices de los
            %landmarks que haya en el mapa
            Zk=zeros(2*length(Bal),1);
            vector=[1 2];
            for l=1:length(r)
                for s=1:length(vector)
                    if r(l)==vector(s)
                        vector(s)=0;
                    end

                end
            end
            %Solo usaremos las medidas que nos de realmente el laser
            for p=1:length(r)

                
                    Zk(2*r(p),1)=laser.angle(p);
                    Zk(2*r(p)-1,1)=laser.distance(p);
                    
                    
            end
            %La matriz de observacion debe ser modificada de la misma
            %manera
            for t=1:length(vector)
                if vector(t)~=0
                    for x=1:3
                        Hk(2*t,x)=0;
                        Hk(2*t-1,x)=0;
                    end
                end
             end
            
            for ju=1:length(Zk)

                if Zk(ju)==0
                    Zk_(ju)=0;
                    

                end


            end
            %Derivada de la prediccion respecto a las variables de estado
            %Hk = [-1*((Bal(1,1))-X_k(1))/(sqrt((Bal(1,1)-Xk_1(1))^2+ (Bal(1,2)-Xk_1(2))^2)) -1*((Bal(1,2))-X_k(2))/(sqrt((Bal(1,1)-Xk_1(1))^2+ (Bal(1,2)-Xk_1(2))^2)) 0;   
              %((Bal(1,2)-X_k(2))/((Bal(1,1)-X_k(1))^2+(Bal(1,2)-X_k(2))^2)) (-(Bal(1,1)-X_k(1))/((Bal(1,1)-X_k(1))^2+(Bal(1,2)-X_k(2))^2)) (-1) ];

            

            %Matrices complementarias para calcular distancia de mahalobis
            %limpia
            Zk_m_=[];
            Hk_m=[];
            Zk_m=[];
            for p=1:length(r)
                    Zk_m_=[Zk_m_; Zk_(2*r(p)-1) ; Zk_(2*r(p))];
                
                    Zk_m=[Zk_m ; laser.distance(p) ; laser.angle(p)];
                    
                    Hk_m=[Hk_m ; [-1*((Bal(r(p),1))-X_k(1))/(sqrt((Bal(r(p),1)-Xk_1(1))^2+ (Bal(r(p),2)-Xk_1(2))^2)) -1*((Bal(r(p),2))-X_k(2))/(sqrt((Bal(r(p),1)-Xk_1(1))^2+ (Bal(r(p),2)-Xk_1(2))^2)) 0;   
              ((Bal(r(p),2)-X_k(2))/((Bal(r(p),1)-X_k(1))^2+(Bal(r(p),2)-X_k(2))^2)) (-(Bal(r(p),1)-X_k(1))/((Bal(r(p),1)-X_k(1))^2+(Bal(r(p),2)-X_k(2))^2)) (-1) ];
];
            end
            %Vector de innovacion de la medida para mahalanobis y normal
            Muk = Zk-Zk_;
            Muk_m=Zk_m_-Zk_m;

          %Trabajamos solo en el primer y cuarto cuadrante

          for te=2:2:length(Muk)
            if Muk(te)>pi
                Muk(te) = Muk(te) - 2*pi;
                
            end
            if Muk(te)<(-pi)
                Muk(te) = Muk(te) + 2*pi;
            end
          end


            Rk_m=Rk(1:2*length(r),1:2*length(r));
            %Matriz de varianzas de mu
            Sk_m = Hk_m*P_k*((Hk_m)') + Rk_m;
            Sk = Hk*P_k*((Hk)') + Rk;

            %Ganancia de kalman
             Wk = P_k*((Hk)')*inv(Sk);

            %Distancia de mahalanobis
          Dm=Muk_m'*inv(Sk_m)*Muk_m;

             
            %En las tablas es lo que pone para dos grados de libertad y 99%
           chi2=13.8;
             if Dm>chi2

                 Pk=P_k;
                 Xk=X_k;
 
             else
            %Consigo la estimacion del estado
            Xk = X_k + Wk*Muk;

             
            %Actualizo P, la P aumenta en muchos de los casos porque la
            %distancia de mahalanobis es muy restrictiva, en el primer paso
            %siempre aumenta la P y despues se reduce.
            %Hay que estudiar si esto es correcto o habria que reducir el
            %umbral
            Pk = P_k-Wk*Hk*P_k;

             end

           %Solo computo el filtro de kalman si veo un landmark como
           %minimo, si no solo odometria
            else
                Xk=X_k;
                   Pk=P_k;

            
            end
            apoloResetOdometry('Marvin', [Xk(1) Xk(2) Xk(3)]);
    
            Xest(i,:) = Xk;
            Pacumulado(1,i) = Pk(1,1);
            Pacumulado(2,i) = Pk(2,2);
            Pacumulado(3,i) = Pk(3,3);
            go(1)=coordenadas_def(length(coordenadas_def),1);
            go(2)=coordenadas_def(length(coordenadas_def),2);
            d=sqrt((go(1)-Xk(1))^2+(go(2)-Xk(2))^2);

%Compruebo si es necesario el control reactivo
                    flag=0;
res=apoloGetUltrasonicSensor('u0');

occupied = checkOccupancy(MAP,[0.2*cos(Xest(3))+Xest(1)+5 0.2*sin(Xest(3))+Xest(2)+5]);
%En caso de que la distancia sea menor que un limite y no sea un obstaculo
%ya modelado esquivo el objeto
if (res<0.2 && occupied==0)

while flag==0
for s=1:150
apoloMoveMRobot('Marvin',[0, 0.1], 0.1);
apoloUpdate();
%Llamo a función de estimación de la pose para poder seguir manteniendo un
%registro del movimiento
kalmann;
end
for t=1:40
apoloMoveMRobot('Marvin',[0.1, 0], 0.1);
apoloUpdate();
kalmann;
end

for s=1:150
apoloMoveMRobot('Marvin',[0, -0.1], 0.1);
apoloUpdate();
kalmann;
end
u=apoloGetUltrasonicSensor('u0');
t=apoloGetUltrasonicSensor('u1');


if u>0.2 && t>0.2
flag=1;
%No será necesario replanificar
%  AlgoritmoA;
%  waypoints=[coordenadas_def(:,1) coordenadas_def(:,2)];
%  controller.Waypoints = waypoints;
end



end
end



       i=i+1;
    end
   
%Calculo error cuadratico 
error2_x=(Xest(:,1)-Xr(:,1)).^2;
error2_y=(Xest(:,2)-Xr(:,2)).^2;
error2_theta=(Xest(:,3)-Xr(:,3)).^2;


figure(1);
subplot(2,2,1);
plot(Xr(:,1),Xr(:,2),'b');
xlabel('X (m)');
ylabel('Y (m)');
hold on;
plot(Xest(:,1),Xest(:,2),'g');
legend('Ground truth',' estimacion');

subplot(2,2,2);
plot(Xr(:,1),'b');
xlabel ('t (muestras)');
ylabel ('X (m)');
hold on;
plot(Xest(:,1),'g');
legend('Ground truth',' estimacion');

subplot(2,2,3);
plot(Xr(:,2),'b');
xlabel ('t (muestras)');
ylabel ('Y (m)');
hold on;
plot(Xest(:,2),'g');
legend('Ground truth',' estimacion');

subplot(2,2,4);
plot(Xr(:,3),'b');
xlabel ('t (muestras)');
ylabel ('\theta (rad)');
hold on;
plot(Xest(:,3),'g');
legend('Ground truth',' estimacion');



figure(2);
subplot(3,1,1);
axis([0 12 0 9])
plot(Pacumulado(1,:),'b','MarkerSize',2);
xlabel ('t (muestras)');
ylabel ('Varianza X (m2)');
hold on;

subplot(3,1,2);
axis([0 12 0 9])
plot(Pacumulado(2,:),'b');
xlabel ('t (muestras)');
ylabel ('Varianza Y (m2)');

subplot(3,1,3);
axis([0 12 0 9])
plot(Pacumulado(3,:),'b');
xlabel ('t (muestras)');
ylabel ('Varianza \theta (rad2)');
%%Puedes añadir graficas de mahalanobis 
%%O graficas
figure(3);
plot(error2_x,'r');
xlabel('t (muestras)');
ylabel('Magnitud del error cuadrático en estimacion');
grid;
hold on;
plot(error2_y,'g');
hold on;
plot(error2_theta,'b');
legend('Error_x','Error_y','Error_theta');





