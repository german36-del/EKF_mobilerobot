function [] = untitled2()
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
            res=apoloGetUltrasonicSensor('u0');



       i=i+1;
    end
   
