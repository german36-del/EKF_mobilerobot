%%Script para el control .
Xref=[0.2 0.2; 0.3 0.5; 0.1 0.6; -0.4 -0.5];
u=0.1;
r=0;
a=apoloGetOdometry('Marvin')
Xact=a;
i=0;
K_a_p=0.001;
K_l_p=0.0007;
K_l_d=0.0003;
K_l_i=0.0005;
for s=1:length(Xref)

a=apoloGetOdometry('Marvin');
errorL=sqrt(((a(1)-Xref(s,1))^2)+(a(2)-Xref(s,2))^2);
errorA=atan2(Xref(s,2),Xref(s,1))-a(3);
error_total=0;
errorL_ant=errorL;
    while abs(errorA)>0.001
        errorA=atan2(Xref(s,2),Xref(s,1))-a(3);
        velocidadA=K_a_p*errorA;
        apoloMoveMRobot('Marvin', [0, velocidadA], 0.1);
        a=apoloGetOdometry('Marvin');
        Xact=[Xact; a];
        i=i+1;
        apoloUpdate()
    end
    
    err_x=(a(1)-Xref(s,1));
    err_y=(a(2)-Xref(s,2));
    while (abs(err_x) && abs(err_y))>0.09
        err_x=(a(1)-Xref(s,1));
        err_y=(a(2)-Xref(s,2));
        errorL=sqrt(((a(1)-Xref(s,1))^2)+(a(2)-Xref(s,2))^2);
        velocidadL=K_l_p*errorL+(errorL-errorL_ant)*K_l_d+error_total*K_l_i;
        apoloMoveMRobot('Marvin', [velocidadL, 0], 0.1);
        a=apoloGetOdometry('Marvin');
        Xact=[Xact; a];
        apoloUpdate()
        errorL_ant=errorL;
        error_total=error_total+errorL;
        i=i+1;
    end

r=[r;i]
end
y=length(Xact);
u=[0.1:0.1:y*0.1];
figure(1);
grid;
plot(Xact(:,1),Xact(:,2),'r',Xref(:,1),Xref(:,2),'b');
xlabel('Eje x(m)')
ylabel('Eje y(m)')
title('Evolucion de la posicion')
