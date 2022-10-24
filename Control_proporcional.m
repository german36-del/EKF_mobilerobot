%%Script para el control reactivo.

Xref=[]
Xref(:,1)=pthObj.States(:,1);
Xref(:,2)=pthObj.States(:,2);
u=0.1;
r=0;
a=apoloGetOdometry('Marvin')
Xact=a;
i=0;
K_a_p=0.0001;
K_l_p=0.0001;

for s=1:length(Xref)

a=apoloGetOdometry('Marvin');
errorL=sqrt(((a(1)-Xref(s,1))^2)+(a(2)-Xref(s,2))^2);
errorA=atan2(Xref(s,2),Xref(s,1))-a(3);

    while abs(errorA)>0.001
        errorA=atan2(Xref(s,2),Xref(s,1))-a(3);
        velocidadA=K_a_p*errorA;
        apoloMoveMRobot('Marvin', [0, velocidadA], 0.1);
        a=apoloGetOdometry('Marvin');
        Xact=[Xact; a];
        i=i+1;
        apoloUpdate()
    end
    t=atan2(Xref(s,2),Xref(s,1))
    w=a(3)
a=apoloGetOdometry('Marvin')
err_x=(a(1)-Xref(s,1));
        err_y=(a(2)-Xref(s,2));
    while abs(err_x)&&abs(err_y)>0.09
        err_x=(a(1)-Xref(s,1));
        err_y=(a(2)-Xref(s,2));
        errorL=sqrt(((a(1)-Xref(s,1))^2)+(a(2)-Xref(s,2))^2);
        velocidadL=K_l_p*errorL;
        apoloMoveMRobot('Marvin', [velocidadL, 0], 0.1);
        a=apoloGetOdometry('Marvin');
        Xact=[Xact; a];
        apoloUpdate()
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
