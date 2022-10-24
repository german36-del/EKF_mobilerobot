pos=apoloGetOdometry('Marvin')
A=[pos]
velocidadL=0.3;
velocidadA=0;
xini = 0;
yini = 0;
thetaini = 0;
k=1;
Xrealk = [xini; yini; thetaini];
for i = 1:1000
    if i==100*k 
        velocidadL=-velocidadL
        k=k+1;
    end
 apoloMoveMRobot('Marvin', [velocidadL, velocidadA], 0.1);
 apoloUpdate()
 trayectoriaD(i) = velocidadL*0.1;
 trayectoriaB(i) = velocidadA*0.1;
 XrealkAUX = Xrealk;
 Xrealk(1) = XrealkAUX(1) + trayectoriaD(i)*cos(XrealkAUX(3)+(trayectoriaB(i)/2));
 Xrealk(2) = XrealkAUX(2) + trayectoriaD(i)*sin(XrealkAUX(3)+(trayectoriaB(i)/2));
 Xrealk(3) = XrealkAUX(3) + trayectoriaB(i);
 Xreal(:,i) = Xrealk;
 a=apoloGetOdometry('Marvin');
 A=[A ; a]; 
   

end
dif=zeros(length(A),1);
for i=1:1000
    dif(i)=Xreal(1,i)-A(i+1,1);
end
varianza=sum(dif.^2)/1000
plot(A(:,1), A(:,2),'b')