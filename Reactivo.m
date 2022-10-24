

res_ant=0;
for i =1:1000
apoloMoveMRobot('Marvin',[0.1, 0], 0.1);
            apoloUpdate();
            a=apoloGetOdometry('Marvin')
            ground_truth = apoloGetLocationMRobot('Marvin')


res=apoloGetUltrasonicSensor('u0');

res_ant=res;
flag=0;
%Al final realizo la comprobacion con chekoccupancy, esto es realmente un
%script de prueba
% if (res<0.2 && MObstaculos(round(0.2*cos(a(3)+a(1))+5),round(0.2*sin(a(3))+a(2))+5)==1 && 0.2*cos(a(3))+a(1)<40 && 0.2*cos(a(3))+a(1)>0)
if (res<0.4)
while flag==0
for s=1:150
apoloMoveMRobot('Marvin',[0, 0.1], 0.1);
apoloUpdate();
a=apoloGetOdometry('Marvin');
end
for t=1:40
apoloMoveMRobot('Marvin',[0.1, 0], 0.1);
apoloUpdate();

end

for s=1:150
apoloMoveMRobot('Marvin',[0, -0.1], 0.1);
apoloUpdate();

end
u=apoloGetUltrasonicSensor('u0');
t=apoloGetUltrasonicSensor('u1');


if u>0.2 && t>0.2
flag=1;
%No va a ser necesario replanificar al final
%  AlgoritmoA;
%  waypoints=[coordenadas_def(:,1) coordenadas_def(:,2)];
%  controller.Waypoints = waypoints;
end



end
end


end
