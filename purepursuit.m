controller = controllerPurePursuit;
waypoints=[coordenadas_def(:,1) coordenadas_def(:,2)];
controller.Waypoints = waypoints;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 1;
controller.LookaheadDistance = 0.1;

         tiempo=0.1;   
       d=100101;  
       s=1;
while d>0.1

ground_truth = apoloGetLocationMRobot('Marvin');
Xrk = [ground_truth(1) ground_truth(2) ground_truth(4)];
            Xr(s,:) = Xrk;
    [vel,angvel] = controller([ground_truth(1) ground_truth(2) ground_truth(4) ]);
    apoloMoveMRobot('Marvin',[vel, angvel], tiempo);
            apoloUpdate();

            d=sqrt((ground_truth(1)-waypoints(length(waypoints),1))^2+(ground_truth(2)-waypoints(length(waypoints),2))^2);


s=s+1;

end


figure(1);
plot(waypoints(:,1),waypoints(:,2),'b');
hold on;
plot(Xr(:,1),Xr(:,2),'r');
grid;

