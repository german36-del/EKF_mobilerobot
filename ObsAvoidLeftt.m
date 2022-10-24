%apoloMoveMRobot('Marvin',[0.2,0],5);
%apoloMoveMRobot('Marvin',[0,-0.1],10);
%apoloPlaceMRobot('Marvin',[-2 0 0],0)
%disFront=apoloGetAllultrasonicSensors('Marvin');

%%


clear all
apoloPlaceMRobot('Marvin',[-2 0 0],0) %initialise position

%%
velocidadL = 0.2; 
velocidadA = 0.1; 
tiempo = 10;
maxDistanceBeforeObstacle=4;
angleAroundObstacle=30;

map=binaryOccupancyMap;
ground=apoloGetLocationMRobot('Marvin');  
occVal=checkOccupancy(map,[ground(1) ground(2)]);  %not to consider walls as obstacles for example

disFront=apoloGetAllultrasonicSensors('Marvin');
groundTurn=ground; 

if occVal < 1
if abs(disFront(1)) < maxDistanceBeforeObstacle
    if abs(groundTurn(4)) < abs(ground(4))+angleAroundObstacle    %turn 30° to the left

        apoloMoveMRobot('Marvin',[0,velocidadA],tiempo);
        groundTurn=apoloGetLocationMRobot('Marvin');

    end
        
        dist=0;
        if dist <= maxDistanceBeforeObstacle/cos(angleAroundObstacle)  %progress toward the obstacle

            apoloMoveMRobot('Marvin',[velocidadL,0],tiempo);
            groundU=apoloGetLocationMRobot('Marvin');
            dist=sqrt((groundU(1)-ground(1))^2+(groundU(2)-ground(2))^2);

        end
        
        groundTurnBack=apoloGetLocationMRobot('Marvin');
        if groundTurnBack(4) > ground(4)   %adjust orientation back to normal

            apoloMoveMRobot('Marvin',[0,-velocidadA],tiempo);
            groundTurnBack=apoloGetLocationMRobot('Marvin');
        end
        
        apoloMoveMRobot('Marvin',[velocidadL,0],tiempo);
        disL=apoloGetAllultrasonicSensors('Marvin');
        if disL(3) < maxDistanceBeforeObstacle    %progress and pass obstacle, checking we have passed it

            apoloMoveMRobot('Marvin',[velocidadL,0],tiempo);
            disL=apoloGetAllultrasonicSensors('Marvin');
        end
        
       groundAdjust=apoloGetLocationMRobot('Marvin');
        if groundAdjust(4)>ground(4)-angleAroundObstacle %adjust orientation to go back to inital orientation

            apoloMoveMRobot('Marvin',[0,-velocidadA],tiempo);
            groundAdjust=apoloGetLocationMRobot('Marvin');

        end

        dist2=0;
        if dist2<maxDistanceBeforeObstacle/cos(angleAroundObstacle) 

            apoloMoveMRobot('Marvin',[velocidadL,0],tiempo);
            groundU2=apoloGetLocationMRobot('Marvin');
            dist2=sqrt((groundU2(1)-groundAdjust(1))^2+(groundU2(2)-groundAdjust(2))^2);

        end

         groundU2=apoloGetLocationMRobot('Marvin');

        if groundU2(4) < ground(4)

            apoloMoveMRobot('Marvin',[0,0.1],tiempo);
            groundU2=apoloGetLocationMRobot('Marvin');

        end
       disFront=apoloGetAllultrasonicSensors('Marvin');
    
end
end

