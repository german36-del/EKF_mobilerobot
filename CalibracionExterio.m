for i=1:100

res=apoloGetLaserLandMarks('LMS100'); 
angulo(i)=res.angle;
distancia(i)=res.distance;
anguloreal=pi/4;
distancia_real=5.4452;


end