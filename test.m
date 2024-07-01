clc

clear

close all



%%%%%%%%%%%%%%%%%%%%%%% T6矩阵参数%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%位姿1的时候机器人末端相对于机器人基坐标系下变换矩阵

Pose1=[1141.243,-15.261,-97.721,178.91,0.47,92.37];

Px = Pose1(1);

Py = Pose1(2);

Pz = Pose1(3);

rota = Pose1(4)*pi/180;

rotb = Pose1(5)*pi/180;

rotc = Pose1(6)*pi/180;

Rx = [1 0 0; 0 cos(rota) -sin(rota); 0 sin(rota) cos(rota)];

Ry = [cos(rotb) 0 sin(rotb); 0 1 0; -sin(rotb) 0 cos(rotb)];

Rz = [cos(rotc) -sin(rotc) 0; sin(rotc) cos(rotc) 0; 0 0 1];

R1 = Rz*Ry*Rx;

T1= [Px Py Pz]';

%%%%%%%%%%位姿2的时候机器人末端相对于机器人基坐标系下变换矩阵

Pose2=[1103.946,-163.910,-107.673,-160.90,-0.14,-91.62];

Px = Pose2(1);

Py = Pose2(2);

Pz = Pose2(3);

rota = Pose2(4)*pi/180;

rotb = Pose2(5)*pi/180;

rotc = Pose2(6)*pi/180;

Rx = [1 0 0; 0 cos(rota) -sin(rota); 0 sin(rota) cos(rota)];

Ry = [cos(rotb) 0 sin(rotb); 0 1 0; -sin(rotb) 0 cos(rotb)];

Rz = [cos(rotc) -sin(rotc) 0; sin(rotc) cos(rotc) 0; 0 0 1];

R2 = Rz*Ry*Rx;

T2= [Px Py Pz]';

%%%%%%%%%%位姿3的时候机器人末端相对于机器人基坐标系下变换矩阵

Pose3=[1073.714,2.669,-142.448,-142.86,0.84,-178.55];

Px = Pose3(1);

Py = Pose3(2);

Pz = Pose3(3);

rota = Pose3(4)*pi/180;

rotb = Pose3(5)*pi/180;

rotc = Pose3(6)*pi/180;

Rx = [1 0 0; 0 cos(rota) -sin(rota); 0 sin(rota) cos(rota)];

Ry = [cos(rotb) 0 sin(rotb); 0 1 0; -sin(rotb) 0 cos(rotb)];

Rz = [cos(rotc) -sin(rotc) 0; sin(rotc) cos(rotc) 0; 0 0 1];

R3 = Rz*Ry*Rx;

T3= [Px Py Pz]';

%%%%%%%%%位姿1,2,3时候机器人末端相对于机器人基坐标系下变换矩阵

T61=[R1 T1;0 0 0 1] ;      

T62=[R2 T2;0 0 0 1];

T63=[R3 T3;0 0 0 1];
 


%%%%%%摄像机外参数矩阵（平面靶标在摄像机坐标系下表示）%%%%%%%%

Extrinsic1=[0.051678,-0.998634,0.007660,21.747985;

         -0.998617,-0.051600,0.010060,27.391246;

        -0.009651,-0.008169,-0.999920,319.071378];%%%3行4列矩阵

Extrinsic2=[0.014949,0.999738,0.017361,-35.869608 

        0.949779,-0.019626,0.312304,-20.701811

        0.312563,0.011821,-0.949823,306.463155];

Extrinsic3=[0.999176,0.039246,0.010343,-26.361812

        0.025037,-0.796606,0.603980,20.533884

        0.031943,-0.603223,-0.796933,318.110756];



%%%%%%%

TC1=[Extrinsic1; 0 0 0 1];     

TC2=[Extrinsic2; 0 0 0 1];

TC3=[Extrinsic3; 0 0 0 1];



TL1=inv(T61)*T62;

TL2=inv(T62)*T63;

 

TR1=TC1*inv(TC2);

TR2=TC2*inv(TC3);



A=[TL1,TL2];

B=[TR1,TR2];

X= tsai(A,B);

disp(X)




