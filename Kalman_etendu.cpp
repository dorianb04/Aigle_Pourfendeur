//Cette librairie à pour but de présenter les filtres utilisables dans le cadre du projet drone. 
//Le filtre le plus important étant le filtre de Kalman

float Q_angle = 0.001;
float Q_biais = 0.003;
float Vd = 0.03; //covariance du bruit blanc centré gaussien de l'équation de mesure. 

float biais = 0;
float Angle = 0;

float P[2][2]={{0, 0}, {0, 0}};

float Kalman (float, float, float);

float Kalman (float angle_nouveau, float gyro_measures, float dt){
//Filtre les mesures du bruit blanc gaussien centré. 
//angle_nouveau : en degrés
//gyro_measures : degrés/seconde
//dt : secondes

  float y;
  float k[2];
  float P00_int;
  float P01_int;

/////////////////Prédiction/////////////////
//Calcul de ^x(k+1|k)
  Angle+= dt*(gyro_measures - biais);

//Calcul de P(k+1|k)
  P[0][0] += dt*(dt*P[0][0] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] += -dt*P[1][1];
  P[1][0] += -dt*P[1][1];
  P[1][1] += Q_biais*dt;

/////////////////Calcul du gain optimal/////////////////
//Calcul de l'innovation 
  y = angle_nouveau - Angle;
//Calcul du gain de filtrage Kf(k+1)
  k[0]=(P[0][0])/(P[0][0]+Vd);
  k[1]=(P[1][0])/(P[0][0]+Vd);

/////////////////Recalage/////////////////
//Calcul de x(k+1|k+1)
  Angle+=k[0]*y;
  biais+=k[1]*y;

//Calcul de P(k+1|k+1)
  P00_int = P[0][0];
  P01_int = P[0][1];

  P[0][0] -= k[0] * P00_int;
  P[0][1] -= k[0] * P01_int;
  P[1][0] -= k[1] * P00_int;
  P[1][1] -= k[1] * P01_int;

  return Angle;
}
