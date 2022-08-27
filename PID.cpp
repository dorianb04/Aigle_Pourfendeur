
#define E_precedente 0
#define E_actuelle 1
#define sum_E 2
#define delta_E 3

/*
La fonction est concue pour fcontionner avec les tableaux et constantes faites de la manière suivante : 
float K[3]={4, 0.02, 0};// {Kp, Ki, Kd} les coefficient du correcteur
float Consigne = 0;
float erreur[4] = {0, 0, 0, 0};//{erreur_précédente, erreur actuelle, sum_erreurs, delta_erreurs}
*/

float PID(float[], float, float, float[]);

float PID(float K[3], float Mesure, float Consigne, float erreur[4]){

  float PID_val=0;

//--------- Calcul des constantes ---------

//Calcul de l'erreur actuelle 
  erreur[E_actuelle] = Mesure - Consigne;

//Calcul de la somme des erreurs 
  erreur[sum_E]+=erreur[E_actuelle];
//erreur[sum_E]   = minMax(erreur[sum_E],   -400/K[1],   400/K[1]); //Au cas où il faut borner le problème. 

//Calcul du delta des erreurs
  erreur[delta_E]=erreur[E_actuelle]-erreur[E_precedente];

//Passage à t+1
  erreur[E_precedente] = erreur[E_actuelle];

//--------- Correcteur PID ---------  

  return PID_val = (erreur[E_actuelle]   * K[0])   + (erreur[sum_E]   * K[1])   + (erreur[delta_E]   * K[2]);
}
