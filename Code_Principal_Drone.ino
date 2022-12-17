/*@autor : Stanislas BRUSSELLE
Branchements : 

ESC_M1 = 4
ESC_M2 = 5
ESC_M3 = 6
ESC_M4 = 7

LED = 12

*/
//////////////////////////////////
////// Librairies à inclure //////
//////////////////////////////////
#include <Kalman.h>
#include <Wire.h>

//////////////////////////////////////
////// Déclaration des constantes //////
//////////////////////////////////////

#define ADRESSE_MPU 0x68  // adresse du MPU-6050 = 0x68

#define ROLL  0     // indice 0 : axe roll
#define PITCH 1     // indice 1 : axe pitch
#define YAW   2     // indice 2 : axe yaw
#define GAZ   3     // indice 3 : gaz

#define X      0     // indice 0 : axe X
#define Y      1     // indice 1 : axe Y
#define Z      2     // indice 2 : axe Z

#define ARRET   0   // ARRET équivaut à 0
#define ARME    1   // ARME équivaut à 1
#define MARCHE  2   // MARCHE équivaut à 2

#define CANAL1 0    // CANAL1 équivaut à 0 (ordre ROLL)
#define CANAL2 1    // CANAL2 équivaut à 1 (ordre PITCH)
#define CANAL3 2    // CANAL3 équivaut à 2 (ordre GAZ)
#define CANAL4 3    // CANAL4 équivaut à 3 (ordre YAW)
#define CANAL5 4

#define ESC1 0    // ESC1 équivaut à 0
#define ESC2 1    // ESC2 équivaut à 1
#define ESC3 2    // ESC3 équivaut à 2
#define ESC4 3    // ESC4 équivaut à 3

#define LED 12    // LED équivaut à 12

#define pi 3.14159265359 // constante Pi

/////////////////////////////////////////////////////////
////// Déclaration des Constantes et des Variables //////
/////////////////////////////////////////////////////////

//----------------------------
// définition des constantes
//----------------------------
const int chA = 11;  
const int chB = 12;
const int chC = 9;
const int chD = 8;
const int chE = 13;

int timeOut = 20000;

int MaxEscSpeed = 180;

const float FE_Gyro = 65.5;  // facteur d'échelle du gyro
const float FE_Accel = 4096; // facteur d'échelle de l'accéléro

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//\\\\                  PARAMETRES DE REGLAGE PID                     \\\\
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


const float KpRoll = 1.80;   // valeur du gain proportionnel en roll
const float KiRoll = 0.03;   // valeur du gain intégral en roll
const float KdRoll = 12.6;   // valeur du gain dérivé en roll
const float limite_PID_Roll = 400; // limite en valeur absolue de la
                                   // correction PID en Roll

const float KpPitch = KpRoll;   // valeur du gain proportionnel en pitch
const float KiPitch = KiRoll;   // valeur du gain intégral en pitch
const float KdPitch = KdRoll;   // valeur du gain dérivé en pitch
const float limite_PID_Pitch = limite_PID_Roll; // limite en valeur absolue de la
                                                // correction PID en Pitch

const float KpYaw = 4.00;   // valeur du gain proportionnel en yaw
const float KiYaw = 0.02;   // valeur du gain intégral en yaw
const float KdYaw = 0.00;   // valeur du gain dérivé en yaw
const float limite_PID_Yaw = 400; // limite en valeur absolue
                                  // de la correction PID en Yaw


const int coeff_stabilisation = 3.0;  // coefficient de correction d'assiette, 3.00 est une bonne valeur
boolean mode_stabilise = true; //Mode stabilisé (true) ou Mode acrobatique (false)

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//------------------------------------
// déclaration des variables globales
//------------------------------------

//---------------------------------------------------------------------------------------
// variables servant à calculer la durée écoulée entre 2 lectures des mesures du MPU6050
//---------------------------------------------------------------------------------------

unsigned long dT; // variable de calcul de la durée écoulée entre 2 lectures des mesures du MPU6050
unsigned long oldTime;
unsigned long currentTime;

//--------------------------------------------------------------------------------------------
// variables servant à calculer la durée des impulsions délivrées sur les canaux du récepteur
//--------------------------------------------------------------------------------------------

volatile unsigned int duree_impulsion[5];

volatile unsigned int duree_impulsion_TMP[5];

volatile unsigned long instant_courant; // variable de mémorisation de l'instant courant utilisée dans l'ISR
                                        // pour calculer les durées des impulsions du récepteur

volatile unsigned long debut_impulsion[4]; // tableau des instants de début des impulsions des canaux du récepteur

volatile byte memo_etat_canal[4]; // mémorisation des états des canaux de réception


//--------------------------------------------------
// variables servant à générer les impulsions d'ESC
//--------------------------------------------------

unsigned long instant_actuel; // variable de mémorisation de l'instant
                              // courant pour savoir quand générer les fronts descendants
                              // des signaux de commande des ESCs

unsigned long debut_impulsions_ESC;   // instant de début de génération des impulsions d'ESC
unsigned int duree_impulsion_cde[4]= {1000,1000,1000,1000};  //  durée des impulsions de commande
                                                             // à envoyer aux ESC
unsigned long fin_impulsion_cde[4];   // instants des fins d'impulsions d'ESCs

//-----------------------------------------------
// variables de lecture des données du MPU6050
//-----------------------------------------------
int accel_raw[3] = {0,0,0};  // tableau des accélérations brutes

int gyro_raw[3] = {0,0,0};  // tableau des vitesses angulaires brutes

float gyro_filtre[3] = {0,0,0}; // tableau des vitesses angulaires filtrées

float accel_filtre[3] = {0,0,0};// tableau des accélérations filtrées

int temperature = 0;

float gyro[3] = {0,0,0}; // tableau des vitesses angulaires sans offsets, filtrées et
                         // à l'échelle (en [°/s])

float accel[3] = {0,0,0};// tableau des accélérations sans offsets, filtrées et
                         // à l'échelle (en multiple de "g")

float angle_gyro[3] = {0,0,0}; // tableau des angles calculés à partir des vitesses angulaires

float angle_accel[2] = {0,0};  // tableau des angles calculés à partir des accélérations

float angle[3] = {0,0,0}; // tableau des angles calculés par "sensor fusion"

float gyro_offset[3] = {0,0,0}; // tableau des offsets du gyroscope

float accel_offset[3] = {0,0,0}; // tableau des offsets de l'accéléromètre

boolean init_angles_gyro=false; // flag permettant de savoir si l'alignement des
                                // angles calculés à partir des valeurs du gyro
                                // sur les angles calculés à partir des valeurs
                                // de l'accéléromètre a été fait
float acc_total_vector = 0;
//-----------------------------------------
// variables de calcul des corrections PID
//-----------------------------------------

// variables contenant les consignes de vol
float consigne[4] = {0,0,0,0};

// valeurs des corrections calculées en P, I et D

float correct_P[3] = {0,0,0}; // tableau des corrections proportionnelles en roll, pitch et yaw
float correct_I[3] = {0,0,0}; // tableau des corrections intégrales en roll, pitch et yaw
float correct_D[3] = {0,0,0}; // tableau des corrections dérivées en roll, pitch et yaw
float correct_PID[3] = {0,0,0}; // tableau des corrections complètes PID en roll, pitch et yaw
float limite_PID[3] = {limite_PID_Roll, limite_PID_Pitch, limite_PID_Yaw}; // limites en valeurs absolues
                                                                           // des corrections PID en Roll, Pitch et Yaw

// variables nécessaires aux calculs PID
float erreur[3]= {0,0,0}; // tableau des erreurs en roll, pitch et yaw
float integrale_erreur[3] = {0,0,0}; // tableau des intégrales des erreurs (pour les calculs PID)
float derivee_erreur[3] = {0,0,0}; // tableau des dérivées des erreurs (pour les calculs PID)
float memo_erreur[3] = {0,0,0}; // tableau de mémorisation des erreurs en roll, pitch et yaw

// variables d'ajustement pour la correction des consignes de vitesses angulaires Pitch et Roll
float ajustement_pitch = 0;
float ajustement_roll = 0;

//--------------------
// variables diverses
//--------------------

byte etat; // variable contenant l'état de fonctionnement du drone

// variable permettant la mesure de la durée d'exécution de la boocle "loop()"
unsigned long debut_loop;

///////////////////////////////////////////////////////////////////
////// Déclaration des Fonctions utilisées dans le programme //////
///////////////////////////////////////////////////////////////////

void configurerSorties(); // fonction de configuration des pins utilisés comme des sorties

void initialiser_MPU6050(); // fonction d'initialisation du circuit MPU6050 par I2C

void calibrerMPU6050();     // fonction de calibration du MPU6050 : calcul des offsets du gyroscope et de l'accéléromètre

void attendreGazMini();     // attente de positionnement du joystick de gauche en bas et à gauche : gaz mini, yaw mini

void lireMPU6050();         // lecture des données du MPU6050

void calculerAnglesFusion(); // calcul des angles Roll et Pitch par "Sensor Fusion"

void calculerConsignes();        // calcul des valeurs des consignes à partir des durées d'impulsions PWM des canaux du récepteur

void calculerCommandesPID();     // calcul des corrections PID

void calculerImpulsionsESC();    // calcul des durées des impulsions PWM à envoyer aux ESC

void genererImpulsionsESC();     // génération des impulsions d'ESC et de lecture des mesures du MPU-6050

void razControleursPID();        // raz des variables de calcul des corrections PID

void stopperMoteurs();           // fixe les durées d'impulsions des ESCs à 1000 (valeur minimale)

void Calc_dT();

void pulse();

void mode();

float borner(float valeur, float valeur_min, float valeur_max); // fonction qui permet de borner une grandeur de type "float" entre 2 valeurs 

//#######################################################
//####                     SETUP()                   ####
//#######################################################

void setup()  
{
 
    Serial.begin(115200);

    duree_impulsion_cde[ESC1] = 1000;
    duree_impulsion_cde[ESC2] = 1000;
    duree_impulsion_cde[ESC3] = 1000;
    duree_impulsion_cde[ESC4] = 1000;
    
    pinMode(chA, INPUT);
    pinMode(chB,INPUT);
    pinMode(chC,INPUT);
    pinMode(chD,INPUT);
    pinMode(chE,INPUT);
    
    configurerSorties(); // configuration des pins utilisés en sortie
    
    Serial.print("initialiserMPU6050");
    initialiserMPU6050(); // fonction d'initialisation du MPU-6050
    
    Serial.print("calibrerMPU6050");
    calibrerMPU6050(); // fonction de calibration du MPU6050 : calcul des offsets du gyroscope et de l'accéléromètre
    
    Serial.print("attendreGazMini");
    
    // attente tant que le joystick des Gaz n'est pas en position de sécurité : au minimum et centré
    attendreGazMini();
    // après l'exécution de "attendreGazMini()", l'état du quadricoptère correspond à la position de sécurité du joystick, "etat" est alors mis à zéro (état "ARRET")
    etat = ARRET; // passage à l'état ARRET
    // lecture des vitesses angulaires et des accélérations
    lireMPU6050();

}
//##### Fin SETUP #####


//####################################################
//####                    LOOP                    ####
//####################################################

void loop()  // boucle du programme principal
{
  debut_loop = micros();

  Serial.print("Ch1:");  //Affiche les valeurs reçue par les canaux de communication
  Serial.print(duree_impulsion[CANAL1]);     
  
  Serial.print("|Ch2:");
  Serial.print(duree_impulsion[CANAL2]);

  Serial.print("|Ch3:");
  Serial.print(duree_impulsion[CANAL3]);

  Serial.print("|Ch4:");
  Serial.print(duree_impulsion[CANAL4]);

  Serial.print("|Ch5:");
  Serial.print(duree_impulsion[CANAL5]);

  pulse();
  mode();
  lireMPU6050();

  // traitement des mesures du gyroscope pour obtenir des valeurs de vitesses angulaires fiables et
  // calcul des angles par fusion des mesures du gyro et de l'accélérometre par filtrage de Kalman
  calculerAnglesFusion();

  //////////////////////////////////////////////////////////////////////////////////////
  //  Evolution des états du drone en fonction de la position du joystick de gauche.  //
  //                                                                                  //
  //  Rappel :  GAZ   --> Canal 3                                                     //
  //            YAW   --> Canal 4                                                     //
  //            PITCH --> Canal 2                                                     //
  //            ROLL  --> Canal 1                                                     //
  //                                                                                  //
  //////////////////////////////////////////////////////////////////////////////////////

  //---------------------------
  // ETAT ARRET VERS ETAT ARME
  //---------------------------
  //
  // après le "setup" le quadricoptère est dans l'état "ARRET"
  // si à partir de cette état on met le joystick gauche en bas à gauche alors le quadricoptère passe
  // à l'état "ARME" : les moteurs sont "armés" (prêts à tourner si on ramène le YAW en position médiane)
  //
  // duree_impulsion[CANAL3] --> GAZ
  // duree_impulsion[CANAL4] --> YAW
  //

  if (etat == ARRET && duree_impulsion[CANAL3] < 1050 && duree_impulsion[CANAL4] < 1050)
  {
    etat = ARME;  // passage à l'état "ARME"
    Serial.println("ARME");
  }

  // Rq : après le "setup()", l'état est forcément "ARRET" on pourrait donc être tenté de ne tester que la position du
  // joystick (ce que j'avais fait au début...). Mais si par la suite on est en "MARCHE" et qu'on met le joystick en bas
  // à gauche on peut aussi passer en état "ARME" ce qui coupe les moteurs : ce n'est pas le comportement attendu.
  // Normalement on ne peut arriver sur "ARME" que depuis l'état "ARRET". Il est donc indispensable de mettre la condition
  // "etat == ARRET" dans le "if".

  //----------------------------
  // ETAT ARME VERS ETAT MARCHE
  //----------------------------
  //
  // si l'état est l'état "ARME" et qu'on ramène le joystick gauche au centre (Yaw médian)
  // toujours avec les GAZ au minimum alors :   - "etat" est positionné à "MARCHE"
  //                                            - on remet à zéro les grandeurs de calcul des PID (=> razControleursPID();)
  // dans l'état "MARCHE", le démarrage des moteurs est possible dès que les gaz augmentent

  if (etat == ARME && duree_impulsion[CANAL3] < 1050 && duree_impulsion[CANAL4] > 1450)
  {
    etat = MARCHE; // passage à l'état "MARCHE"
    Serial.println("MARCHE");

    razControleursPID();  // reset des variables de calcul PID pour pas que les moteurs s'emballent au démarrage
                          // au premier démarrage il n'y a pas de problème, mais par exemple après un premier
                          // arrêt ces variables ne sont pas nulles et au démarrage suivant le programme
                          // va les prendre en compte et donner des ordres erronnés aux moteurs
  }

  //-----------------------------
  // ETAT MARCHE VERS ETAT ARRET
  //-----------------------------
  // si on est dans l'état "MARCHE" (i.e. moteurs qui tournent) et qu'on met la manette
  // de gauche en bas à droite :
  //     - GAZ au mini
  //     - YAW au maxi (droite)
  // l'état repasse alors en état "ARRET"

  if (etat == MARCHE && duree_impulsion[CANAL3] < 1050 && duree_impulsion[CANAL4] > 1700)
  {
    etat = ARRET;
    Serial.println("ARRET");
  }

  // conversion des ordres de commande reçus en données exploitables par les PIDs
  calculerConsignes();

  // les consignes et les mesures sont connues, on peut calculer les corrections PID
  calculerCommandesPID();

  ////////////////////////////////////////////////////
  //  Actions à effectuer selon les états du drone  //
  ////////////////////////////////////////////////////

  //--------------
  // ETAT MARCHE
  //--------------

  if (etat == MARCHE) 
  {
    calculerImpulsionsESC();  // calcul des durées d'impulsions des ESCs
  }
  else
  {
    //--------------------
    // ETAT ARME ou ARRET
    //--------------------
    
    //  dans tous les autres états que MARCHE, les moteurs sont mis à l'arrêt
    stopperMoteurs(); // on fixe une impulsion de 1000us sur les 4 ESC
    
    init_angles_gyro = false; // on autorise l'affectation des angles issus de l'accéléromètre dans les
                              // variables d'angles issues du gyroscope afin que le drone ne redécolle
                              // pas avec des valeurs d'angles issues du gyroscope erronées (la surface
                              // où l'arrêt a eu lieu n'est pas horizontale).
  }

  // génération des impulsions d'ESCs
  genererImpulsionsESC(); // IMPORTANT : la fonction genererImpulsionsESC() contient la fonction lireMPU6050()
                          // afin de mettre à jour les mesures

  dT = micros() - debut_loop;
  // on attend que la durée d'exécution de la "loop()" atteigne 4000us
  while(micros()-debut_loop < 4000);
}
//##### Fin "LOOP()" #####

///////////////////////////////////////////////
//// FONCTIONS UTILISEES DANS LE PROGRAMME ////
///////////////////////////////////////////////

//#####################################################
//  Fonction d'initialisation des sorties de l'Arduino
//
//  "configurerSorties()"
//  - passe en sortie (OUTPUT) toutes les sorties nécessaires 
//  - pas de paramètre d'entrée
//  - aucune valeur retournée
//
//#####################################################
void configurerSorties()
{
    // déclaration des broches utilisées comme sorties par programmation des ports
    DDRD |= B11110000;  // configuration des pins 4, 5, 6 et 7 (les ESC) du port D comme sorties
    DDRB |= B00010000;  // configuration du pin 12 (la LED indicatrice) du port B comme sortie
}


//#########################################
//  Fonction d'initialisation du MPU6050
//
//  "initialiserMPU6050()"
//  - permet la configuration de l'IMU 
//  - pas de paramètre d'entrée
//  - aucune valeur retournée
//
//#########################################

void initialiserMPU6050()
{

    // Ouverture de la ligne I2C en maître
    Wire.begin();
    // Configuration de la clock à 400kHz au lieu des 100kHz par défaut
    TWBR = 12;

    //On remercie bien Lobodol est ca série de tutoriels sur comment contruire un drone. 
    //La configuration et l'utilisation de l'IMU est inspirée (pour ne pas dire reprise) de son travail. 
    
    // Configuration horloge interne
    Wire.beginTransmission(ADRESSE_MPU); // début communication
    Wire.write(0x6B);                    // registre PWR_MGMT_1
    Wire.write(0x00);                    // horloge interne 8MHz
    Wire.endTransmission();              // fin de transmission
    
    // Configuration échelle gyroscope
    Wire.beginTransmission(ADRESSE_MPU);
    Wire.write(0x1B);                    // registre GYRO_CONFIG
    Wire.write(0x08);                    // plage ±500°/s
    Wire.endTransmission();
    
    // Configuration échelle accéléromètre
    Wire.beginTransmission(ADRESSE_MPU);
    Wire.write(0x1C);                    // registre ACCEL_CONFIG
    Wire.write(0x10);                    // plage ±8g
    Wire.endTransmission();
    
    // Configuration filtre passe-bas
    Wire.beginTransmission(ADRESSE_MPU);
    Wire.write(0x1A);                    // registre CONFIG
    Wire.write(0x03);                    // coupure à ~43Hz
    Wire.endTransmission();

    delay(250); // on laisse le temps au MPU-6050 de démarrer
}


//#####################################################
//  Fonction de lecture des mesures brutes du MPU6050
//
//  "lireMPU6050()"
//  - permet un lecture dees données brutes retournées par l'IMU (en deg/s et deg/s^(-2))
//  - pas de paramètre d'entrée
//  - aucune valeur retournée
//
//#####################################################
void lireMPU6050()
{

    Wire.beginTransmission(ADRESSE_MPU);
    Wire.write(0x3B); // adresse de début
    Wire.endTransmission();
    Wire.requestFrom(ADRESSE_MPU,14);// requête lecture 14 octets

    // Attente jusqu'à ce que les 14 octets soient reçus
    while(Wire.available() < 14);

    accel_raw[ROLL]  = Wire.read() << 8 | Wire.read();   // accél. X
    accel_raw[PITCH]  = Wire.read() << 8 | Wire.read();   // accél. Y
    accel_raw[YAW]  = Wire.read() << 8 | Wire.read();   // accél. Z
    temperature = Wire.read() << 8 | Wire.read();     // température
    gyro_raw[ROLL] = Wire.read() << 8 | Wire.read();  // gyro roll
    gyro_raw[PITCH] = Wire.read() << 8 | Wire.read(); // gyro pitch
    gyro_raw[YAW] = Wire.read() << 8 | Wire.read();   // gyro yaw

    // changement de signes pour la convention d'orientationssur les vitesses angulaires 
    gyro_raw[PITCH]=-gyro_raw[PITCH];
    gyro_raw[YAW]=-gyro_raw[YAW];
    
    accel_raw[X]=-accel_raw[X];
}

//##########################################################
//  Fonction de calcul des offsets moyens du gyroscope
//  et de l'accéléromètre
//
//  "calibrerMPU6050()"
//
//  - Permet de calculer les offset. Pour cela, on fait la moyenne de 2000 échantillons, pendant ces mesures 
//le MPU-6050 doit rester horizontal et immobile.
//  - pas de paramètre d'entrée
//  - pas de valeur retournée
//
//##########################################################

void calibrerMPU6050()
{
    int max_samples = 2000;

    for (int i = 0; i < max_samples; i++)
    {
        // on fait clignoter la LED rapidement pendant la calibration
        // pour cela on la fait changer d'état toutes les 50 itérations de la boucle "for"
        //Ici il n'y a pas besoin d'utiliser la programmation des ports car il n'y a pas d'objectif d'optimisation
        //de la vitesse d'execution. Ceci explique le "digitalWrite". 
        if(i % 20 == 0) digitalWrite(LED, !digitalRead(LED));
        
        lireMPU6050(); // fonction de lecture du MPU6050

        // somme des échantillons de mesures

        gyro_offset[ROLL] += gyro_raw[ROLL];
        gyro_offset[PITCH] += gyro_raw[PITCH];
        gyro_offset[YAW] += gyro_raw[YAW];

        accel_offset[X] += accel_raw[X];
        accel_offset[Y] += accel_raw[Y];
        accel_offset[Z] += accel_raw[Z];

        // pour éviter que les ESC n'émettent des beeps pendant la calibration
        // il faut qu'ils reçoivent un signal PWM, on leur envoie donc la valeur
        // minimale : impulsions de 1000us

        PORTD |= B11110000;     // mise à "1" des sorties reliées aux ESC
        delayMicroseconds(1000);// attente de 1000 microsecondes (1ms)
        PORTD &= B00001111;     // mise à "0" des sorties reliées aux ESC
        
        delay(3);     // attente de 3ms pour compléter la période du signal PWM à environ 4ms
                      // (environ car il y a d'autres instructions dans la boucle qui prennent
                      // du temps d'exécution) soit une fréquence proche de 250 Hz, ce qui est
                      // la limite haute acceptable pour un ESC standard

    }

    // calcul de la moyenne des offsets pour les vitesses angulaires
    gyro_offset[ROLL] /= max_samples;
    gyro_offset[PITCH] /= max_samples;
    gyro_offset[YAW] /= max_samples;

    // calcul de la moyenne des offsets pour les accélérations
    accel_offset[X] /= max_samples;
    accel_offset[Y] /= max_samples;

    // ATTENTION :
    //
    // pour les axes X et Y, on retranche la valeur de l'offset
    // à la valeur brute pour la recentrer sur zéro car au
    // repos et horizontal le MPU a des accélérations nulles
    // en X et Y
    //
    // pour l'axe Z, la rectification des mesures brutes est
    // différente de celle des axes X et Y
    // au repos et à l'horizontale le MPU6050 subit une
    // accélération de 1g
    // pour une accélération de 1g, la valeur renvoyée doit
    // être de 4096 donc l'offset à retrancher à la valeur
    // brute pour la recentrer sur 4096 est : moyenne - 4096

    // calcul de l'offset moyen
    accel_offset[Z] /= max_samples;
    // recentrage sur 4096
    accel_offset[Z] -= 4096;
}

//##########################################################
//  Fonction de remise à zéro des variables servant à
//  calculer les corrections PID.
//
//  "razControleursPID()"
//  - permet la remise à zero des valeurs calculées pour le correcteur PID
//  - pas de paramètre d'entrée
//  - pas de valeur retournée
//
//##########################################################
void razControleursPID()
{
    erreur[ROLL] = 0;
    erreur[PITCH] = 0;
    erreur[YAW] = 0;

    integrale_erreur[ROLL] = 0;
    integrale_erreur[PITCH] = 0;
    integrale_erreur[YAW] = 0;

    derivee_erreur[ROLL] = 0;
    derivee_erreur[PITCH] = 0;
    derivee_erreur[YAW] = 0;

    memo_erreur[ROLL] = 0;
    memo_erreur[PITCH] = 0;
    memo_erreur[YAW] = 0;

     correct_P[ROLL] = 0;
    correct_I[ROLL] = 0;
    correct_D[ROLL] = 0;

    correct_P[PITCH] = 0;
    correct_I[PITCH] = 0;
    correct_D[PITCH] = 0;

    correct_P[YAW] = 0;
    correct_I[YAW] = 0;
    correct_D[YAW] = 0;

    correct_PID[ROLL] = 0;
    correct_PID[PITCH] = 0;
    correct_PID[YAW] = 0;

}

//###################################################################
//  Fonction qui fixe les durées des impulsions de commande des ESC
//  à la valeur minimale de 1000.
//
//  "stopperMoteurs()"
//  - permet l'arret complet des moteurs
//  - pas de paramètre d'entrée
//  - pas de valeur retournée
//
//###################################################################
void stopperMoteurs()
{
    duree_impulsion_cde[ESC1] = 1000;
    duree_impulsion_cde[ESC2] = 1000;
    duree_impulsion_cde[ESC3] = 1000;
    duree_impulsion_cde[ESC4] = 1000;
}

//################################################################
//  Fonction de calcul des angles Roll et Pitch
//
//  "calculerAnglesFusion()"
//  - permet la fusion des angles données par le gyroscope et l'accéléromètre
//  - pas de paramètre d'entrée
//  - pas de valeur retournée
//
//################################################################
void calculerAnglesFusion()
{

    // calcul des vitesses angulaires brutes sans offsets
    gyro_raw[ROLL] = gyro_raw[ROLL] - gyro_offset[ROLL];
    gyro_raw[PITCH] = gyro_raw[PITCH] - gyro_offset[PITCH];
    gyro_raw[YAW] = gyro_raw[YAW] - gyro_offset[YAW];

    // calcul des accélérations brutes sans offsets
    accel_raw[X] = accel_raw[X] - accel_offset[X];
    accel_raw[Y] = accel_raw[Y] - accel_offset[Y];
    accel_raw[Z] = accel_raw[Z] - accel_offset[Z];

    // filtrage des vitesses angulaires brutes sans offsets
    gyro_filtre[ROLL] = 0.8*gyro_filtre[ROLL] + 0.2*gyro_raw[ROLL];
    gyro_filtre[PITCH] = 0.8*gyro_filtre[PITCH] + 0.2*gyro_raw[PITCH];
    gyro_filtre[YAW] = 0.8*gyro_filtre[YAW] + 0.2*gyro_raw[YAW];

    // filtrage des accélérations brutes sans offsets
    accel_filtre[X] = 0.8*accel_filtre[X] + 0.2*accel_raw[X];
    accel_filtre[Y] = 0.8*accel_filtre[Y] + 0.2*accel_raw[Y];
    accel_filtre[Z] = 0.8*accel_filtre[Z] + 0.2*accel_raw[Z];

    // mise à l'échelle des vitesses angulaires brutes
    // sans offsets et filtrées
    gyro[ROLL] = gyro_filtre[ROLL] / FE_Gyro;
    gyro[PITCH] = gyro_filtre[PITCH] / FE_Gyro;
    gyro[YAW] = gyro_filtre[YAW] / FE_Gyro;

    // mise à l'échelle des accélérations brutes sans offsets filtrées
    accel[X] = accel_filtre[X] / FE_Accel;
    accel[Y] = accel_filtre[Y] / FE_Accel;
    accel[Z] = accel_filtre[Z] / FE_Accel;

    //--------------------------------------------------
    // calcul des angles à partir des données des gyros
    //--------------------------------------------------
    angle_gyro[ROLL] += gyro[ROLL]*(dT/(float)1000000); // "cast" obligatoire sinon les résultats sont constants

    angle_gyro[PITCH] += gyro[PITCH]*(dT/(float)1000000);

    angle_gyro[YAW] += gyro[YAW]*(dT/(float)1000000);


    // transfert d'angle ROLL <--> PITCH dans le cas où il y a une rotation YAW

    angle_gyro[ROLL] += angle_gyro[PITCH] * sin(gyro[YAW]*(dT/(float)1000000)*0.0174533); // (pi/180=0,0174533)

    angle_gyro[PITCH] -= angle_gyro[ROLL] * sin(gyro[YAW]*(dT/(float)1000000)*0.0174533);

    //------------------------------------------------------
    // calcul des angles à partir des données des accéléros
    //------------------------------------------------------

    //Les formules suivantes sont à vérifier 
    //angle_accel[ROLL] = atan(accel[Y]/(sqrt(accel[X]*accel[X]+accel[Z]*accel[Z])))*(float)(180/pi);
    //angle_accel[PITCH] = -atan(accel[X]/(sqrt(accel[Y]*accel[Y]+accel[Z]*accel[Z])))*(float)(180/pi);

    //Calcul du vecteur accélération totale
    acc_total_vector = sqrt(pow(accel[X], 2) + pow(accel[Y], 2) + pow(accel[Z], 2));
    
  // deux conditions if pour garantir que l'arcsin reste dans l'intervalle [-1; 1]
    if (abs(accel[X]) < acc_total_vector) {
      angle_accel[X] = asin((float)accel[Y] / acc_total_vector) * (180 / pi); // asin donne des angles en radian, il faut multiplier par (180 / pi) pour obtenir des degrés.
    }
    if (abs(accel[Y]) < acc_total_vector) {
      angle_accel[Y] = asin((float)accel[X] / acc_total_vector) * (180 / pi);
    }

    //--------------------------------------------
    // calcul des angles par filtre complémentaire
    //--------------------------------------------

    if(init_angles_gyro)
    {
        angle[ROLL] = 0.9996*angle_gyro[ROLL]+0.0004*angle_accel[ROLL];
        angle[PITCH] = 0.9996*angle_gyro[PITCH]+0.0004*angle_accel[PITCH];
    }
    else
    {
      // alignement des angles du gyro sur ceux de l'accéléro
      // une seule fois au démarrage
      angle_gyro[ROLL]=angle_accel[ROLL];
      angle_gyro[PITCH]=angle_accel[PITCH];
      init_angles_gyro = true;
    }

      Serial.print ("|Roll:");  //Display text string on Serial Monitor to distinguish variables
      Serial.print (angle[ROLL]);     //Print in the value of channel 1

      Serial.print ("|Pitch:");
      Serial.print (angle[PITCH]);

      Serial.print ("|Yaw:");
      Serial.print (angle_gyro[YAW]);

    // calcul de la correction pour la stabilisation horizontale
    ajustement_roll = angle[ROLL] * coeff_stabilisation;
    ajustement_pitch = angle[PITCH] * coeff_stabilisation;

    if(!mode_stabilise)
    {   // si le quadricoptère n'est pas en mode stabilisé
        ajustement_roll = 0;   // fixe la correction de consigne roll à zéro
        ajustement_pitch = 0;  // fixe la correction de consigne pitch à zéro
    }

}


//##########################################################################
//  Fonction de conversion des signaux du récepteur en valeurs de consignes
//
//  "calculerConsignes()"
//
//  - pas de paramètre d'entrée
//
//  - pas de valeur retournée
//
//##########################################################################
void calculerConsignes()
{
    // calcul des consignes des PIDs

    // Remarque : il faut une petite bande morte d'environ 16us autour des points médians des joysticks de commande sinon la régulation est instable
    //            lorsque les joysticks sont en position médiane
    //
    // Rappel des canaux :
    //
    //      ROLL   --> Channel 1
    //      PITCH  --> Channel 2
    //      GAZ    --> Channel 3
    //      YAW    --> Channel 4
    //


    //#######################
    //#### Consigne ROLL ####
    //#######################

    // la consigne Roll en degrés/sec est déterminée à partir des durées d'impulsions du canal Roll du récepteur
    // ces durées d'impulsions doivent être converties en deg/sec pour attaquer l'entrée du PID
    //
    // une vitesse angulaire de 0,33tr/sec maxi semble raisonnable, ce qui correspond à 120deg/s
    // les équations correspondantes sont :
    //
    //       pour l'intervalle [1000us - 1492us] : consigne_vitesse_angulaire = 0.244 x durée_impulsions_consigne - 363,90
    //
    //       pour l'intervalle [1508us - 2000us] : consigne_vitesse_angulaire = 0.244 x durée_impulsions_consigne - 367,80
    //

    consigne[ROLL] = 0;

    if(duree_impulsion[CANAL1] > 1508)
    {
          consigne[ROLL] = 0.244*duree_impulsion[CANAL1] - 367.80;
    }
    else if(duree_impulsion[CANAL1] < 1492)
    {
          consigne[ROLL] = 0.244*duree_impulsion[CANAL1] - 363,90;
    }

    consigne[PITCH] = 0;

    // la consigne PITCH a une pente négative car une consigne PITCH inférieure à 1492us doit faire relever le nez du drone
    if(duree_impulsion[CANAL2] > 1508)
    {
          consigne[PITCH] = -0.244*duree_impulsion[CANAL2] + 367.80;
    }
    else if(duree_impulsion[CANAL2] < 1492)
    {
      consigne[PITCH] = -0.244*duree_impulsion[CANAL2] + 363.90;
    }

    consigne[YAW] = 0;

    if(duree_impulsion[CANAL3] > 1050)    // on ne calcule la consigne YAW que si la commande des GAZ est non nulle
    {                                     // sinon les moteurs accélèrent lorsque qu'on passe en état "ARRET"


        if(duree_impulsion[CANAL4] > 1508)
        {
              consigne[YAW] = 0.244*duree_impulsion[CANAL4] - 367.80;
        }
        else if(duree_impulsion[CANAL4] < 1492)
        {
              consigne[YAW] = 0.244*duree_impulsion[CANAL4] - 363,90;
        }

    }

    //#######################
    //#### Stabilisation ####
    //#######################

    // correction des consignes de vitesses angulaires
    // en Roll et Pitch pour assurer l'Auto-Level

    consigne[ROLL] -= ajustement_roll;
    consigne[PITCH] -= ajustement_pitch;


    //######################
    //#### Consigne GAZ ####
    //######################

    consigne[GAZ] = duree_impulsion[CANAL3];

    if (consigne[GAZ] > 1700) consigne[GAZ] = 1700; // on limite la valeur des GAZ, car il va s'y ajouter les corrections PID pour donner les durées
                                                    // d'impulsions des ESC, sans cette limite la correction PID n'est pas prise en compte pour les valeurs
                                                    // de GAZ supérieures à 1600us (PID bornés à 400 et longueur maxi impulsions = 2000)
}

//#########################################
//  Fonction de calcul des commandes PID
//
//  "calculerCommandesPID()"
//
//  - pas de paramètre d'entrée
//
//  - pas de valeur retournée
//
//#########################################
void calculerCommandesPID()
{

    // calcul des corrections PID

    //###############
    //   Axe ROLL
    //###############

    //----------------------------
    // calcul de l'erreur en ROLL
    //----------------------------
    erreur[ROLL] = consigne[ROLL] - gyro[ROLL] ;

    //--------------------------------------------
    // calcul de l'action proportionnelle en ROLL
    //--------------------------------------------
    correct_P[ROLL] = KpRoll * erreur[ROLL];

    //--------------------------------------
    // calcul de l'action intégrale en ROLL
    //--------------------------------------
    integrale_erreur[ROLL] += erreur[ROLL];    // formule condensée

    correct_I[ROLL] = KiRoll * integrale_erreur[ROLL];

    // bornage de la valeur de correction I en Roll
    correct_I[ROLL]=borner(correct_I[ROLL], -limite_PID[ROLL], limite_PID[ROLL]);

    //------------------------------------
    // calcul de l'action dérivée en ROLL
    //------------------------------------

    // la variable "memo_erreur[ROLL]" sert à mémoriser la précédente valeur de "erreur[ROLL]"
    derivee_erreur[ROLL] = erreur[ROLL] - memo_erreur[ROLL];
    correct_D[ROLL] = KdRoll * derivee_erreur[ROLL];

    // mise à jour de la mémorisation de l'erreur pour le calcul de la prochaine itération
    memo_erreur[ROLL] = erreur[ROLL];

    //-----------------------------------------
    // calcul de l'action complète PID en ROLL
    //-----------------------------------------

    // correction PID complète en roll
    correct_PID[ROLL] =  correct_P[ROLL] + correct_I[ROLL] + correct_D[ROLL];

    // bornage de la valeur de correction PID
    correct_PID[ROLL]=borner(correct_PID[ROLL], -limite_PID[ROLL], limite_PID[ROLL]);

    //###############
    //   Axe PITCH
    //###############

    //-----------------------------
    // calcul de l'erreur en PITCH
    //-----------------------------
    erreur[PITCH] = consigne[PITCH] - gyro[PITCH] ;

    //---------------------------------------------
    // calcul de l'action proportionnelle en PITCH
    //---------------------------------------------
    correct_P[PITCH] = KpPitch * erreur[PITCH];

    //---------------------------------------
    // calcul de l'action intégrale en PITCH
    //---------------------------------------
    integrale_erreur[PITCH] += erreur[PITCH];    // formule condensée

    correct_I[PITCH] = KiPitch * integrale_erreur[PITCH];

    // bornage de la valeur de correction I en Pitch
    correct_I[PITCH]=borner(correct_I[PITCH], -limite_PID[PITCH], limite_PID[PITCH]);

    //-------------------------------------
    // calcul de l'action dérivée en PITCH
    //-------------------------------------

    // la variable "memo_erreur[PITCH]" sert à mémoriser la précédente valeur de "erreur[PITCH]"
    derivee_erreur[PITCH] = erreur[PITCH] - memo_erreur[PITCH];
    correct_D[PITCH] = KdPitch * derivee_erreur[PITCH];

    // mise à jour de la mémorisation de l'erreur pour le calcul de la prochaine itération
    memo_erreur[PITCH] = erreur[PITCH];

    //------------------------------------------
    // calcul de l'action complète PID en PITCH
    //------------------------------------------

    // correction PID complète en PITCH
    correct_PID[PITCH] =  correct_P[PITCH] + correct_I[PITCH] + correct_D[PITCH];

    // bornage de la valeur de correction PID
    correct_PID[PITCH]=borner(correct_PID[PITCH], -limite_PID[PITCH], limite_PID[PITCH]);


    //###############
    //   Axe YAW
    //###############

    //---------------------------
    // calcul de l'erreur en YAW
    //---------------------------
    erreur[YAW] = consigne[YAW] - gyro[YAW];

    //-------------------------------------------
    // calcul de l'action proportionnelle en YAW
    //-------------------------------------------
    correct_P[YAW] = KpYaw * erreur[YAW];

    //-------------------------------------
    // calcul de l'action intégrale en YAW
    //-------------------------------------
    integrale_erreur[YAW] += erreur[YAW];    // formule condensée

    correct_I[YAW] = KiYaw * integrale_erreur[YAW];

    // bornage de la valeur de correction I en Yaw
    correct_I[YAW]=borner(correct_I[YAW], -limite_PID[YAW], limite_PID[YAW]);

    //-----------------------------------
    // calcul de l'action dérivée en YAW
    //-----------------------------------

    // la variable "memo_erreur[YAW]" sert à mémoriser la précédente valeur de "erreur[YAW]"
    derivee_erreur[YAW] = erreur[YAW] - memo_erreur[YAW];
    correct_D[YAW] = KdYaw * derivee_erreur[YAW];

    // mise à jour de la mémorisation de l'erreur pour le calcul de la prochaine itération
    memo_erreur[YAW] = erreur[YAW];

    //----------------------------------------
    // calcul de l'action complète PID en YAW
    //----------------------------------------

    // correction PID complète en YAW
    correct_PID[YAW] =  correct_P[YAW] + correct_I[YAW] + correct_D[YAW];

    // bornage de la valeur de correction PID
    correct_PID[YAW]=borner(correct_PID[YAW], -limite_PID[YAW], limite_PID[YAW]);
}

//#############################################################
// Fonction de bornage de valeurs
//
//  "float borner(float valeur, float valeur_min, float valeur_max)"
//
// paramètres d'entrée :
//
// float valeur : valeur réelle à borner
// float valeur_min : borne minimale
// float valeur_max : borne maximale
//
// valeur renvoyée :
//
// valeur bornée en type float
//
//#############################################################
float borner(float valeur, float valeur_min, float valeur_max)
{
    if (valeur > valeur_max)
    {
        valeur = valeur_max;
    }
    else if (valeur < valeur_min)
    {
        valeur = valeur_min;
    }

    return valeur;
}

//######################################################
//  Fonction de calcul des durées d'impulsions des ESCs
//
//  "calculerImpulsionsESC()"
//
//  - pas de paramètre d'entrée
//
//  - pas de valeur retournée
//
//######################################################
void calculerImpulsionsESC()
{
    //---------------------------
    // mixage des commandes PID
    //---------------------------

    duree_impulsion_cde[ESC1] = consigne[GAZ] + correct_PID[PITCH] - correct_PID[ROLL] + correct_PID[YAW];

    duree_impulsion_cde[ESC2] = consigne[GAZ] - correct_PID[PITCH] - correct_PID[ROLL] - correct_PID[YAW];

    duree_impulsion_cde[ESC3] = consigne[GAZ] - correct_PID[PITCH] + correct_PID[ROLL] + correct_PID[YAW];

    duree_impulsion_cde[ESC4] = consigne[GAZ] + correct_PID[PITCH] + correct_PID[ROLL] - correct_PID[YAW];

    // bornage des durées d'impulsions calculées entre 1100 et 2000
    duree_impulsion_cde[ESC1] = borner(duree_impulsion_cde[ESC1], 1100, 2000);
    duree_impulsion_cde[ESC2] = borner(duree_impulsion_cde[ESC2], 1100, 2000);
    duree_impulsion_cde[ESC3] = borner(duree_impulsion_cde[ESC3], 1100, 2000);
    duree_impulsion_cde[ESC4] = borner(duree_impulsion_cde[ESC4], 1100, 2000);
}

//##################################################################
//  Fonction de génération des impulsions d'ESCs et de lecture des
//  mesures du MPU-6050.
//
//  Juste après la génération des fronts montants des impulsions
//  d'ESCs il reste 1000us de libre dont on profite pour lire les
//  mesures du MPU-6050 et calculer l'intervalle de temps "dT"
//  écoulé depuis la dernière prise de mesures.
//
//  "genererImpulsionsESC()"
//
//  - pas de paramètre d'entrée
//
//  - pas de valeur retournée
//
//##################################################################
void genererImpulsionsESC(){

  float now =0;
  float difference=0;
  
  PORTD |= B11110000;
      
  while (PORTD >= 16) {
      now = micros();
      difference = now - debut_loop;

      if (difference >= duree_impulsion_cde[ESC1]) PORTD &= B11101111; // Passe la broche #4 à LOW
      if (difference >= duree_impulsion_cde[ESC2]) PORTD &= B11011111; // Passe la broche #5 à LOW
      if (difference >= duree_impulsion_cde[ESC3]) PORTD &= B10111111; // Passe la broche #6 à LOW
      if (difference >= duree_impulsion_cde[ESC4]) PORTD &= B01111111; // Passe la broche #7 à LOW
  }   
}

//##############################################################################
//  Fonction d'attente du positionnement "sécurité" des joysticks de commandes,
//  c'est à dire le joystick de gauche en bas à gauche.
//
//  "attendreGazMini()"
//
//  - pas de paramètre d'entrée
//
//  - pas de valeur retournée
//
//##############################################################################
void attendreGazMini()
{
        // fonction contenant une boucle "while" de sécurité pour que les moteurs ne se mettent pas à tourner dès la mise
        // sous tension du quadricoptère
        //
        // on attend une position de sécurité des joysticks de la radiocommande avant d'accepter des ordres de sa part
        // la position fixée est celle du joystick de gauche (en mode radiocommande 2) en bas et centré
        // ce qui signifie : commande de GAZ au minimum et commande YAW médiane
        //
        // on reste dans la boucle "while" de sécurité tant que :
        //
        //      - la commande des GAZ n'est pas en position basse ("duree_impulsion[CANAL3]" entre 990 et 1020)
        //
        //      - avec la commande YAW médiane ("duree_impulsion[CANAL4]" entre 1450 et 1550)
        //
        // Rappel des canaux :
        //
        //      ROLL   --> CANAL 1
        //      PITCH  --> CANAL 2
        //      GAZ    --> CANAL 3
        //      YAW    --> CANAL 4
        //
        //
        // Rappel des plages de valeurs reçues :
        //
        //     pour les joysticks des 3 axes et de la commande des gaz :
        //         - une position centrale renvoie une valeur de 1500
        //         - une position basse (ou gauche) renvoie une valeur inférieure à 1500 (1000 minimum)
        //         - une position haute (ou droite) renvoie une valeur supérieure à 1500 (2000 maximum)
        //

        int cpt=0;

        while(duree_impulsion[CANAL3] <990 || duree_impulsion[CANAL3] > 1020 || duree_impulsion[CANAL4] < 1450 || duree_impulsion[CANAL4] > 1550 )   // boucle tant que la commande des Gaz est supérieure au minimum
        {
              cpt ++;          // la variable "cpt" est incrémentée à chaque passage dans la boucle

              // on ne désire pas que les ESC beepent continuellement en attente du bon positionnement du joystick, alors on leur
              // envoie des impulsions brèves en attendant les valeurs adéquates provenant du récepteur RF

              PORTD |= B11110000;       // fixe les PINS 4, 5, 6 et 7 du port D à "high" (ESC 1 à 4)
              pulse();
              delayMicroseconds(1000);  // attente de 1000us

              PORTD &= B00001111;     // place les PINS 4, 5, 6 et 7 du port D à "low"

              delay(3);               // attente de 3 millisecondes avant la prochaine boucle

              // clignotement lent de la LED pendant l'attente du bon positionnement du joystick gauche
              if(cpt == 125)  // toutes les 125 boucles (#500ms)
              {
                    digitalWrite(LED, !digitalRead(LED));  // inversion de l'état de la LED par rapport à son état courant lu sur le pin 12
                    cpt = 0;                               // remet la variable "cpt" à 0
              }
        }

        digitalWrite(LED,LOW);  // on éteint la LED
}

void pulse()
{
  duree_impulsion_TMP[CANAL1] = pulseIn(chA,HIGH,timeOut);  //Read and store channel 1
  duree_impulsion_TMP[CANAL2] = pulseIn(chB,HIGH,timeOut);
  duree_impulsion_TMP[CANAL3] = map(pulseIn(chC,HIGH,timeOut),1000,2000,1000,MaxEscSpeed);
  duree_impulsion_TMP[CANAL4] = pulseIn(chD,HIGH,timeOut);
  duree_impulsion_TMP[CANAL5] = pulseIn(chE,HIGH,timeOut);

  if(duree_impulsion_TMP[CANAL1] != 0)
  {
    duree_impulsion[CANAL1] = duree_impulsion_TMP[CANAL1];
  }

  if(duree_impulsion_TMP[CANAL2] != 0)
  {
    duree_impulsion[CANAL2] = duree_impulsion_TMP[CANAL2];
  }

  if(duree_impulsion_TMP[CANAL3] != 0)
  {
    duree_impulsion[CANAL3] = duree_impulsion_TMP[CANAL3];
  }

  if(duree_impulsion_TMP[CANAL4] != 0)
  {
    duree_impulsion[CANAL4] = duree_impulsion_TMP[CANAL4];
  }

  if(duree_impulsion_TMP[CANAL5] != 0)
  {
    duree_impulsion[CANAL5] = duree_impulsion_TMP[CANAL5];
  }
}

void mode()
{
  if(duree_impulsion[CANAL5] > 1800)
  {
    // mode normal
    MaxEscSpeed = 1500;
    mode_stabilise = true;
  }
  else if(duree_impulsion[CANAL5] < 1200)
  {
    // mode acro
    MaxEscSpeed = 2000;
    mode_stabilise = false;
  }
  else
  {
    // mode sport
    MaxEscSpeed = 2000;
    mode_stabilise = true;
  }
}
