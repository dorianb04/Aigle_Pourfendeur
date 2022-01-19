/*A faire : 
* Calibrage gyroscope (optionnel)
* Conversion données brutes gyroscope (degrées/secondes) (optionnel)

* Communication arduino vers arduino par protocole UART
* Configurer les coefficients PID 
* Déterminer les branchements
* Ne pas oublier de dé-souder les joysticks
* Configuration boutons et modes. 

 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
 
#define None 0
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define NMAX 200

SoftwareSerial gps(8, 7);
RF24 radio(3, 2);   // nRF24L01 (CE, CSN)

//constantes et variables de communication radio. 
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

//Constantes et variables de déplacement du drone et autamatismes de vol. 
float erreur_precedente_x=0,erreur_x=0,variation_erreur_x=0,commande_x=0,somme_erreurs_x=0; 
float erreur_precedente_y=0,erreur_y=0,variation_erreur_y=0,commande_y=0,somme_erreurs_y=0; 
float erreur_precedente_z=0,erreur_z=0,variation_erreur_z=0,commande_z=0,somme_erreurs_z=0; 
//Les paramètres du PID sont à ré-ajuster. 
float Kp[3], Ki[3], Kd[3]; 
float refx=0,refy=0, refz=0;

//constante de battery et compensation de déchargement batterie. 
int battery_voltage = 0;

//Initilaisation des moteurs brushless (presque le même comportement que des servomoteurs). 
Servo ESC1; 
Servo ESC2;
Servo ESC3;
Servo ESC4;

//Initialisation des structures de discution drone-télécommande. 
typedef struct Data_PACKAGE {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
}Data_package;

typedef struct PACKAGE_RETURN {
  float Baro_X;
  float Baro_Y;
  float Baro_Z;
  float Acc_X;
  float Acc_Y;
  float Acc_Z;
  float Temperature; 
  float Altitude; 
  float Pression; 
  char Coord_GPS_RMC[NMAX];
  int Batterie;
}Package_return;

Data_package data;
Package_return data_DRONE; 

void setup(){
  
  Serial.begin(9600);
  radio.begin();
  gps.begin(9600);
  delay(2000);
   
  Serial.println("Demarrage : IMU");
  Serial.println("Demarrage : capteur pression/temperature");
  Serial.println("Demarrage : GPS trame RMC");

  if (!IMU.begin()) {
    Serial.println("Echec : demarrage IMU!");
    while (1);
  }

  if (!BARO.begin()) {
    Serial.println("Echec : demarrage capteur pression/temperature !");
    while (1);
  }

//Le GPS utilise le protocole NMEA. 
  gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
  gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
  gps.println(PMTK_Q_RELEASE);
  
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //Initialise le modul en recepteur. 

  Serial.println("Initialisation achevee");

  ESC1.attach(None); //PIN A DEFINIR.
  ESC2.attach(None); //PIN A DEFINIR.
  ESC3.attach(None); //PIN A DEFINIR.
  ESC4.attach(None); //PIN A DEFINIR.

//compléter refx, refy et refz par les valeurs d'initialisation du gyroscope. 
}

void loop(){
  
//L'impulsion électrique de controle des Brushless motors se fait entre 1000 et 2000 µs.  
//J'établi une relation entre les valeurs de control des moteurs et les valeurs prises par les variables des joysticks. 
//Mesures 10 à 400 seront peut être à changer pour éviter que le drone ne se retourne. 
  float commande_j1PotY_up_down = map(data.j1PotY, 0, 255, 1000, 2000);
  float commande_j1PotX_right = map(data.j1PotX, 127, 255, 10, 400);
  float commande_j1PotX_left = map(data.j1PotX, 0, 127, 10, 400);
  
  float commande_j2PotY_front = map(data.j2PotY, 127, 255, 10, 400);
  float commande_j2PotY_back = map(data.j2PotY, 0, 127, 10, 400);
  float commande_j2PotX_right = map(data.j2PotX, 127, 255, 10, 400);
  float commande_j2PotX_left = map(data.j2PotX, 0, 127, 10, 400);

//Valeurs de l'impulsion électrique à envoyer aux moteurs. 
  float pulse_length_esc1;
  float pulse_length_esc2;
  float pulse_length_esc3;
  float pulse_length_esc4;
  
///////////////////////////////////////////////////////////////////////////////////////////////////
//PARTIE RECU ET INTERPRETATION DU PACKAGE DONNEES DE TELECOMMANDE. 
///////////////////////////////////////////////////////////////////////////////////////////////////

  radio.startListening(); //Initialise le module en recepteur. 

// Le if agit comme une sorte de sécurité, si le signal est perdu ou inchangé : retourne aux paramètres de controle initiaux.
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) {
    resetData(); 
  }

//Check s'il y a des données à recevoir. 
  if (radio.available()) {
    radio.read(&data, sizeof(Data_package));
    lastReceiveTime = millis();//Marque les données comme reçues à cet instant t. 
  }

//////////////////////////////////////////////////////////////////////////////////////////////////
//PARTIE CONTROL ET AUTOMATISMES DE VOL. 
//////////////////////////////////////////////////////////////////////////////////////////////////

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data_DRONE.Baro_X, data_DRONE.Baro_Y, data_DRONE.Baro_Z);
  }
//J'utilise un correcteur PID sur les axes x, y et z.
  erreur_x = data_DRONE.Baro_X - refx;
  erreur_y = data_DRONE.Baro_Y - refy;
  erreur_z = data_DRONE.Baro_Z - refz;
  
  somme_erreurs_x +=erreur_x;
  somme_erreurs_y +=erreur_y;
  somme_erreurs_z +=erreur_z;

  variation_erreur_x = erreur_x - erreur_precedente_x;
  variation_erreur_y = erreur_y - erreur_precedente_y;
  variation_erreur_z = erreur_z - erreur_precedente_z;

  commande_x = Kp[0]*erreur_x + Ki[0]*somme_erreurs_x + Kd[0]*variation_erreur_x;
  commande_y = Kp[1]*erreur_x + Ki[1]*somme_erreurs_y + Kd[1]*variation_erreur_y;
  commande_z = Kp[2]*erreur_z + Ki[2]*somme_erreurs_z + Kd[2]*variation_erreur_z;  

  erreur_precedente_x = erreur_x;
  erreur_precedente_y = erreur_y;
  erreur_precedente_z = erreur_z; 

//commande_x et commande_y sont les variables de correction de trajectoire. 

  pulse_length_esc1 = commande_j1PotY_up_down - commande_x - commande_y + commande_z + commande_j2PotY_back + commande_j2PotX_left + commande_j1PotX_left;
  pulse_length_esc2 = commande_j1PotY_up_down + commande_x - commande_y - commande_z + commande_j2PotY_back + commande_j2PotX_right + commande_j1PotX_right;
  pulse_length_esc3 = commande_j1PotY_up_down - commande_x + commande_y - commande_z + commande_j2PotY_front + commande_j2PotX_left + commande_j1PotX_right;
  pulse_length_esc4 = commande_j1PotY_up_down + commande_x + commande_y + commande_z + commande_j2PotY_front + commande_j2PotX_right + commande_j1PotX_left;

  pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
  pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
  pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
  pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
  
  ESC1.write(pulse_length_esc1);
  ESC2.write(pulse_length_esc2);
  ESC3.write(pulse_length_esc3);
  ESC4.write(pulse_length_esc4);

///////////////////////////////////////////////////////////////////////////////////////////////////
//PARTIE COMPENSATION PERTE DE BATTERIE
///////////////////////////////////////////////////////////////////////////////////////////////////

  if (isBatteryConnected()) {
    
    int coeff = ((1240 - battery_voltage) / (float) 3500);
    
    pulse_length_esc1 += pulse_length_esc1 * coeff;
    pulse_length_esc2 += pulse_length_esc2 * coeff;
    pulse_length_esc3 += pulse_length_esc3 * coeff;
    pulse_length_esc4 += pulse_length_esc4 * coeff;
  }
  
///////////////////////////////////////////////////////////////////////////////////////////////////
//PARTIE ENVOIE PACKAGE DONNEES DRONE VERS TELECOMMANDE. 
///////////////////////////////////////////////////////////////////////////////////////////////////
  Traitement_data_DRONE(data_DRONE);
  radio.write(&data_DRONE, sizeof(Data_package));
}

void resetData() {
//Reset les commandes en cas d'interruption du signal. 
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}

bool isBatteryConnected() {
    // J'applique un simple filtre passe-bas pour filtrer le signal (Fc ≈ 10Hz et gain de ~2.5dB dans la bande passante)
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    return battery_voltage < 1240 && battery_voltage > 800;
}

void Traitement_data_DRONE(Package_return data_D){
//Envoie les données relevées par le drone à la télécommande. 
  int i=0;
  int VAL_STOP=0;
  char RMC[NMAX];
  
//Capteur LSM9DS1. 
/*
//Ignoré car déjà calculé avant pour l'équilibrage de l'assiette. 
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data_D.Baro_X, data_D.Baro_Y, data_D.Baro_Z);
  }
*/
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(data_D.Acc_X, data_D.Acc_Y, data_D.Acc_Z);
  } 
   
//capteur LPS22HB. 
  data_DRONE.Pression = BARO.readPressure();
  data_DRONE.Altitude = 44330 * ( 1 - pow(data_DRONE.Pression/101.325, 1/5.255)); 
  data_D.Temperature = BARO.readTemperature();

  data_D.Batterie = battery_voltage; 
  
//Capteur GPS. 
//$__RMC,heure,validité des donnees,latitude,pôle latitude,longitude,pôle longitude,vitesse en noeuds,,date,,,A*7B
    while(VAL_STOP<1){
    if (gps.available()) {

      delay(10);
      
      char c = gps.read(); 
      data_D.Coord_GPS_RMC[i]=c;
     
      i++;
      
      if(c=='\n'){
        VAL_STOP+=1;
      }     
     }
  } 
}

float minMax( float valeur_encadree, float minimum, float maximum){
  if(valeur_encadree > maximum){
    Serial.println("Valeur encadree trop elevee");
    return maximum;
  }
  if(valeur_encadree < minimum){
    Serial.println("Valeur encadree trop faible");
    return minimum;
  }
}
