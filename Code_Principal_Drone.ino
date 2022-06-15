/* 
@AUTOR : Stanislas Brusselle
Code adapté a l'ATmega 328p

Branchements IMU: 
Leonardo : SCL -> 3, SDA -> 2 ou SCL -> SCL, SDA -> SDA
Nano Every : SCL -> D19, SDA -> D18
Vin -> 3,3v
GND -> GND 
 */


#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <SPI.h>

////////////////////////////////////////////////////////CONSTANTES////////////////////////////////////////////////////////
#define PPM_FrLen 27000  
#define PPM_PulseLen 400
#define sigPin 2 //C'est la pin de l'arduino qui recevra les info commande
//Cette pin est primordiale car elle permet de gérer les interruptions.

#define YAW   0
#define PITCH 1
#define ROLL  2
#define THROTTLE 3

#define X     0     
#define Y     1     
#define Z     2     

#define MPU_ADDRESS 0x68  
#define FREQ        250   
#define SSF_GYRO    65.5  

////////////////////////////////////////////////////////VARIABLES////////////////////////////////////////////////////////

//ESC////////////////////////////////////////////////////////


Servo ESC1; 
Servo ESC2;
Servo ESC3;
Servo ESC4;

Servo Dir_CAM;


//Télécommande////////////////////////////////////////////////////////
RF24 radio(2, 3); //Branchements peut être à revoir. 
const uint64_t pipeOut = 0xE8E8F0F0E1LL; 

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

volatile unsigned int pulse_length[4] = {1000, 1000, 1000, 1000};

typedef struct Data_PACKAGE {
  
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;

  int val_IA;
  
}Data_package;

Data_package data;

//Variables de l'IMU////////////////////////////////////////////////////////
int gyro_raw[3] = {0, 0, 0};  
long gyro_offset[3] = {0, 0, 0};
float gyro_angle[3]  = {0, 0, 0};
int acc_raw[3] = {0 , 0 , 0};
float acc_angle[3] = {0, 0, 0};

long acc_total_vector;
float measures[3] = {0, 0, 0};

int temperature;

boolean initialized;

unsigned int  period; 
unsigned long loop_timer;

float angular_motions[3] = {0, 0, 0};

//Signal ESC////////////////////////////////////////////////////////

unsigned long now, difference;

unsigned long pulse_length_esc1 = 1000,
        pulse_length_esc2 = 1000,
        pulse_length_esc3 = 1000,
        pulse_length_esc4 = 1000;

//Correcteur PID////////////////////////////////////////////////////////

float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll

float errors[3];                     
float delta_err[3]      = {0, 0, 0}; 
float error_sum[3]      = {0, 0, 0}; 
float previous_error[3] = {0, 0, 0}; 

float Kp[3] = {4.0, 1.3, 1.3};   //A REGLER SELON LES DIMENSIONS DE NOTRE DRONE
float Ki[3] = {0.02, 0.04, 0.04}; 
float Kd[3] = {0, 18, 18};        

//Batterie////////////////////////////////////////////////////////

int battery_voltage;

////////////////////////////////////////////////////////VOID SETUP////////////////////////////////////////////////////////

void setup() {

  Wire.begin();
  
//Cette commande fonctionne avec une arduino Uno mais pas avec une Arduino Nano Every. Il reste à voir si celle-ci est réellement utile. 
//La commande permet en principe de fixer l'horloge I2C à une vitesse de 400kHz. 
  //TWBR = 12; 

  setupMpu6050Registers();
  calibrateMpu6050();

  radio.begin();
  radio.openReadingPipe(1, pipeOut);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //Initialise le module en recepteur. 
  
  resetData();
  
  ESC1.attach(4); 
  ESC2.attach(5); 
  ESC3.attach(6); 
  ESC4.attach(7); 

  Dir_CAM.attach(13);

  loop_timer = micros();
  period = (1000000 / FREQ) ; 
}

////////////////////////////////////////////////////////VOID LOOP////////////////////////////////////////////////////////

void loop() {
// 1. Map les données reçues par la télécommande aux différentes valeurs qui doivent leur être associées. 
  Remote_control();
  
// 2. Données brutes de l'IMU. 
  readSensor();

// 3. Calcule les angles du drone selon chaque axe. 
  calculateAngles();

// 4. Calcule les consignes de correction selon chaque axe.
  calculateSetPoints();

// 5. En déduis l'erreur entre les set_points et les angles réels. 
  calculateErrors();

// 6. Corrige les impulsions à donner aux ESC. 
  Correcteur_PID();

// 7. Gère la perte de batterie. 
  compensateBatteryDrop();

// 8. Envoie les impulsions déterminées aux ESC. 
    Impulsion_ESC();
}


////////////////////////////////////////////////////////FONCTIONS////////////////////////////////////////////////////////

////////////////////////////////////////////////////////MPU 6050////////////////////////////////////////////////////////
  void setupMpu6050Registers() {
//Permet l'initialisation du MPU 6050

  Wire.beginTransmission(MPU_ADDRESS); 
  Wire.write(0x6B);                    
  Wire.write(0x00);                    
  Wire.endTransmission();              
  
  Wire.beginTransmission(MPU_ADDRESS); 
  Wire.write(0x1B);                    
  Wire.write(0x08);                    
  Wire.endTransmission();              
     
  Wire.beginTransmission(MPU_ADDRESS); 
  Wire.write(0x1C);                    
  Wire.write(0x10);                    
  Wire.endTransmission(); 
          
  Wire.beginTransmission(MPU_ADDRESS); 
  Wire.write(0x1A);                    
  Wire.write(0x03);                    
  Wire.endTransmission();             
}

void calibrateMpu6050(){
//Calibre le MPU 6050 en prenant la moyenne glissante de 2000 relevés. 
//Cette fonction sert à gérer l'offset. 
  
  int max_samples = 2000;
  
  for (int i = 0; i < max_samples; i++) {
    
    readSensor();
    
    gyro_offset[X] += gyro_raw[X];
    gyro_offset[Y] += gyro_raw[Y];
    gyro_offset[Z] += gyro_raw[Z];
    delay(3);
  }
  
  gyro_offset[X] /= max_samples;
  gyro_offset[Y] /= max_samples;
  gyro_offset[Z] /= max_samples;
}

void readSensor() {
//Lit les données brutes renvoyées par l'accéléromètre et le gyroscope et les stock dans les tableaux associés. 

  Wire.beginTransmission(MPU_ADDRESS); 
  Wire.write(0x3B);                   
  Wire.endTransmission();              
  Wire.requestFrom(MPU_ADDRESS, 14);  
  
  while (Wire.available() < 14);
  
  acc_raw[X]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[X] variable
  acc_raw[Y]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Y] variable
  acc_raw[Z]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Z] variable
  temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
  gyro_raw[X] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[X] variable
  gyro_raw[Y] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Y] variable
  gyro_raw[Z] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Z] variable
}
  
void calculateAngles(){
//Fais les corrélation entre les angles donnés par le gyroscope et l'accéléromètre pour renvoyer l'inclinaison de l'assiette
//selon le pitch, yaw et roll. 

  calculateGyroAngles();
  calculateAccelerometerAngles();
  
  if (initialized) {
    
    gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
    gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
  } 
  else {

    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
    initialized = true;
   }
    
  measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
  measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
  measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; 
}
   
void calculateGyroAngles(){
//Utilise les données brutes récupérées par le gyroscope pour les transformer en angles en degrés. 
  
  gyro_raw[X] -= gyro_offset[X];
  gyro_raw[Y] -= gyro_offset[Y];
  gyro_raw[Z] -= gyro_offset[Z];
  
  gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
  gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); 
  
  gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
  gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
 }
    
void calculateAccelerometerAngles(){
//Utilise les données brutes récupérées par l'accéléromètre pour les transformer en angles en degrés. 

  acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));
  
  if (abs(acc_raw[X]) < acc_total_vector) {
    acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI);
  }
  
  if (abs(acc_raw[Y]) < acc_total_vector) {
    acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
  }
}

////////////////////////////////////////////////////////CORRECTEUR PID////////////////////////////////////////////////////////

float calculateSetPoint(float angle, int channel_pulse) {
//Prend en entrée : un angle en degrés et la durée en µs de l'impulsion reçue pour l'axe (comprise entre 1000µs et 2000µs)
//Calcule la consigne d'un angle pour un axe.
//Interprète un angle donné pour en déduire une consigne exploitable par le correcteur PID. 
//Retourne la consigne en degrés/seconde pour la correction.

    float level_adjust = angle * 15;
    float set_point    = 0;

    // On s'accorde une marge 16µs pour de meilleurs résultats
    if (channel_pulse > 1508) {
        set_point = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        set_point = channel_pulse - 1492;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
//Calcul le set_point pour l'axe YAW. 
    float set_point = 0;
    if (throttle_pulse > 1050) {
//Il n'y a pas de notion d'angle sur le YAW car le drone peut tourner sur lui même
        set_point = calculateSetPoint(0, yaw_pulse);
    }
    return set_point;
}

void calculateSetPoints() {
//Calcule le set_point selon tous les axes.
    pid_set_points[YAW]   = calculateYawSetPoint(pulse_length[YAW], pulse_length[THROTTLE]);
    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse_length[PITCH]);
    pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL], pulse_length[ROLL]);
}

void calculateErrors() {
//Calcule les erreurs relatives a chaque axe. 
//Pour calculer les erreurs, se base sur la formule du PID 
  
//Calcule l'erreur actuelle.
    errors[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    errors[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];

//Calcule la somme des erreurs (intégration)
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);

//(dérivation)
    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];
}


void Correcteur_PID() {
//Renvoie les impulsions de correction de l'assiette du drone. 

    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    int   throttle     = pulse_length[THROTTLE];

    pulse_length_esc1 = throttle;
    pulse_length_esc2 = throttle;
    pulse_length_esc3 = throttle;
    pulse_length_esc4 = throttle;

//Ne calcule rien si la commande des gazs (throttle) est égale à 0
    if (throttle >= 1012) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        yaw_pid   = minMax(yaw_pid, -400, 400);
        pitch_pid = minMax(pitch_pid, -400, 400);
        roll_pid  = minMax(roll_pid, -400, 400);

        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;
    }

//Garde les impulsion dans le range acceptable de valeur. 
    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
}

////////////////////////////////////////////////////////COMPENSATION PERTE BATTERIE////////////////////////////////////////////////////////

void compensateBatteryDrop() {
//Compense la perte de batterie du drone en appliquant un coefficient aux impulsions des ESC. 

    if (isBatteryConnected()) {
        int coeff = ((1240 - battery_voltage) / (float) 3500);

        pulse_length_esc1 += pulse_length_esc1 * coeff;
        pulse_length_esc2 += pulse_length_esc2 * coeff;
        pulse_length_esc3 += pulse_length_esc3 * coeff;
        pulse_length_esc4 += pulse_length_esc4 * coeff;
    }
}

bool isBatteryConnected() {
//Vérifie que la batterie soit bien connectée. 
//Renvoie TRUE si la batterie est connectée.
  
    // On applique un simple filtre passe-bas pour filtrer le signal (Fc ≈ 10Hz et gain de ~2.5dB dans la bande passante)
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    return battery_voltage < 1240 && battery_voltage > 800;
}

////////////////////////////////////////////////////////IMPULSION ESC////////////////////////////////////////////////////////

void Impulsion_ESC(){
  ESC1.write(pulse_length_esc1);
  ESC2.write(pulse_length_esc2);
  ESC3.write(pulse_length_esc3);
  ESC4.write(pulse_length_esc4);
}

////////////////////////////////////////////////////////RADIO////////////////////////////////////////////////////////

void Remote_control(){
//Reçoit les remote controls et les interprète pour venir les mettre dans le tableau pulse_lenght[4].

  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { 
    resetData(); //Reset les commandes si la connection est perdue. 
  }

//Vérifie s'il y a des données à recevoir.
  if (radio.available()) {
    radio.read(&data, sizeof(Data_package)); //Lit les données reçues et les stock dans la structure data.
    lastReceiveTime = millis();
  }

  if (AIGLE_POURFENDEUR_DES_PLAINES_ARIDES()==0){

    mapping_commands();
    
  }
  else{
//Ici ce trouveront toute les commande pour le tracking IA
  }
}

void resetData() {
  // Reset l'ensemble des commandes remote. 
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.val_IA = 0;

  mapping_commands();
}

void mapping_commands(){
//Map les valeurs des joysticks aux valeurs aux valeurs d'impulsion des ESC.
    pulse_length[THROTTLE] = map(data.throttle, 0, 255, 1000, 2000);//Pour le throttle.
    pulse_length[YAW] = map(data.yaw, 0, 255, 1000, 2000);//Pour le yaw.
    pulse_length[PITCH] = map(data.pitch, 0, 255, 1000, 2000);//Pour le pitch.
    pulse_length[ROLL] = map(data.roll, 0, 255, 1000, 2000);//Pour le roll.
}

void setupPPM() {
//Initialise la procédure d'interruption commande du drone. 

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);  

//J'utilise ici du langage processeur. Problème : ce langage est prévu pour les processeurs d'arduino uno et non d'arduino Nano Every ! 
//Il va donc falloir que j'aille voir dans les register de la Nano Every pour voir quelle commande processeur sont équivalentes 
//à celles que j'utilise ci-dessous. 
  cli();
  TCCR1A = 0; 
  TCCR1B = 0;

  OCR1A = 100;  
  TCCR1B |= (1 << WGM12); 
  TCCR1B |= (1 << CS11);  
  TIMSK1 |= (1 << OCIE1A); 
  sei();
}

//Cette partie de code ne vient pas de moi, mais permet les interruptions commande du programme. 

#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect){
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)
    state = true;

    if(cur_chan_numb >= channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}


////////////////////////////////////////////////////////FONCTION MATHS////////////////////////////////////////////////////////

float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }
    return value;
}

////////////////////////////////////////////////////////FONCTIONS MODES DE VOL////////////////////////////////////////////////////////

int AIGLE_POURFENDEUR_DES_PLAINES_ARIDES(){
//Permet la sélection des différents mode du drone. 
//Si la fonction renvoie 1, alors les commandes joystick ne seront plus prises en compte. Dans le cas contraire, 
//les commandes joysticks sont toujours utilisées. 
 
  if (data.val_IA == 1) {
    //MODE IA DRONE SUIVI
    return 1;
  }
  return 0;
}
