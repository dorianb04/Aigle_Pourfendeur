/*
@AUTOR : Stanislas Brusselle
Code adapté a l'ATmega 328p

Branchements --------------------------------------------------------------------------------------------
RC : 
Channel 1 -> 5, Channel 2 -> 6, Channel 3 -> 7, Channel 4 -> 8;

IMU : 
SDA -> SDA, SCL -> SCL, VIN -> 3.3v, GND -> GND;

ESC : 
esc1 -> 4, esc2 -> 5, esc3 -> 6, esc4 -> 7;

Lien utile --------------------------------------------------------------------------------------------
https://www.firediy.fr/article/realisation-d-un-drone-a-base-d-arduino-chapitre-1
*/

//LIBRAIRIES --------------------------------------------------------------------------------------------
#include <Wire.h>
#include "Kalman_etendu.h"

//CONSTANTES --------------------------------------------------------------------------------------------
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     
#define Y           1     
#define Z           2   

#define MPU_ADDRESS 0x68  
#define FREQ        250   
#define SSF_GYRO    65.5  

//VARIABLES --------------------------------------------------------------------------------------------
//RC ------------------------ 
char mode = '0';

//IMU ------------------------ 
volatile byte previous_state[5];

volatile unsigned int motor_pulse[4] = {1500, 1500, 1000, 1500};

volatile unsigned long current_time;
volatile unsigned long timer[4]; 


int mode_mapping[4];

int gyro_raw[3] = {0,0,0};
long gyro_offset[3] = {0, 0, 0};
float gyro_angle[3]  = {0,0,0};

int acc_raw[3] = {0 ,0 ,0};
float acc_angle[3] = {0,0,0};
long acc_total_vector;
float angle_IMU[3] = {0, 0, 0};
float data_measures[3] = {0, 0, 0};
int temperature;

boolean initialized;

unsigned int  period; 
unsigned long loop_timer;
unsigned long now, difference;

//ESC ------------------------ 

unsigned long pulse_ESC1 = 1000,
              pulse_ESC2 = 1000,
              pulse_ESC3 = 1000,
              pulse_ESC4 = 1000;

//PID ------------------------ 
float pid_consignes[3] = {0, 0, 0}; 
float errors[3];                     
float delta_err[3]      = {0, 0, 0}; 
float error_sum[3]      = {0, 0, 0}; 
float previous_error[3] = {0, 0, 0}; 

float Kp[3] = {4.0, 1.3, 1.3};   
float Ki[3] = {0.02, 0.04, 0.04}; 
float Kd[3] = {0, 18, 18};        

//VOID SETUP --------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(9600);
    Wire.begin();
//Regle l'horloge à 400
    TWBR = 12; 
//PLace les broches liées au canaux 5, 6, 7, 8 en entrée
    DDRD |= B11110000;

//Calibre l'IMU
    setupMpu6050Registers();
    calibrateMpu6050();
    configureChannelMapping();

//Accède au registres pour créer une routine d'interruption sur les broches 5, 6, 7 et 8 (broches liées aux canaux de communication)
    PCICR  |= (1 << PCIE0);  
    PCMSK0 |= (1 << PCINT0); //pin 5
    PCMSK0 |= (1 << PCINT1); //pin 6
    PCMSK0 |= (1 << PCINT2); //pin 7
    PCMSK0 |= (1 << PCINT3); //pin 8
    
//Variable d'échantillonnage 
    period = (1000000/FREQ) ; 
    loop_timer = micros();
}

//VOID LOOP --------------------------------------------------------------------------------------------

void loop() {
    readSensor();
    calculateAngles();
    calculate_Consignes();
    calculate_Errors();
    pidController();
    applyMotorSpeed();
}

//FONCTIONS --------------------------------------------------------------------------------------------

//IMU ------------------------ 

void resetGyroAngles() {
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}

void setupMpu6050Registers() {

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

void calibrateMpu6050() {
    int samples = 2000;

    for (int i = 0; i < samples; i++) {
        readSensor();

        gyro_offset[X] += gyro_raw[X];
        gyro_offset[Y] += gyro_raw[Y];
        gyro_offset[Z] += gyro_raw[Z];

        PORTD |= B11110000;      
        delayMicroseconds(1000); 
        PORTD &= B00001111;      
        delay(3);
    }

    gyro_offset[X] /= samples;
    gyro_offset[Y] /= samples;
    gyro_offset[Z] /= samples;
}

void readSensor() {
    Wire.beginTransmission(MPU_ADDRESS); 
    Wire.write(0x3B);                    
    Wire.endTransmission();              
    Wire.requestFrom(MPU_ADDRESS,14);    
    while(Wire.available() < 14);

    acc_raw[X]  = Wire.read() << 8 | Wire.read(); 
    acc_raw[Y]  = Wire.read() << 8 | Wire.read(); 
    acc_raw[Z]  = Wire.read() << 8 | Wire.read(); 
    temperature = Wire.read() << 8 | Wire.read();
    gyro_raw[X] = Wire.read() << 8 | Wire.read(); 
    gyro_raw[Y] = Wire.read() << 8 | Wire.read(); 
    gyro_raw[Z] = Wire.read() << 8 | Wire.read(); 
}

void calculateGyroAngles() {
    // Subtract offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];
    gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
    gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); 
    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
}

void calculateAccelerometerAngles() {

    acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));
    if (abs(acc_raw[X]) < acc_total_vector) {
        acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); 
    }

    if (abs(acc_raw[Y]) < acc_total_vector) {
        acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
    }
}

void calculateAngles() {
    calculateGyroAngles();
    calculateAccelerometerAngles();

    if (initialized) {
//Utilisation d'un filtre complémentaire 
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
    } else {
        resetGyroAngles();
        initialized = true;
    }

    data_measures[ROLL]  = data_measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
    data_measures[PITCH] = data_measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
    data_measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; 

    angle_IMU[ROLL]  = 0.7 * angle_IMU[ROLL]  + 0.3 * gyro_raw[X] / SSF_GYRO;
    angle_IMU[PITCH] = 0.7 * angle_IMU[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO;
    angle_IMU[YAW]   = 0.7 * angle_IMU[YAW]   + 0.3 * gyro_raw[Z] / SSF_GYRO;
}

//PID ------------------------ 

float calculate_Consigne(float angle, int channel_pulse) {
    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
    float consigne    = 0;

    // Need a dead band of 16µs for better result
    if (channel_pulse > 1508) {
        consigne = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        consigne = channel_pulse - 1492;
    }

    consigne -= level_adjust;
    consigne /= 3;

    return consigne;
}

float calculateYaw_Consigne(int yaw_pulse, int throttle_pulse) {
    float consigne = 0;

    if (throttle_pulse > 1050) {
        consigne = calculate_Consigne(0, yaw_pulse);
    }

    return consigne;
}

void calculate_Consignes() {
    pid_consignes[YAW]   = calculateYaw_Consigne(motor_pulse[mode_mapping[YAW]], motor_pulse[mode_mapping[THROTTLE]]);
    pid_consignes[PITCH] = calculate_Consigne(data_measures[PITCH], motor_pulse[mode_mapping[PITCH]]);
    pid_consignes[ROLL]  = calculate_Consigne(data_measures[ROLL], motor_pulse[mode_mapping[ROLL]]);
}

void calculate_Errors(){

    errors[YAW]   = angle_IMU[YAW]   - pid_consignes[YAW];
    errors[PITCH] = angle_IMU[PITCH] - pid_consignes[PITCH];
    errors[ROLL]  = angle_IMU[ROLL]  - pid_consignes[ROLL];
    
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);

    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];
}

void pidController() {
  
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    int   throttle     = motor_pulse[mode_mapping[THROTTLE]];

    pulse_ESC1 = throttle;
    pulse_ESC2 = throttle;
    pulse_ESC3 = throttle;
    pulse_ESC4 = throttle;                          

    if (throttle >= 1012) {
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        yaw_pid   = minMax(yaw_pid, -400, 400);
        pitch_pid = minMax(pitch_pid, -400, 400);
        roll_pid  = minMax(roll_pid, -400, 400);

        pulse_ESC1 = throttle - roll_pid - pitch_pid + yaw_pid;
        pulse_ESC2 = throttle + roll_pid - pitch_pid - yaw_pid;
        pulse_ESC3 = throttle - roll_pid + pitch_pid - yaw_pid;
        pulse_ESC4 = throttle + roll_pid + pitch_pid + yaw_pid;
    }

    pulse_ESC1 = minMax(pulse_ESC1, 1100, 2000);
    pulse_ESC2 = minMax(pulse_ESC2, 1100, 2000);
    pulse_ESC3 = minMax(pulse_ESC3, 1100, 2000);
    pulse_ESC4 = minMax(pulse_ESC4, 1100, 2000);
}

void resetPidController() {
    errors[YAW]   = 0;
    errors[PITCH] = 0;
    errors[ROLL]  = 0;

    error_sum[YAW]   = 0;
    error_sum[PITCH] = 0;
    error_sum[ROLL]  = 0;

    previous_error[YAW]   = 0;
    previous_error[PITCH] = 0;
    previous_error[ROLL]  = 0;
}

//RC ------------------------ 

void configureChannelMapping() {
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}


ISR(PCINT0_vect) {
        current_time = micros();

        // Channel 1 -------------------------------------------------
        if (PINB & B00000001) {                                        
            if (previous_state[CHANNEL1] == LOW) {                     
                previous_state[CHANNEL1] = HIGH;                      
                timer[CHANNEL1] = current_time;                        
            }
        } 
        else if (previous_state[CHANNEL1] == HIGH) {                 
            previous_state[CHANNEL1] = LOW;                            
            motor_pulse[CHANNEL1] = current_time - timer[CHANNEL1];   
        }

        // Channel 2 -------------------------------------------------
        if (PINB & B00000010) {                                        
            if (previous_state[CHANNEL2] == LOW) {                     
                previous_state[CHANNEL2] = HIGH;                       
                timer[CHANNEL2] = current_time;                        
            }
        } 
        else if (previous_state[CHANNEL2] == HIGH) {                
            previous_state[CHANNEL2] = LOW;                            
            motor_pulse[CHANNEL2] = current_time - timer[CHANNEL2];   
        }

        // Channel 3 -------------------------------------------------
        if (PINB & B00000100) {                                        
            if (previous_state[CHANNEL3] == LOW) {                     
                previous_state[CHANNEL3] = HIGH;                       
                timer[CHANNEL3] = current_time;                        
            }
        } 
        else if (previous_state[CHANNEL3] == HIGH) {                 
            previous_state[CHANNEL3] = LOW;                            
            motor_pulse[CHANNEL3] = current_time - timer[CHANNEL3];   
        }

        // Channel 4 -------------------------------------------------
        if (PINB & B00001000) {                                        
            if (previous_state[CHANNEL4] == LOW) {                     
                previous_state[CHANNEL4] = HIGH;                       
                timer[CHANNEL4] = current_time;                        
            }
        } 
        else if (previous_state[CHANNEL4] == HIGH) {                 
            previous_state[CHANNEL4] = LOW;                            
            motor_pulse[CHANNEL4] = current_time - timer[CHANNEL4];   
        }
}

//ESC ------------------------ 

void stopAll() {
    pulse_ESC1 = 1000;
    pulse_ESC2 = 1000;
    pulse_ESC3 = 1000;
    pulse_ESC4 = 1000;
}

void applyMotorSpeed() {

    while ((now = micros()) - loop_timer < period);
    loop_timer = now;
    PORTD |= B11110000;
    while (PORTD >= 16) {
        now        = micros();
        difference = now - loop_timer;

        if (difference >= pulse_ESC1) PORTD &= B11101111; 
        if (difference >= pulse_ESC2) PORTD &= B11011111; 
        if (difference >= pulse_ESC3) PORTD &= B10111111; 
        if (difference >= pulse_ESC4) PORTD &= B01111111; 
    }
}

//MATHS ------------------------ 

float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}
