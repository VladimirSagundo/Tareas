#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <BluetoothSerial.h>
#include <HardwareSerial.h>

BluetoothSerial SerialBT; // Objeto para Bluetooth Serial

String lat = "0", lon = "0";
String idDevice = "NA";
String idUserConnected = "NA";
bool isTracking = false;
bool isEnableRA = true;
bool isEnableDEC = true;
bool isRemoteControl = false;
int stepperAR = 0, stepperDEC = 0, COMD = 0;
String ARH = "0", ARM = "0", ARS = "0.0";
String DECH = "0", DECM = "0", DECS = "0.0";
bool isObjectPosition = false;
float earthSpeed = 16.8011, DRV1R = 60, DRV2R = 60; //Gira a la misma velocidad de la tierra en sentido antihorario

void optimizeMotorSpeed();
bool checkDriverStatus();
float calculateStepsPerDegree(int motor);
void preciseMove(int motor, int steps);
void monitorPerformance();

//Motor AR
#define DIR_PIN_1 4  // Pin DIR para el TMC2209
#define FAN_PIN 22  // Ajusta el número de pin según tu hardware
#define STEP_PIN_1 5  // Pin STEP para el TMC2209
#define EN_PIN_1 21   // Pin EN para el TMC2209
//Motor DEC
#define DIR_PIN_2 13   // Pin DIR para el TMC2209
#define STEP_PIN_2 12  // Pin STEP para el TMC2209
#define EN_PIN_2 15   // Pin EN para el TMC2209

#define R_SENSE 0.11f  // Resistencia de detección (depende del hardware)

HardwareSerial TMC_SERIAL_1(1);  // Usar UART1
HardwareSerial TMC_SERIAL_2(2);  // Usar UART2

TMC2209Stepper driver1(&TMC_SERIAL_1, R_SENSE, 0);  // Inicializa el driver TMC2209
TMC2209Stepper driver2(&TMC_SERIAL_2, R_SENSE, 0);  // Inicializa el driver TMC2209

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// Ajusta los parámetros según tu motor y la configuración
const int spr = 200; // Número de pasos por revolución de tu motor (1.8° por paso).

int microsteps1 = 16; // Número de micropasos (ajustable)
float totalSteps1 = spr * microsteps1; // Pasos totales para una revolución completa
float maxSpeed1 = 150 * microsteps1;
float speed1 = microsteps1 * 100;
float accel1 = microsteps1 * 30;

int microsteps2 = 16; // Número de micropasos (ajustable)
float totalSteps2 = spr * microsteps2; // Pasos totales para una revolución completa
float maxSpeed2 = 150 * microsteps2;
float speed2 = microsteps2 * 100;
float accel2 = microsteps2 * 30;

void setup() {

  SerialBT.begin("ANDROMEDA_V1");

  pinMode(EN_PIN_1, OUTPUT);  // Configurar EN_PIN_1 como salida
  pinMode(EN_PIN_2, OUTPUT);  // Configurar EN_PIN_2 como salida
  pinMode(FAN_PIN, OUTPUT);  // Configura FAN_PIN como salida

  digitalWrite(EN_PIN_1, LOW);  // Activar el primer driver (enable)
  digitalWrite(EN_PIN_2, LOW);  // Activar el segundo driver (enable)

  Serial.begin(115200); //Establece la comunicación en 115200
  delay(500);

  //Configura el primer driver
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);

  // Configura el primer driver TMC2209
  TMC_SERIAL_1.begin(115200, SERIAL_8N1, 19, 18);  // Motor DEC
  TMC_SERIAL_2.begin(115200, SERIAL_8N1, 16, 17);  // Motor AR

  driver1.begin();
  driver1.pdn_disable(true);
  driver1.toff(5);  // Habilita los drivers internos del motor
  driver1.rms_current(600);  // Ajusta la corriente RMS según tu motor
  driver1.en_spreadCycle(false);  // Activa StealthChop para un funcionamiento silencioso
  driver1.pwm_autoscale(true);
  driver2.intpol(true);
  driver1.microsteps(microsteps1);  // Configura los micropasos

  stepper1.setMaxSpeed(maxSpeed1); //Ajusta la velocidad del motor
  stepper1.setSpeed(speed1);
  stepper1.setAcceleration(accel1); //Ajusta la aceleracion del motor
  stepper1.setCurrentPosition(0);  // Reiniciar la posición actual

  //Configura el segundo driver
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);

  // Configura el segundo driver TMC2209
  driver2.begin();
  driver2.pdn_disable(true);
  driver2.toff(5);  // Habilita los drivers internos del motor
  driver2.rms_current(600);  // Ajusta la corriente RMS según tu motor
  driver2.en_spreadCycle(false);  // Activa StealthChop para un funcionamiento silencioso
  driver2.pwm_autoscale(true);
  driver2.intpol(true);
  driver2.microsteps(microsteps2);  // Configura los micropasos

  stepper2.setMaxSpeed(maxSpeed2); //Ajusta la velocidad del motor
  stepper2.setSpeed(speed2);
  stepper2.setAcceleration(accel2); //Ajusta la aceleracion del motor
  stepper2.setCurrentPosition(0);  // Reiniciar la posición actual
  
  int drvV1 = driver1.version();
  int drvV2 = driver2.version();

  Serial.print("UART 1: ");
  Serial.println(drvV1);
  Serial.print("UART 2: ");
  Serial.println(drvV2);

  if(drvV1 < 1 || drvV2 < 1){
    COMD = 0;
  }else{
    COMD = 1;
  }
}
void loop() {

}
// --------------------------------------------------------------------------------------------------------------------                                                     1
void adjustTrackingSpeed(float factor) {
    volatile uint32_t new_speed = (uint32_t)(earthSpeed * factor * 1000);
    
    asm volatile(
        "movi a0, 0x3FF5F000 \n"   // Dirección registro TIMG_T0ALARMLO
        "s32i %0, a0, 0     \n"    // Escribe nuevo valor de velocidad
        "movi a0, 0x3FF5F004 \n"   // Dirección TIMG_T0ALARMHI
        "movi a1, 0          \n"
        "s32i a1, a0, 0      \n"   // Reset parte alta
        : 
        : "r" (new_speed)
        : "a0", "a1"
    );
}

/*Para poder cambiar la configuración*/
void changueDriverConfiguration(int driver, int microSteps, float speed, int maxSpeeds, int accel) {

  Serial.print("Se ha cambiado los micropasos a ");
  Serial.print(microSteps);
  Serial.print(" en el driver ");
  Serial.println(driver);

  switch (driver) {
    case 1: {
        driver1.microsteps(microSteps);  // Configura los micropasos
        stepper1.setMaxSpeed(maxSpeeds); //Ajusta la velocidad del motor
        stepper1.setAcceleration(accel); //Ajusta la aceleracion del motor
        stepper1.setSpeed(speed); 
        break;
      }

    case 2: {
        driver2.microsteps(microSteps);  // Configura los micropasos
        stepper2.setMaxSpeed(maxSpeeds); //Ajusta la velocidad del motor
        stepper2.setAcceleration(accel); //Ajusta la aceleracion del motor
        stepper2.setSpeed(speed);
        break;
      }
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------                         2
uint32_t readPositionSensors() {
    volatile uint32_t sensor_data;
    
    asm volatile(
        "movi a0, 0x3FF44800 \n"   // GPIO_IN_REG
        "l32i %0, a0, 0      \n"   // Lee todos los pines
        "extui %0, %0, 16, 4 \n"   // Extrae bits 16-19 (sensores)
        : "=r" (sensor_data)
        : 
        : "a0"
    );
    
    return sensor_data;
}
// ----------------------------------------------------------------------------------------------------------------------------------------                                                    3

void syncAxisMovement(int steps_ra, int steps_dec) {
    volatile int *ra_pos = &stepperAR;
    volatile int *dec_pos = &stepperDEC;
    
    asm volatile(
        "1:                        \n"
        "addi %0, %0, -1           \n"  // Decrementa RA (usa addi con valor negativo)
        "addi %1, %1, -1           \n"  // Decrementa DEC 
        "bgez %0, 1b               \n"  // Repetir hasta 0
        : "+r" (steps_ra), "+r" (steps_dec)
        : 
        : // No se modifican registros adicionales
    );
    
    *ra_pos += steps_ra;
    *dec_pos += steps_dec;
}


void moveMotorDA(float DECG, float ARG) {

  float angleForStepDEC = 360 / totalSteps1 / DRV1R;
  float angleRecDEC = 0;
  int stepsDEC = (DECG * totalSteps1) / 360 * DRV1R;

  float angleForStepAR = 360 / totalSteps2 / DRV2R;
  float angleRecAR = 0;
  int stepsAR = (ARG * totalSteps2) / 360 * DRV2R;

  // Mover ambos motores a la misma posición al mismo tiempo
  stepper1.moveTo(stepsDEC);
  stepper2.moveTo(stepsAR);

  while (stepper1.isRunning() || stepper2.isRunning()) {
    stepper1.run();
    stepper2.run();
  }

  Serial.print(", Pasos a dar en DEC: ");
  Serial.println(stepsDEC);

  Serial.print("Pasos a dar en AR: ");
  Serial.print(stepsAR);

  double angleCurrentDEC = stepper1.currentPosition() * angleForStepDEC;
  Serial.print("Pasos recorrido en DEC: ");
  Serial.print(angleCurrentDEC);

  double angleCurrentAR = stepper2.currentPosition() * angleForStepAR;
  Serial.print(", pasos recorrido en AR: ");
  Serial.println(angleCurrentAR);

  DynamicJsonDocument jsonDoc(1024);
  JsonObject net = jsonDoc.to<JsonObject>();
  net["status"] = "OK";
  // Convertir a JSON
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  // Enviar JSON por Bluetooth
  SerialBT.println(jsonString);
}
void moveMotorGAD(int DRV, float GRADES) {
  // Mover ambos motores a la misma posición al mismo tiempo
  switch(DRV){

    case 1:{
      stepper1.moveTo(GRADES);
      while (stepper1.isRunning() || stepper2.isRunning()) {
        stepper1.run();
      }
      break;
    }

    case 2:{
      stepper2.moveTo(GRADES);
      while (stepper1.isRunning() || stepper2.isRunning()) {
        stepper2.run();
      }
      break;
    }
  }
}

// ----------------------------------------------- COMPROBAR EL ESTADO DE LOS CONTROLADORES ----------------------------------------------------------------------                     4
void thermalManagement() {
    float temp = 0;
    
    // Lectura del sensor de temperatura interno del ESP32
    asm volatile (
        "movi a0, 0x3FF48000    \n"   // Registro de control del sensor
        "movi a1, 0x80000000    \n"   // Comando para iniciar lectura
        "s32i a1, a0, 0         \n"    // Escribe en el registro
        "1:                     \n"
        "l32i a1, a0, 0         \n"    // Lee estado
        "bbci a1, 31, 1b        \n"    // Espera hasta que esté listo (bit 31)
        "l32i %0, a0, 4         \n"    // Lee valor del registro de datos (0x3FF48004)
        : "=r" (temp)
        :
        : "a0", "a1"
    );
    
    temp = (temp - 32) / 1.8;  // Convertir a grados Celsius (ajuste empírico)
    
    if(temp > 45.0) {
        digitalWrite(FAN_PIN, HIGH);
    } else {
        digitalWrite(FAN_PIN, LOW);
    }
}

void processCommand(String command) {

  if (command.startsWith("GOTO:")) {
    command.remove(0, 5);
    int commaIndex = command.indexOf(',');
    float DECR = command.substring(0, commaIndex).toFloat();
    float AHR = command.substring(commaIndex + 1).toFloat();

    Serial.print("AZ: ");
    Serial.print(AHR);
    Serial.print(" ALT: ");
    Serial.println(DECR);

    moveMotorDA(DECR, AHR);

  }else if(command.startsWith("MDRV:")){

    command.remove(0, 5);
    int commaIndex = command.indexOf(',');
    float drv = command.substring(0, commaIndex).toInt();
    float Ggrades = command.substring(commaIndex + 1).toFloat();

    moveMotorGAD(drv, Ggrades);

  }else if (command.startsWith("GET:")) {
    DynamicJsonDocument jsonDoc(1024);
    JsonObject net = jsonDoc.to<JsonObject>();
    net["isTracking"] = isTracking;
    net["earthSpeed"] = earthSpeed;
    // Convertir a JSON
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    // Enviar JSON por Bluetooth
    SerialBT.println(jsonString);

  }else if (command.startsWith("MODE:")) {

    command.remove(0, 5);
    int commaIndex = command.indexOf(',');
    int mode = command.substring(0, commaIndex).toInt();
    bool is = (command.substring(commaIndex + 1) == "True" || command.substring(commaIndex + 1) == "1");

    switch(mode){
      case 1:{
        Serial.print("El modo seguimiento ha sido ");
        Serial.println(!is ? "desactivado" : "activado");
        isTracking = is;

          if (isTracking) {
            changueDriverConfiguration(2, 256, earthSpeed, 50, 0);
            printDriverStatus();
          } else {

            const float speed = 1600;
            changueDriverConfiguration(2, 16, speed, 2000, 500);
            printDriverStatus();
          }

        break;
      }

      case 2:{
        Serial.print("El driver RA ha sido ");
        Serial.println(!is ? "desactivado" : "activado");
        isEnableRA = is;
        if(isEnableRA){
          digitalWrite(EN_PIN_2, LOW);  // Activar el driver RA
        }else{
          digitalWrite(EN_PIN_2, HIGH);  // Desactivar el driver RA
        }

        stepper1.setCurrentPosition(0);  // Reiniciar la posición actual
        stepper2.setCurrentPosition(0);  // Reiniciar la posición actual 
        break;     
      }

      case 3:{
        Serial.print("El driver DEC ha sido ");
        Serial.println(!is ? "desactivado" : "activado");
        isEnableDEC = is;
        if(isEnableDEC){
          digitalWrite(EN_PIN_1, LOW);  // Activar el driver DEC
        }else{
          digitalWrite(EN_PIN_1, HIGH);  // Desactivar el driver DEC
        }

        stepper1.setCurrentPosition(0);  // Reiniciar la posición actual
        stepper2.setCurrentPosition(0);  // Reiniciar la posición actual 

        break;   
      }
    }

  }else if (command.startsWith("ANDRV1:")){
    SerialBT.println("OK"); // Respuesta reconocible para el C#
  }else if(command.startsWith("GETD:")){
    
    DynamicJsonDocument jsonDoc(1024);
    JsonObject net = jsonDoc.to<JsonObject>();

    net["microsteps1"] = (COMD == 0) ? microsteps1 : driver1.microsteps();
    net["position1"] = stepper1.currentPosition();
    net["isEnableDEC"] = isEnableDEC;
    net["UART1"] =  driver1.version();
    net["DRV1R"] =  DRV1R;

    net["microsteps2"] = (COMD == 0) ? microsteps2 : driver2.microsteps();
    net["position2"] = stepper2.currentPosition();
    net["isEnableRA"] = isEnableRA;
    net["UART2"] =  driver2.version();
    net["DRV2R"] =  DRV2R;

    // Convertir a JSON
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    SerialBT.println(jsonString);
  }else if(command.startsWith("CHGDRV:")){
    command.remove(0, 7);
    int commaIndex = command.indexOf(',');
    int drv = command.substring(0, commaIndex).toInt();
    int microSTP = command.substring(commaIndex + 1).toInt();

    Serial.println("Driver: " + drv);

    switch (drv){
      case 1:{

        microsteps1 = microSTP;
        totalSteps1 = spr * microsteps1; // Pasos totales para una revolución completa
        maxSpeed1 = 150 * microsteps1;
        speed1 = microsteps1 * 100;
        accel1 = microsteps1 * 30;

        changueDriverConfiguration(drv, microsteps1, speed1, maxSpeed1, accel1);
        break;
      }

      case 2:{

        microsteps2 = microSTP;
        totalSteps2 = spr * microsteps2; // Pasos totales para una revolución completa
        maxSpeed2 = 150 * microsteps2;
        speed2 = microsteps2 * 100;
        accel2 = microsteps2 * 30;

        changueDriverConfiguration(drv, microsteps2, speed2, maxSpeed2, accel2);
        break;
      }
    }

  }else if(command.startsWith("SETSP:")){
    command.remove(0, 6);
    float speed = command.toFloat();
    earthSpeed = speed;

    Serial.println("Se ha cambiado la velocidad");
  }else{
    Serial.println("ERROR: Comando no reconocido");
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------                          5
uint32_t encodeCommand(String cmd) {
    volatile uint32_t encoded;
    
    asm volatile(
        "l8ui a2, %1, 0       \n"   // Primer caracter
        "l8ui a3, %1, 1       \n"   // Segundo caracter
        "slli a2, a2, 24      \n"
        "slli a3, a3, 16      \n"
        "or %0, a2, a3        \n"
        : "=r" (encoded)
        : "r" (cmd.c_str())
        : "a2", "a3"
    );
    
    return encoded;
}

void printDriverStatus() {
  Serial.println("===== CONFIGURACIÓN DEL DRIVER TMC2209 =====");

  // Leer microstepping actual
  Serial.print("Microsteps 1: ");
  Serial.println(driver1.microsteps());

  Serial.print("Microsteps 2: ");
  Serial.println(driver2.microsteps());

  // Leer corriente RMS configurada
  Serial.print("Corriente RMS 1: ");
  Serial.println(driver1.rms_current());
  Serial.print("Corriente RMS 2: ");
  Serial.println(driver2.rms_current());

  // Leer velocidad PWM autoscale
  Serial.print("PWM autoscale 1: ");
  Serial.println(driver1.pwm_autoscale());

  Serial.print("PWM autoscale 2: ");
  Serial.println(driver2.pwm_autoscale());

  // Leer modo de operación (StealthChop o SpreadCycle)
  Serial.print("SpreadCycle activado: ");
  Serial.println(driver2.en_spreadCycle());

  // Verificar si está en modo stealthChop
  Serial.print("Modo StealthChop: ");
  Serial.println(driver2.stealth());

  // Estado de los pines
  Serial.print("Driver Enabled (EN_PIN_2): ");
  Serial.println(digitalRead(EN_PIN_2) == LOW ? "Sí" : "No");

  Serial.println("========================================");
  
}
