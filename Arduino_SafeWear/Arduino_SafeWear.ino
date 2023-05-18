#include "Arduino_BMI270_BMM150.h" //Gyroscope et accélérometre
#include <ArduinoBLE.h> // Bluetooth
#include <Arduino_HS300x.h> //Temperature et humidité

float old_temp = 0;
float old_hum = 0;

float x, y, z;
int degreesX = 0;
int degreesY = 0;
int degreesZ = 0;

int gdegreesX = 0;
int gdegreesY = 0;
int gdegreesZ = 0;

float gyro_x, gyro_y, gyro_z;

const char BLE_SERVICE_UUID[] = "181C";
const char BLE_CHARACTERISTIC_UUID[] = "2A6E";
const char BLE_CHARACTERISTIC_UUID_TEMP[] = "2A6F";
const char BLE_CHARACTERISTIC_UUID_HUMIDITY[] = "2A6D";
BLEService bleService(BLE_SERVICE_UUID);
const char BLE_SERVICE_UUID_V[] = "180A";
// BLEService environnementService("180A");
BLEService environnementService(BLE_SERVICE_UUID);
//BLEFloatCharacteristic bleCharacteristic(BLE_CHARACTERISTIC_UUID, BLERead | BLENotify);
BLEFloatCharacteristic humidityCharacteristic(BLE_CHARACTERISTIC_UUID_HUMIDITY, BLERead | BLENotify);
BLEFloatCharacteristic temperatureCharacteristic(BLE_CHARACTERISTIC_UUID_TEMP, BLERead | BLENotify);
BLEFloatCharacteristic accelerationCharacteristic(BLE_CHARACTERISTIC_UUID, BLERead | BLENotify);

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  if (!BLE.begin()) {
    Serial.println("Erreur de démarrage du module BLE");
    while (1)
      ;
  }

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1)
      ;
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  Serial.println("-----------");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");


  Serial.println(" --- Initialisation BLE --- ");
  BLE.setLocalName("SafeWear");
  /*BLE.setAdvertisedService(bleService);
  bleService.addCharacteristic(bleCharacteristic);
  BLE.addService(bleService);
  bleCharacteristic.writeValue(0);
  */
 //BLE.setAdvertisedService(environnementService);
  environnementService.addCharacteristic(humidityCharacteristic);
  environnementService.addCharacteristic(temperatureCharacteristic);
  environnementService.addCharacteristic(accelerationCharacteristic);

  BLE.addService(environnementService);

  humidityCharacteristic.writeValue(0);
  temperatureCharacteristic.writeValue(0);
  accelerationCharacteristic.writeValue(0);


  BLE.advertise();
  delay(2000);
}

void loop() {
  BLEDevice central = BLE.central();
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    if (x > 0.1) {
      degreesX = map(x, 0, 1, 0, 90);
      Serial.print("Tilting up ");
      Serial.print(degreesX);
      Serial.print("  degrees");
    }
    if (x < -0.1) {
      degreesX = map(x, 0, -1, 0, 90);
      Serial.print("Tilting down ");
      Serial.print(degreesX);
      Serial.print("  degrees");
    }
    if (y > 0.1) {
      degreesY = map(y, 0, 1, 0, 90);
      Serial.print("Tilting left ");
      Serial.print(degreesY);
      Serial.print("  degrees");
    }
    if (y < -0.1) {
      degreesY = map(y, 0, -1, 0, 90);
      Serial.print("Tilting right ");
      Serial.print(degreesY);
      Serial.print("  degrees");
    }
    if (z > 0.1) {
      degreesZ = map(z, 0, 1, 0, 90);
      Serial.print("Tilting up ");
      Serial.print(degreesZ);
      Serial.print("  degrees");
    }
    if (z < -0.1) {
      degreesZ = map(z, 0, -1, 0, 90);
      Serial.print("Tilting z ");
      Serial.print(degreesZ);
      Serial.print("  degrees");
    }
    //Gyroscope

    float gx, gy, gz;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      if (gx > 0.1) {
        gdegreesX = map(gx, 0, 800, 0, 90);
        Serial.print("Gyro up ");
        Serial.print(gdegreesX);
        Serial.print("  degrees");
      }
      if (gx < -0.1) {
        gdegreesX = map(gx, 0, -800, 0, 90);
        Serial.print("Gyro down ");
        Serial.print(gdegreesX);
        Serial.print("  degrees");
      }
      if (y > 0.1) {
        gdegreesY = map(y, 0, 800, 0, 90);
        Serial.print("Gyro left ");
        Serial.print(y);
        Serial.print("  degrees");
      }
      if (y < -0.1) {
        gdegreesY = map(y, 0, -800, 0, 90);
        Serial.print("Gyro right ");
        Serial.print(y);
        Serial.print("  degrees");
      }
      if (z > 0.1) {
        gdegreesZ = map(z, 0, 800, 0, 90);
        Serial.print("Gyro up ");
        Serial.print(gdegreesZ);
        Serial.print("  degrees");
      }
      if (z < -0.1) {
        gdegreesZ = map(z, 0, -800, 0, 90);
        Serial.print("Gyro z ");
        Serial.print(gdegreesZ);
        Serial.println("  degrees");
      }

      int GyroTotal = gdegreesZ + gdegreesY + gdegreesX;
      Serial.print("GyroTotal: ");
      Serial.println(GyroTotal);
      int AccelTotal = degreesX + degreesY + degreesX;
      Serial.print("AccelTotal: ");
      Serial.println(AccelTotal);     

      float total_acc = sqrt(pow(degreesX, 2) + pow(degreesY, 2) + pow(degreesZ, 2)) - 9.81;

      // Calcul de la correction due à la rotation
      gdegreesX += gdegreesX;
      gdegreesY += gdegreesY;
      gdegreesZ += gdegreesZ;

      float correction = sqrt(pow(gdegreesX, 2) + pow(gdegreesY, 2) + pow(gdegreesZ, 2)) * 0.0174533;
Serial.print("Correction: ");
      Serial.println(correction);
      // Application de la correction à l'accélération totale
      total_acc -= correction;
      Serial.print("Total_ acc: ");
      Serial.println(total_acc);
      // Vérification si l'accélération est inférieure à une certaine valeur seuil
      // if (total_acc < 3.0) {
      if (total_acc > 140) {
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
        Serial.println("Chute détectée !");
        // Insérer ici le code pour gérer la détection de chute
      } else {
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
      }





      float temperature = fetch_temp();
      float humidity = fetch_humidity();
      //Envoyer au bluetooth
      // if (central) {
      //   while (central.connected()) {
      //     Serial.print("Données envoyées au PC: ");
      //     Serial.println(temperature);
      //     bleCharacteristic.writeValue(temperature);
      //   }
      // }

      if (central) {
        if(central.connected()) {
          //Serial.print("Données envoyées au PC: ");
          // Serial.println(temperature);
          float temp=floor(10*temperature+0.5)/10;
          humidityCharacteristic.writeValue(humidity);
          temperatureCharacteristic.writeValue(temp);
          accelerationCharacteristic.writeValue(total_acc);
          //bleCharacteristic.writeValue(temperature);
        }else{
         // Serial.println("Central pas connecte");
        }
      }else{
       // Serial.println("Pas de central");
      }
    }else{
      Serial.println("Gyroscope pas connecte");
    }
  }else{
    Serial.println("Accelerometre pas connecte");
  }



  delay(5);
}

float fetch_temp() {

  float temperature = HS300x.readTemperature();
  
  if (abs(old_temp - temperature) >= 0.5) {
    // print each of the sensor values
    // Serial.print("Temperature = ");
    // Serial.print(temperature);
    // Serial.println(" °C");

    // Serial.print("Humidity    = ");
    // Serial.print(humidity);
    // Serial.println(" %");

    // print an empty line
    //Serial.println();
    return(temperature);
  }
}

float fetch_humidity(){
  float humidity = HS300x.readHumidity();
  Serial.println("Humidité: ");
  Serial.println(humidity);
  if (abs(old_hum - humidity) >= 1){
    return(humidity);
}
}