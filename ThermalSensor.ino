#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SigFox.h>
#include <SPI.h>
#include <MFRC522.h>

//#define WITHSHARPIR
#define WITH_VL53L0X
#define WITH_SIGFOX_TX

#ifdef WITH_VL53L0X
  #include <VL53L0X.h>
  VL53L0X vl53l0x;
  #define DIST_MIN    30
  #define DIST_MAX    40
  #define DIST_OUT   300
  #define DIST_LOOPS   4
#elif defined WITH_SHARPIR
  #include <ZSharpIR.h>
  ZSharpIR ZSharpIR(A1, 20150);
  #define DIST_MIN  290
  #define DIST_MAX  350
  #define DIST_OUT  450
  #define DIST_LOOPS 10
#endif


#define DISTPIN     1
#define REDLEDPIN   5
#define GREENLEDPIN 7
#define RFID_SS     3
#define RFID_RST    4
#define BUZZ_PIN    2


Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MFRC522 rfid(RFID_SS,RFID_RST);
MFRC522::MIFARE_Key key; 


typedef enum {
   WAITFORSOMEONE = 0,
   SOMEONEHERE,
   GOODDISTANCE,
   DISTANCESTABLE,
   WAITFORLEAVING
} state_e;

state_e curentState;

void setup() {
  // for easy reprogramming after manual reset because of low power mode
  //  you can also double click on reset button at anytime to restart on bootloader
  delay(4000);
  Wire.begin();
  
  // Setup
  Serial.begin(9600);
  mlx.begin();
  #ifdef WITH_SHARPIR
    ZSharpIR.setARefVoltage(3300);
  #endif 
  pinMode(DISTPIN,OUTPUT);
  digitalWrite(DISTPIN,LOW);
  pinMode(REDLEDPIN,OUTPUT);
  digitalWrite(REDLEDPIN,LOW);
  pinMode(GREENLEDPIN,OUTPUT);
  digitalWrite(GREENLEDPIN,LOW);
  curentState = WAITFORSOMEONE;

  SigFox.begin();
  SigFox.reset();
  delay(100);
  SigFox.debug();
  Serial.print("SIGFOX ID= 0x");
  Serial.println(SigFox.ID());

  SPI.begin();
  rfid.PCD_Init();
}

typedef struct __attribute__ ((packed)) sigfox_message {
int16_t envTemp;
int16_t objTemp;
uint64_t nfcId;
} SigfoxMessage;

/* distance in milimeter
 *  
 */
uint16_t getDistance() {
#ifdef WITH_VL53L0X
  digitalWrite(DISTPIN,HIGH);
  vl53l0x.setTimeout(500);
  if ( !vl53l0x.init() ) {
    Serial.println("Error init VL53L0X");
  }
  vl53l0x.setMeasurementTimingBudget(100000);
  uint16_t d = vl53l0x.readRangeSingleMillimeters();
  digitalWrite(DISTPIN,LOW);
  return d;
#elif defined WITH_SHARPIR
  return ZSharpIR.distance();
#endif
}

void getBuz(int freq, int durMs) {
   pinMode(BUZZ_PIN, OUTPUT);
   tone(BUZZ_PIN,freq);
   delay(durMs);
   pinMode(BUZZ_PIN, INPUT); // because noTone seems to be ineficient
}


void loop() {
  int distance, dmin, dmax;
  int sleepTime = 800;
  float envTemp, objTemp;
  static uint64_t rfidId = 0;
  SigfoxMessage msg;

  // Read the card is present and update the corresponding variable
  if ( rfid.PICC_IsNewCardPresent() ) {
     if(rfid.PICC_ReadCardSerial()) {
        getBuz(2000,200);
        for (byte i = 0; i < rfid.uid.size; ++i) { 
          rfidId <<= 8;
          rfidId |= rfid.uid.uidByte[i];
        }
     }
  }
  
  switch(curentState) {
    case WAITFORSOMEONE:
      digitalWrite(DISTPIN,HIGH);   // switch distance sensor on  
      digitalWrite(REDLEDPIN,HIGH);
      delay(100);
      digitalWrite(REDLEDPIN,LOW);
      distance=getDistance();
      //Serial.print("Distance = "); Serial.println(distance);
      if ( distance < DIST_OUT ) {
        curentState = SOMEONEHERE;
        sleepTime = 0;
      } else {
        digitalWrite(DISTPIN,LOW);   // switch distance sensor off
        sleepTime = 800;
      }
    break;  

    case SOMEONEHERE:
      digitalWrite(REDLEDPIN,LOW);
      // wait for the right distance and stability
      dmin = 1000;
      dmax = 0;
      for ( int i = 0 ; i < DIST_LOOPS ; i++ ) {
        digitalWrite(GREENLEDPIN,HIGH);
        delay(50);
        digitalWrite(GREENLEDPIN,LOW);
        distance=getDistance();
        if ( distance < dmin ) dmin = distance;
        if ( distance > dmax ) dmax = distance;
        delay(50);
      }
      sleepTime = 0;
      Serial.print("dmin: ");Serial.print(dmin); Serial.print("dmax: ");Serial.print(dmax);Serial.println("");
      if ( dmin > DIST_MIN && dmax < DIST_MAX ) {
        curentState = GOODDISTANCE;
      } else if ( dmin > DIST_OUT ) {
        rfidId = 0;
        curentState = WAITFORSOMEONE;
      } else {
        if ( dmin < DIST_MIN ) {
           getBuz(500,300);
        }
        sleepTime = 50;
      }
      break;

    case GOODDISTANCE:
      // ensure stability
      getBuz(3000,100);
      for ( int i = 0 ; i < 2 ; i++ ) {
        digitalWrite(GREENLEDPIN,HIGH);
        delay(250);
        digitalWrite(GREENLEDPIN,LOW);        
        delay(250);
      }
      distance=getDistance();
      if ( distance > DIST_MIN && distance < DIST_MAX ) {
        curentState = DISTANCESTABLE;
      } else {
        curentState = SOMEONEHERE;
      }
      sleepTime = 100;
      break;

    case DISTANCESTABLE:
      // switch IR down to not pertubate measure
      digitalWrite(DISTPIN,LOW);
      delay(100);
      digitalWrite(GREENLEDPIN,LOW);
      // Get measure
      envTemp = mlx.readAmbientTempC();
      objTemp = 0;
      for (int i= 0; i<4 ; i++) {
        objTemp += mlx.readObjectTempC();
      }
      objTemp /= 4.0;
      // Verify distance
      digitalWrite(DISTPIN,HIGH);
      delay(100);
      distance=getDistance();
      Serial.print("Dist = ");Serial.print(distance);
      if ( distance > DIST_MIN && distance < DIST_MAX ) {
        // reading sounds valid
        Serial.print("mm Ambient = "); Serial.print(envTemp); 
        Serial.print("*C Object = "); Serial.print(objTemp); Serial.println("*C");
        
        getBuz(1000,200);
        delay(200);
        getBuz(1000,200);
        
        curentState = WAITFORLEAVING;
        digitalWrite(GREENLEDPIN,HIGH);
        digitalWrite(DISTPIN,LOW);
        // send the message
        #ifdef WITH_SIGFOX_TX
          msg.envTemp = (int16_t)(envTemp*100);
          msg.objTemp = (int16_t)(objTemp*100);
          msg.nfcId = rfidId;
          SigFox.beginPacket();
          SigFox.write((uint8_t*)&msg,sizeof(msg));
          SigFox.endPacket(false);
          sleepTime = 5000; // + about 7 seconds for Sigfox message
        #else
          sleepTime = 12000;
        #endif
      } else {
        curentState = SOMEONEHERE;
        sleepTime = 1000;
      }
      break;
      
    case WAITFORLEAVING:
      rfidId=0;
      digitalWrite(DISTPIN,HIGH);
      delay(100);
      distance=getDistance();
      digitalWrite(DISTPIN,LOW);
      if ( distance > DIST_OUT ) {
        digitalWrite(GREENLEDPIN,LOW);
        digitalWrite(REDLEDPIN,LOW);
        curentState = WAITFORSOMEONE;
        sleepTime = 0;
      } else {
        sleepTime = 1000;
      }
      break;
      
    default:
    break;
  }

  if ( sleepTime > 0 ) {
     delay(sleepTime);
  }

}
