#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <ZSharpIR.h>
#include <ArduinoLowPower.h>
#include <SigFox.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
ZSharpIR ZSharpIR(A1, 20150);

#define DISTPIN 2
#define REDLEDPIN 5
#define GREENLEDPIN 6

#define DIST_MIN 290
#define DIST_MAX 350
#define DIST_OUT 450

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
  
  // Setup
  Serial.begin(9600);
  mlx.begin();
  ZSharpIR.setARefVoltage(3300);
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
}

typedef struct __attribute__ ((packed)) sigfox_message {
int16_t envTemp;
int16_t objTemp;
uint64_t nfcId;
} SigfoxMessage;

void loop() {
  int distance, dmin, dmax;
  int sleepTime = 800;
  float envTemp, objTemp;
  SigfoxMessage msg;
  
  switch(curentState) {
    case WAITFORSOMEONE:
      digitalWrite(DISTPIN,HIGH);   // switch distance sensor on  
      digitalWrite(REDLEDPIN,HIGH);
      delay(100);
      digitalWrite(REDLEDPIN,LOW);
      distance=ZSharpIR.distance();
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
      for ( int i = 0 ; i < 10 ; i++ ) {
        digitalWrite(GREENLEDPIN,HIGH);
        delay(50);
        digitalWrite(GREENLEDPIN,LOW);
        distance=ZSharpIR.distance();
        if ( distance < dmin ) dmin = distance;
        if ( distance > dmax ) dmax = distance;
        delay(50);
      }
      sleepTime = 0;
      //Serial.print("dmin: ");Serial.print(dmin); Serial.print("dmax: ");Serial.print(dmax);Serial.println("");
      if ( dmin > DIST_MIN && dmax < DIST_MAX ) {
        curentState = GOODDISTANCE;
      } else if ( dmin > DIST_OUT ) {
        curentState = WAITFORSOMEONE;
      } else {
        sleepTime = 50;
      }
      break;

    case GOODDISTANCE:
      // ensure stability
      for ( int i = 0 ; i < 2 ; i++ ) {
        digitalWrite(GREENLEDPIN,HIGH);
        delay(250);
        digitalWrite(GREENLEDPIN,LOW);        
        delay(250);
      }
      distance=ZSharpIR.distance();
      if ( distance > DIST_MIN && distance < DIST_MAX ) {
        curentState = DISTANCESTABLE;
      } else {
        curentState = SOMEONEHERE;
      }
      sleepTime = 100;
      break;

    case DISTANCESTABLE:
      digitalWrite(GREENLEDPIN,LOW);
      envTemp = mlx.readAmbientTempC();
      objTemp = mlx.readObjectTempC();
      distance=ZSharpIR.distance();
      if ( distance > DIST_MIN && distance < DIST_MAX ) {
        // reading sounds valid
        Serial.print("Ambient = "); Serial.print(envTemp); 
        Serial.print("*C Object = "); Serial.print(objTemp); Serial.println("*C");
        curentState = WAITFORLEAVING;
        digitalWrite(GREENLEDPIN,HIGH);
        sleepTime = 3000; // + about 7 seconds for Sigfox message
        digitalWrite(DISTPIN,LOW);
        // send the message
        msg.envTemp = (int16_t)(envTemp*100);
        msg.objTemp = (int16_t)(objTemp*100);
        msg.nfcId = 0;
        SigFox.beginPacket();
        SigFox.write((uint8_t*)&msg,sizeof(msg));
        SigFox.endPacket(false);
        digitalWrite(GREENLEDPIN,HIGH);
      } else {
        curentState = SOMEONEHERE;
        sleepTime = 1000;
      }
      break;
      
    case WAITFORLEAVING:
      digitalWrite(DISTPIN,HIGH);
      delay(100);
      distance=ZSharpIR.distance();
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
     //LowPower.sleep(sleepTime); -- still buggy unfortunatelly
  }

}
