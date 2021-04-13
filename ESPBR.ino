#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define enA 17
#define in1 18
#define in2 19
#define in3 21
#define in4 22
#define enB 23

const int freq = 500;
const int channel1 = 0;
const int channel2 = 1;
const int resolution = 10;

int speedRobot  = 700;

char cmnd = ' ' ;

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESPBR"); //Bluetooth device name
  Serial.println("ESPBR started, now you can pair it with bluetooth!");

  pinMode(enA,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

  ledcSetup(  channel1 , freq , resolution );
  ledcSetup(  channel2 , freq , resolution );
  ledcAttachPin( enA , channel1 );
  ledcAttachPin( enB , channel2 );
} 

void loop() {
  
  if( SerialBT.available())
  { cmnd = (char)SerialBT.read();
    delay(10);
  }


  if( cmnd == 'U') forward();
  else if( cmnd == 'D') backward();
  else if( cmnd == 'L') left();
  else if( cmnd == 'R') right();
  else if( cmnd == 'S') stopRobot();
  /*else if( cmnd == 'X') specialOne();
  else if( cmnd == 'Y') specialTwo();
  else if( cmnd == 'Z') specialThree();*/
  else if( cmnd == 'a') speedRobot = 400;
  else if( cmnd == 'b') speedRobot = 460;
  else if( cmnd == 'c') speedRobot = 520;
  else if( cmnd == 'd') speedRobot = 580;
  else if( cmnd == 'e') speedRobot = 640;
  else if( cmnd == 'f') speedRobot = 700;
  else if( cmnd == 'g') speedRobot = 760;
  else if( cmnd == 'h') speedRobot = 820;
  else if( cmnd == 'i') speedRobot = 880;
  else if( cmnd == 'j') speedRobot = 940;
  else if( cmnd == 'k') speedRobot = 1023;
  
  else stopRobot();
  cmnd = ' ';
  delay(75);
}

void forward()
{ digitalWrite(in1 , LOW);
  digitalWrite(in2 , HIGH);
  ledcWrite(channel1 , speedRobot);

  digitalWrite(in3 , LOW);
  digitalWrite(in4 , HIGH);
  ledcWrite(channel2 , speedRobot);

  Serial.println("Forward");

}

void backward()
{ digitalWrite(in1 , HIGH);
  digitalWrite(in2 , LOW);
  ledcWrite(channel1 , speedRobot);

  digitalWrite(in3 , HIGH);
  digitalWrite(in4 , LOW);
  ledcWrite(channel2 , speedRobot);

  Serial.println("Backward");
}

void left()
{ digitalWrite(in1 , HIGH);
  digitalWrite(in2 , LOW);
  ledcWrite(channel1 , speedRobot);

  digitalWrite(in3 , LOW);
  digitalWrite(in4 , HIGH);
  ledcWrite(channel2 , speedRobot);

  Serial.println("Left");
  
}

void right()
{ digitalWrite(in1 , LOW);
  digitalWrite(in2 , HIGH);
  ledcWrite(channel1 , speedRobot);

  digitalWrite(in3 , HIGH);
  digitalWrite(in4 , LOW);
  ledcWrite(channel2 , speedRobot);

  Serial.println("Right");
}

void stopRobot()
{ digitalWrite(in1 , LOW);
  digitalWrite(in2 , LOW);
  
  digitalWrite(in3 , LOW);
  digitalWrite(in4 , LOW);

  Serial.println("Stopped");
}
