#include <Arduino.h>
#include <Servo.h>
Servo servox;
int posx=90;
int tarx=90;
Servo servoy;
int posy=90;
int tary=90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  servox.attach(10);
  servoy.attach(9);
}
int trr=1;
long posx_update_millis=0;
long posy_update_millis=0;
void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    String msg = Serial.readStringUntil('\r');
    if(msg[0]=='x')tarx=constrain(msg.substring(2,msg.length()).toInt(),0,180);
    if(msg[0]=='y')tary=constrain(msg.substring(2,msg.length()).toInt(),0,180);

    Serial.print(posx);
    Serial.print(',');
    Serial.println(posy);
  }
  if(millis()-posx_update_millis>10){
    posx_update_millis=millis();
    if(abs(tarx-posx)>trr){
      int tarposx=(tarx-posx<0 ? -1 : 1)*trr;
      posx+=tarposx;
    }else posx=tarx;
  }
  if(millis()-posy_update_millis>50){
    posy_update_millis=millis();
    if(abs(tary-posy)>trr){
        int tarposy=(tary-posy<0 ? -1 : 1)*trr;
        posy+=tarposy;
      }else posy=tary;
  }
  servox.write(posx);
  servoy.write(posy);
}