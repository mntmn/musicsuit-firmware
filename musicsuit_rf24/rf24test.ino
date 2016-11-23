#include <SPI.h>
#include "RF24.h"
#include <string.h>

#define CE_PIN 7
#define CSN_PIN 8
//#define CE_PIN 9
//#define CSN_PIN 10

bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(CE_PIN,CSN_PIN);

byte addresses[][6] = {"1Node","2Node"};

char inbuf[32];

void setup() {
  Serial.begin(115200);
  //Serial1.begin(31250);
  //Serial1.println(F("RF24"));
  
  radio.begin();

  radio.setPALevel(RF24_PA_HIGH);

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  
  radio.startListening();
}

int tick = 0;

char sbuf[64];

void noteOn(int cmd, int pitch, int velocity) {
  //Serial1.write(cmd);
  //Serial1.write(pitch);
  //Serial1.write(velocity);
  
  sprintf(sbuf,"%d,%d,%d\r\n",cmd,pitch,velocity);
  Serial.write(sbuf);
}

int lastv1 = 0;
int lastv2 = 0;
int lastv3 = 0;
int lastv4 = 0;

void controlChange(int channel, int controller, int value) {
  if (lastv1==value && controller==1) return;
  if (lastv2==value && controller==2) return;
  if (lastv3==value && controller==3) return;
  if (lastv4==value && controller==4) return;

  //Serial1.write(0xb0+channel);
  //Serial1.write(controller);
  //Serial1.write(value);

  if (controller==1) lastv1=value;
  if (controller==2) lastv2=value;
  if (controller==3) lastv3=value;
  if (controller==4) lastv4=value;
  
  sprintf(sbuf,"%d,%d,%d\r\n",0xb0+channel,controller,value);
  Serial.write(sbuf);
}

int cc=0;
int lx,ly,lz,dx,dy,dz,ystate=0,ynote,ynoteidx=0,stcount=0;
int noteseq[4]={0x30,0x3b,0x33,0x35};
int dy1,dy2,dy3,dy4;
int s2note=0;
#define noteseqlen 2

void loop() {
  int x,y,z,sensor;
  
  memset(inbuf,0,32);
  radio.read(inbuf, 32);
  tick++;
  
  Serial.print(inbuf);

  /*
    x range: ~ -17000..17000
    y range: ~ -9000(bottom)..9000(top)
  */

  // left leg: 2
  // right leg: 0 yes
  // left arm: 3
  // right arm: 1 yes
  
  /*if (inbuf[0]!=0) {
    sscanf(inbuf, "%d\t%d\t%d\t%d", &sensor, &x,&y,&z);
    
    if (sensor==0) {
      dy=abs(y-ly);
      dy1=dy;
      dy2=dy1;
      dy3=dy2;
      dy4=dy3;

      int avg2=((dy+dy1)/2)/50;
      int avg4=((dy1+dy2+dy3+dy)/4)/50;
      
      ly=y;

      //Serial.print(dy);
      //Serial.print("\r\n");
      
      if (ystate==0) {

        if (dy>100) {
          stcount++;
        } else {
          stcount=0;
        }
        
        if (stcount>4) {
          ystate=1;
          stcount=0;
          ynote=s2note; //noteseq[ynoteidx]; //+((x/10)%3);
          //ynoteidx%=noteseqlen;
          noteOn(0x90, ynote, 0x7f);
          //Serial.print("note on\r\n");
        }
      } else if (ystate==1) {
        //Serial.println(avg4);
        if (dy<100) {
          stcount++;
        } else {
          stcount=0;
        }

        // note off
        if (stcount>4) {
          ystate=0;
          stcount=0;
          noteOn(0x90, ynote, 0x00);
          //Serial.print("note off\r\n");
        }
      }
      
      controlChange(0, 1, 64+y/100);
    } else if (sensor==1) {
      //Serial.println(x/100);
      //controlChange(0, 1, 64+x/100);
      //controlChange(0, 1, 64+y/100);
      controlChange(0, 87, 127-(64+y/100));
      // right arm
      s2note=0x30+(y/1000);
    } else if (sensor==2) {
      controlChange(0, 3, 64+y/100);
    } else if (sensor==3) {
      controlChange(0, 4, 64+y/100);
    }
    
    //Serial.print(inbuf);
  }*/
}
