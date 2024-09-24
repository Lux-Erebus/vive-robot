#include "vive510.h"

#define SIGNALPIN1 32
#define SIGNALPIN2 33
Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);
void setup() {
Serial.begin(115200);
vive1.begin();
vive2.begin();
delay(3000);
Serial.println("Vive tracker Started");
}

void loop() {
Serial.println("Loop entered");

if (vive1.status() == VIVE_LOCKEDON) {
Serial.printf("X1 %d, Y1 %d\n",vive1.xCoord(),  vive1.yCoord());
}
else
vive1.sync(15);   // try to resync 15 times (nonblocking); */

if (vive2.status() == VIVE_LOCKEDON) {
Serial.printf("X2 %d, Y2 %d\n",vive2.xCoord(),  vive2.yCoord());
}
else
vive2.sync(15);   // try to resync 15 times (nonblocking); 
delay(1000);

}
