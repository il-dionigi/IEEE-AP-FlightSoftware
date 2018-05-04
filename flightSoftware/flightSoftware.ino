int sensorValue_LEFT;  //A0
int sensorValue_RIGHT;  //A1
int sensorValue_MID;    //A2
 
int e_LEFT;
int e_RIGHT;
int throttleErrorP;
int throttleErrorI;
int throttleErrorD;
int throttleErrorTotal;
int rollErrorP;
int rollErrorI;
int rollErrorD;
int rollErrorTotal;
int pitchErrorP;
int pitchErrorI;
int pitchErrorD;
int pitchErrorTotal;
int yawErrorP;
int yawErrorI;
int yawErrorD;
int yawErrorTotal;

int errorP;   //position error
int prev_errorP = 0;  //previous position error
int errorD;   //change in error
int onTrack;  //middle sensor reading, if we're on track

// H denotes an H-bridge pin
int out_LEFTp; // D5 -> H2  controls direction of motor spin
int out_LEFTn; // D6 -> H7
 
int out_RIGHTp;  // D10 -> H15
int out_RIGHTn;  // D11 -> H10

int out_motorSpeed[4] = {0, 0, 0, 0};
 
int out_speedLEFT = 0; // D9 -> H1
int out_speedRIGHT = 0; // D3 -> H9

int sensorValue_MAG;
int LED = 13;

int out_topLeft = 0;
int out_topRight = 0;
int out_botLeft = 0;
int out_botRight = 0;

void copyIntArray(int *dst, int *src, int nb) { // faster than memcpy
  while(nb--) *dst++ = *src++;
}

void matrixAdd(int matrixTarget[], int matrixA[], int matrixB[]) { // fixed addition of size four
  int tmpMatrix[4] = {matrixA[0]+matrixB[0],matrixA[1]+matrixB[1],matrixA[2]+matrixB[2],matrixA[3]+matrixB[3]};
  copyIntArray(matrixTarget, tmpMatrix, 4);
}

void matrixMultiplyConst(int matrixTarget[], int matrixIn[], int k) {
  int tmpMatrix[4] = {matrixIn[0]*k,matrixIn[1]*k,matrixIn[2]*k,matrixIn[3]*k};
  copyIntArray(matrixTarget, tmpMatrix, 4);
}

void setup() {
pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);
pinMode(A3, INPUT);
// initialize serial communications at 9600 bps:
Serial.begin(9600);
}
 
void loop() {

errorP = e_RIGHT - e_LEFT;
errorD = errorP - prev_errorP;
prev_errorP = errorP;

//***************************************** NEW CODE ***********************************

// motorSpeed orientations are topLeft, topRight, botLeft, botRight
// positive error means to throttle up, pitch forward, roll right, and yaw right

// top left motor
out_motorSpeed[0] = (throttleErrorTotal * throttleAdj[0]) - (pitchErrorTotal * pitchAdj[0]) + (rollErrorTotal * rollAdj[0]) + (yawErrorTotal + yawAdj[0]);
// top right motor
out_motorSpeed[1] = (throttleErrorTotal * throttleAdj[1]) - (pitchErrorTotal * pitchAdj[1]) - (rollErrorTotal * rollAdj[1]) - (yawErrorTotal + yawAdj[1]);
// bottom left motor
out_motorSpeed[2] = (throttleErrorTotal * throttleAdj[2]) + (pitchErrorTotal * pitchAdj[2]) + (rollErrorTotal * rollAdj[2]) - (yawErrorTotal + yawAdj[2]);
// bottom right motor
out_motorSpeed[3] = (throttleErrorTotal * throttleAdj[3]) + (pitchErrorTotal * pitchAdj[3]) - (rollErrorTotal * rollAdj[3]) + (yawErrorTotal + yawAdj[3]);

delay(1);
}
