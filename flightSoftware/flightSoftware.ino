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

// create "TRPY vector" arrays that holds throttle, roll, pitch, and yaw
// utilize cartesian vectors / quaternions to also have positional error relative to copter's normal

//  ----  Want to deal with error mostly in TRPY form as this will be more easily translated to motor control and is
//        most likely the form user input will take.

// TRPY vector for current throttle, roll, pitch, yaw - "TRPY current vector"
//          calculated from quaternion pointing towards true up
//          the copters internal sense of its roll, pitch and yaw will always be relative to true up, not relative to its current position
//          throttle is a variable magnitude that scales the final motor output, after RPY error and correction has been calculated


// TRPY vector for error from desired orientation -- "TRPY error vector"
//          will usually be the roll, pitch and yaw of the "true up" quaternion relative to the quatd copter's normal
//                   relative vector as calculated with quaternions in our previous code, need to translate to TRPY
//                   throttle error will be calculated relative to acceleration in the opposite direction of "true up" when movement down is not from user input
//          when given user input the error TRPY vector will instead be the difference from current TRPY to user's desired input for TRPY
//                   simple, throttle will still be maintained to oppose acceleration down so as to stabilize the copter in the air
//          magnitude (severity) of TRPY error will be the spherical angle phi component of the positonal error vector
//                   the angle between desired normal and current normal (makes sense)
//          IMPLEMENTATION: 
//                   compute "TRPY desired vector" that is relative RPY to "true up" when no user input is given, otherwise in is equal to "TRPY user input"
//                              the T throttle component is never purely user input, it always has part that resists gravity unless user input purposefully decreases throttle
//                   "TRPY error vector" will be difference from "TRPY current vector" to "TRPY desired vector"


// PROPORTIONAL error will be "TRPY error vector" scaled by a constant "CP"
// DIFFERENTIAL error will be ("TRPY error vector" - "TRPY error vector previous") scaled by a constant "CD"
// INTEGRAL error will be N point sum of past TRPY error vectors scaled by a constant "CI"
// SUM above to form "TRPY output vector"

// GEN METHOD:   Calculate "TRPY error vector" and final output to motors is taken from "TRPY output vector", with each motor taking certain elements
//                      certain motors will be responding to pitch error differently
//               Scaled all final output to motors by throttle T component of "TRPY output vector"


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
