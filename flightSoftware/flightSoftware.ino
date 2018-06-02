#include <sensor_fusion.h>

//registers for high and low bytes of
//accelerometer and gyroscope reading
#define ahx 59
#define alx 60
#define ahy 61
#define aly 62
#define ahz 63
#define alz 64

#define ghx 67
#define glx 68
#define ghy 69
#define gly 70
#define ghz 71
#define glz 72

struct vector accelBias;
struct vector gyroBias;

//normalized vectors pointing towards relative true-up
//use normalized readings from accelerometer, no need for
//separate vector
struct vector trueUpGyro;
struct vector trueUpFusion;

unsigned long timePrev = 0;
unsigned long timeCurr = 0;

//constant to control weight of accelerometer
//and gyroscope contributing to fusionNormal
float alpha = 0.25;


float throttleErrorP;
float throttleErrorI;
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

float trpy_current[4] = {0, 0, 0, 0};
float trpy_current_D[4] = {0, 0, 0, 0};
float trpy_current_D_D[4] = {0, 0, 0, 0};

float trpy_desired[4] = {0, 0, 0, 0};


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

void computeBiasComp(int lowBit, int highBit, float* dest) {
  long sum = 0;
  uint8_t readBuf;
  unsigned short addMe;
  
  int i = 0;
  for (;i < 75; i++) {
    readReg(highBit, &readBuf, 1);
    addMe = ((unsigned short)readBuf) << 8;
    readReg(lowBit, &readBuf, 1);
    addMe += (unsigned short)readBuf;
    sum += (short)(addMe);
  }
  *dest = (float)(-sum / i);
}

//returns normalized vector of acclerometer readings and a vector
//of gyroscope readings with unchanged magnitude
void getVectors(struct vector *accelVector, struct vector *gyroVector) {
  uint8_t readBuf;
  
  //2byte 2-s compliment integers stored in registers
  short vals[3];
  unsigned short val;
  
  for (int i = 0; i < 3; i++) {
    readReg(ahx + 2*i, &readBuf, 1);
    val = ((unsigned short)readBuf) << 8;
    readReg(alx + 2*i, &readBuf, 1);
    val += (short)readBuf;
    vals[i] = val;
  }

  accelVector->x = (float)vals[0];
  accelVector->y = (float)vals[1];
  accelVector->z = (float)vals[2];

  vector_add(accelVector, &accelBias, accelVector);
  vector_normalize(accelVector, accelVector);
  
  for (int i = 0; i < 3; i++) {
    readReg(ghx + 2*i, &readBuf, 1);
    val = ((unsigned short)readBuf) << 8;
    readReg(glx + 2*i, &readBuf, 1);
    val += (short)readBuf;
    vals[i] = val;
  }

  gyroVector->x = (float)vals[0];
  gyroVector->y = (float)vals[1];
  gyroVector->z = (float)vals[2];
  
  vector_add(gyroVector, &gyroBias, gyroVector);
}

void setup() {
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
	// initialize serial communications at 9600 bps:

	Serial.begin(115200);
	Serial.println("OPENED SERIAL");

	Wire.begin();
	Wire.setClock(100000);
	config();

	computeBiasComp(alx, ahx, &accelBias.x);
	computeBiasComp(aly, ahy, &accelBias.y);
	computeBiasComp(alz, ahz, &accelBias.z);

	computeBiasComp(glx, ghx, &gyroBias.x);
	computeBiasComp(gly, ghy, &gyroBias.y);
	computeBiasComp(glz, ghz, &gyroBias.z);

	accelBias.z += 2048; //~gravity

	trueUpGyro.x = 0;
	trueUpGyro.y = 0;
	trueUpGyro.z = 1;

	trueUpFusion.x = 0;
	trueUpFusion.y = 0;
	trueUpFusion.z = 1;
}
 
void loop() {

	errorP = e_RIGHT - e_LEFT;
	errorD = errorP - prev_errorP;
	prev_errorP = errorP;

	//***************************************** NEW CODE ***********************************

	// motorSpeed orientations are topLeft, topRight, botLeft, botRight
	// positive error means to throttle up, pitch forward, roll right, and yaw right

	timePrev = timeCurr;
	timeCurr = millis();
	unsigned long stepTime = (timeCurr - timePrev);
	
	//vectors of sensor readings
	struct vector accelVector;
	struct vector gyroVector;
	getVectors(&accelVector, &gyroVector);
	
	float angularVelocity = vector_normalize(gyroVector, gyroVector);
	float gyroAngle = angularVelocity * stepTime * PI / (180*16.4)/1000;  
	
	struct quaternion rotateQuat;
	//use a quaternion to apply a rotational transformation 
	quaternion_create(&gyroVector, -gAngle, &rotateQuat);
	quaternion_rotate(&trueUpGyro, &rotateQuat, &trueUpGyro);
	vector_normalize(&trueUpGyro, &trueUpGyro);

	quaternion_rotate(&trueUpFusion, &rotateQuat, &trueUpFusion);
	vector_multiply(&trueUpFusion, 1.0-alpha, &trueUpFusion);
	vector_multiply(&accelVector, alpha, &accelVector);
	vector_add(&accelVector, &trueUpFusion, &trueUpFusion);
	vector_normalize(&trueUpFusion, &trueUpFusion);

	// create "TRPY vector" arrays that holds throttle, roll, pitch, and yaw
	tryp_current = [accelVector.z, vector_roll(trueUpFusion), vecotr_pitch(trueUpFusion), gyroAngle];


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