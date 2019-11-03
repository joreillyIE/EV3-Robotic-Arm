//Programmed by Joanne Reilly, Maggie Xu & Will Knock

#pragma config(Sensor, S1,     light,          sensorEV3_Color)
#pragma config(Sensor, S2,     touch,          sensorEV3_Touch)
#pragma config(Motor,  motorA,          grip,          tmotorEV3_Medium, PIDControl, encoder)
#pragma config(Motor,  motorB,          lift,          tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorC,          rotation,      tmotorEV3_Large, PIDControl, encoder)

// Motor values for different arm operations ||
// (Calculated through trial & error)        ||
//                                           ||
// Claw Open = -70                           ||
// Claw Closed = -30                         ||
//                                           ||
// Arm Down (Motor) >= 120                   ||
// Arm Up (Light) >= 65                      ||
//|||||||||||||||||||||||||||||||||||||||||||||


// (x0,y0)=(1,0) -> theta=0 degrees
// When the touch sensor is activated, thetaAbsolute=0;
// The arm only operates in Quadrants I & II of the Unit Circle
float thetaObject = 0.0;
float thetaDestination = 0.0;

float theta = 0.0;    // global var for holding angle values in degrees
float rads = 0.0;     // global var for holding angle values in radians
float sinTheta = 0.0; // global vars for sin()
float cosTheta = 0.0; // & cos()

float rotationVal = 0.0; // var for target rotational motor encoder value for next action
float rotationPos = 0.0; // var for current rotational motor encoder value

// placeholder vars used in Transformation Matrix calculations
float newX = 0.0;
float newY = 0.0;

// 1 -> positive theta (counter-clockwise), -1 -> negative theta (clockwise)
int rotateDirection = 1;

// Error for calculating positions
float epsilon = 0.01;

// Start/End position; Arm intially calibrates to theta=0
// THESE DO NOT CHANGE
float startX = 1.0;
float startY = 0.0;

// Simply Change the object and destination coordinates,   ||
// The program will take care of everything else!!         ||
// NOTE: (x,y) must be in Quadrants I or II                ||
//       of Cartesian plane                                ||
//                                                         ||
// Object position --                                      ||
// Change according to where the object is placed          ||
float objectX = -1.0;                                    //||
float objectY = 0.0;                                     //||
//                                                         ||
// Destination position                                    ||
// Change according to where the object should be placed   ||
float destinationX = 1.0;                                //||
float destinationY = 0.0;                                //||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

// Array used to hold oosition (x,y) coordinates
float pos[2] = {0.0,0.0};

// 2D array for Cartesian (x,y) transformation matrix
float transformationMatrix [2][2] = { {0.0,0.0},{0.0,0.0} };

// Ratio for degrees:rotational-motor-value in xy Cartesian plane
// NOTE: negative value because a negative motor value
//       is associated with a positive theta
float rotRatio = -3.42; // Was calculated (tediously) through Trial & Error

// Lifts arm from dead to a maintainable height
void liftArm() {
	while (SensorValue[light] < 65){
		motor[lift] = -10	;
	}
}

// Maintains elevated arm height
void maintainHeight() {
	if (SensorValue[light] > 72) {
		motor[lift] = 2;
	}
	if (SensorValue[light] < 68) {
		motor[lift] = -2;
	}
}

// Calibrate the arm & grip to start position (startX, startY) -> theta=0
void calibrate (){

	// Lift arm initially to avoid object collision
	liftArm();

	// Calibrate grip
	clearTimer(T1);
	while(time1[T1] < 500) {
		motor[grip] = 15;
	}
	resetMotorEncoder(grip);

	// Calibrate rotation
	while (SensorValue[touch] == 0){
		maintainHeight();
		motor[rotation] = 10;
	}

	// Zero out rotational motor value
	resetMotorEncoder(rotation);
}

// Quick calculation to determine movement direction
// from object to destination
void calculateRotDir() {
	if (rotationPos > rotationVal) {
		rotateDirection = -1;
	} else {
		rotateDirection = 1;
	}
}

// Multiply a Position Matrix by a Rotation Matrix (in-place)
void rTimesM(float *position, float *rotationM) {
	newX = ( (rotationM[0]*position[0]) +
	         (rotationM[1]*position[1]) );
	newY = ( (rotationM[2]*position[0]) +
	         (rotationM[3]*position[1]) );
	position[0] = newX;
	position[1] = newY;
}

// Rotate the arm to hover over the object (objectX,objectY)
void rotateToObject() {

	// Calculate theta between starting position and object position
	rads = acos(objectX);
	thetaObject = (rads / PI) * 180;
	// NOTE: b/c we know the arm only operates in Quadrants I and II
	//       of the Cartesian plane, acos(x) is unique

	// Amount of rotation necessary
	rotationVal = thetaObject * rotRatio;

	// Rotate arm to hover over object position
	rotationPos = getMotorEncoder(rotation);
	while (rotationPos >= rotationVal) {
		maintainHeight();
		motor[rotation] = -10;
		rotationPos = getMotorEncoder(rotation);
	}

	// Kill motor
	motor[rotation] = 0;

	// Calculate a Transition Matrix
	sinTheta = sin(rads);
	cosTheta = cos(rads);
	transformationMatrix[0][0] = cosTheta;
	transformationMatrix[0][1] = -1.0 * sinTheta;
	transformationMatrix[1][0] = sinTheta;
	transformationMatrix[1][1] = cosTheta;

	// Calculate position matrix based on arm transition
	rTimesM(pos, transformationMatrix);
}

// Opens the claw
void openClaw() {
	while (getMotorEncoder(grip) > -70) {
		motor[grip] = -15;
	}
	motor[grip] = 0;
}

// Lowers to arm to height of the surface
void lowerArm() {
	while (getMotorEncoder(lift) < 120) {
		motor[lift] = 10;
	}
	motor[lift] = 0;
}

// Closes the claw
void closeClaw() {
	while (getMotorEncoder(grip) < -30) {
		motor[grip] = 15;
	}
}

// Applies a small motor force to keep object in claw
void maintainClosedClaw() {
	motor[grip] = 5;
}

// Pick up object (assumes arm is currently hovering over object
void pickUpObject() {
	openClaw();
	lowerArm();
	closeClaw();
	liftArm();
}

// Holds object upright in use for transition
void holdObject() {
	maintainHeight();
	maintainClosedClaw();
}

// Sets down object on the floor
void setDownObject() {
	lowerArm();
	openClaw();
	liftArm();
}

// Rotate the arm (holding an object) to the destination (destinationX,destinationY)
void rotateToDestination() {
	// Calculate theta between starting position and object position
	rads = acos(destinationX);
	thetaDestination = (rads / PI) * 180;

	// Final rotation value
	rotationVal = thetaDestination * rotRatio;

	//Transforming relative to last position
	theta = thetaDestination - thetaObject;
	rads = (theta / 180) * PI;

	// Rotate arm to hover over object position
	rotationPos = getMotorEncoder(rotation);

	calculateRotDir();

	if (rotateDirection == -1) {
		while (rotationPos >= rotationVal) {
			holdObject();
			motor[rotation] = 10 * rotateDirection;
			rotationPos = getMotorEncoder(rotation);
		}
	} else {
		while (rotationPos <= rotationVal) {
			holdObject();
			motor[rotation] = 10 * rotateDirection;
			rotationPos = getMotorEncoder(rotation);
		}
	}

	// Kill motor
	motor[rotation] = 0;

	// Calculate a Transition Matrix
	sinTheta = sin(rads);
	cosTheta = cos(rads);
	transformationMatrix[0][0] = cosTheta;
	transformationMatrix[0][1] = -1.0 * sinTheta;
	transformationMatrix[1][0] = sinTheta;
	transformationMatrix[1][1] = cosTheta;

	// Calculate position matrix based on arm transition
	rTimesM(pos, transformationMatrix);
}

task main()
{

	calibrate();

	// Now pos = (1,0)
	pos[0] = startX;
	pos[1] = startY;

	// Converge arm to (objectX, objectY)
	while (abs(pos[0] - objectX) > epsilon ||
		     abs(pos[1] - objectY) > epsilon) {
		rotateToObject();
	}

	// Smoothly pick up object
	pickUpObject();
	clearTimer(T1);
	while(time1[T1] < 1000) {
		maintainHeight();
	}

	// Converge arm to (destinationX, destinationY)
	while (abs(pos[0] - destinationX) > epsilon ||
		     abs(pos[1] - destinationY) > epsilon) {
		rotateToDestination();
	}

	// Smoothly set down object
	clearTimer(T1);
	while(time1[T1] < 1000) {
		maintainHeight();
	}
	setDownObject();

	// Reset to original position and exit gracefully
	calibrate();
	motor[rotation] = 0;
	clearTimer(T1);
	while(time1[T1] < 2000) {
		maintainHeight();
	}
}
