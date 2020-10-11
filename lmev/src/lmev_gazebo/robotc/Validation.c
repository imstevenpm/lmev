#pragma config(Sensor, S1,     touchSensor,    sensorEV3_Touch)
#pragma config(Sensor, S2,     gyroSensor,     sensorEV3_Gyro, modeEV3Gyro_Angle)
#pragma config(Sensor, S3,     colorSensor,    sensorEV3_Color, modeEV3Color_Ambient)
#pragma config(Sensor, S4,     sonarSensor,    sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          armMotor,      tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorB,          leftMotor,     tmotorEV3_Large, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          rightMotor,    tmotorEV3_Large, PIDControl, driveRight, encoder)

/* Init Variables */
int Thread = 1;
int MThresholds = 40;
int Gyroscope_Data =0;
int Color_Data=0;
float Ultrasonic_Data=0;

/* Functions */
int Tank_Rev(int Target_Data){
	setMotorTarget(motorB, Target_Data*360, 50);
	setMotorTarget(motorC, Target_Data*360, 50);
	waitUntilMotorStop(motorB);
	waitUntilMotorStop(motorC);
	return 0;
}

int Turn_Gyro(int Target_Data){
	Gyroscope_Data = getGyroDegrees(S2);
	while (Gyroscope_Data<Target_Data){
		setMotorSpeed(motorB,50);
		setMotorSpeed(motorC,-50);
		Gyroscope_Data = getGyroDegrees(S2);
	}
	setMotorSpeed(motorB,0);
	setMotorSpeed(motorC,0);
	return 0;
}

int begin_validation(){
	while (Thread==1){
		Color_Data= SensorValue(colorSensor);
		if (Color_Data>MThresholds){
			Thread=0;
			Tank_Rev(10);
			Turn_Gyro(90);

			Ultrasonic_Data=SensorValue(sonarSensor);
			if (Ultrasonic_Data<50.0){
				setMotorSpeed(motorA,-50);
				}
			}
		}
	return 0;
}

task main()
{
	begin_validation();

	while(true){
		wait1Msec(1000);
	}
}
