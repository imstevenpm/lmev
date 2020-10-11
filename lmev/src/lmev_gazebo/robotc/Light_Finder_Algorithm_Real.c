#pragma config(Sensor, S1,     touchSensor,    sensorEV3_Touch)
#pragma config(Sensor, S2,     gyroSensor,     sensorEV3_Gyro, modeEV3Gyro_RateAndAngle)
#pragma config(Sensor, S3,     colorSensor,    sensorEV3_Color, modeEV3Color_Ambient)
#pragma config(Sensor, S4,     sonarSensor,    sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          armMotor,      tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorB,          leftMotor,     tmotorEV3_Large, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          rightMotor,    tmotorEV3_Large, PIDControl, driveRight, encoder)

/*++++++++++++++++++++++++++++++*/
/*++++++++++++ VARS ++++++++++++*/
/*++++++++++++++++++++++++++++++*/

int done = 0;

/*Task Concurrency Semaphores*/
int walling = 0;
int lighting = 0;
int backing = 0;

/*Threshold variables*/
int close_wall_thold = 40;
int far_wall_thold = 60;
int impact_thold = 20;
int min_light_thold = 20;
int goal_light_thold = 40;

/*Other thresholds*/
int turning_speed = 25;
int turning_back = 25;
int cruise_speed = 30;
int cruise_back = 40;
/*++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++*/

/*##############################*/
/*############ FUNS ############*/
/*##############################*/

/*Interruption mode:
	0 -> collision_detected
	1 -> wall_deviation
	2 -> light_deviation*/
void interrupt(int mode){
	switch(mode){
		case 0:{
			backing = 1;
			break;
		}
		case 1:{
			walling = 1;
			break;
		}
		case 2:{
			lighting = 1;
			break;
		}
	}
}

int can_enter(int mode){
	int can = 0;
	switch(mode){
		case 0:{
			can = !backing && !walling && !lighting;
			break;
		}
		case 1:{
			can = !backing && !lighting;
			break;
		}
		case 2:{
			can = !backing;
			break;
		}
	}
	return can;
}

void recover(int mode){
	switch(mode){
		case 0:{
			backing = 0;
			break;
		}
		case 1:{
			walling = 0;
			break;
		}
		case 2:{
			lighting = 0;
			break;
		}
	}
}

void reset_encoders(){
	nMotorEncoder[rightMotor] = 0;
	nMotorEncoder[leftMotor] = 0;
}

void wait_finish(){
	waitUntilMotorStop(motorB);
	waitUntilMotorStop(motorC);
}

int get(char sensor){
	int measured = -666;
	switch (sensor){
		case 'l':{
			measured = SensorValue(colorSensor);
			break;}
		case 's':{
			measured = SensorValue(sonarSensor);
			break;}
		case 't':{
			measured = SensorValue(touchSensor);
			break;}
		case 'g':{
			measured = SensorValue(gyroSensor);
			break;}
	}
	return measured;
}

void pass(int t){
	wait1Msec(t);
}

void turn(char mode, int rots){
	reset_encoders();
	switch (mode){
		case 'l':{
				setMotorTarget(motorC, 0, 0);
				setMotorTarget(motorB, rots, turning_speed);
				break;}
		case 'r':{
				setMotorTarget(motorB, 0, 0);
				setMotorTarget(motorC, rots, turning_speed);
				break;}
	}
	wait_finish();
	pass(100);

}

int wall_distance(){
	turn('r', 200);
	wait1Msec(200);
	int distance = get('s');
	turn('r', -200);
	wait1Msec(200);
	return distance;
}



task exit{
	while(true){
		if (get('s') < impact_thold || get('t')){
			interrupt(0);
			reset_encoders();
			setMotorTarget(motorB, -750, cruise_back);
			setMotorTarget(motorC, -750, cruise_back);
			wait_finish();
			turn('r', -100);
			recover(0);
			pass(500);
		}
	}
}

/*Light triangulation (returns dir that increments light value):
	f = front light;
	l = left light
	r = right light
*/
char light_triang(){
	int light = min_light_thold;
	char dir = 'f';
	int aux = get('l');
	if(aux > goal_light_thold){
			playSound(soundFastUpwardTones);
			pass(3000);
			stopAllTasks();
		}
	else{
		turn('r', 100);
		aux = get('l');
		if(aux > light){
			if(aux > goal_light_thold){
				playSound(soundFastUpwardTones);
				pass(3000);
				stopAllTasks();
				}
			else{
				light = aux;
				dir = 'r';
				}
		}
		turn('r', -100);

		turn('l', 100);

		aux = get('l');
		if (aux > light){
			if(aux > goal_light_thold){
				playSound(soundFastUpwardTones);
				pass(3000);
				stopAllTasks();
				}
			else{
				dir = 'l';
				}
		}
		turn('l', -100);
	}
	return dir;
}

/*##############################*/
/*##############################*/
/*##############################*/

/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/*$$$$$$$$$$ TASKS $$$$$$$$$$$$$*/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/


task follow_wall{
	while(true){
		if(can_enter(1)){
			interrupt(1);
			int distance = wall_distance();
			if (can_enter(1))
				if (distance < close_wall_thold){
					playSound(soundBeepBeep);
					turn('l', 100);
				}
				else if (distance > far_wall_thold){
					playSound(soundBlip);
					turn('r', 100);
				}
			recover(1);
			pass(2000);
		}
	}
}

task cruise{
	while(true){
		if(can_enter(0)){
			setMotorSpeed(motorB, cruise_speed);
			setMotorSpeed(motorC, cruise_speed);
			pass(1500);
		}
	}
}

task search{
	char dir = 'n';
	while(get('l') < goal_light_thold){
		if(can_enter(2) && get('l') > min_light_thold){
			playSound(soundException);
			interrupt(2);
			min_light_thold=0;
			if(!done){
				while(walling)
					sleep(1);
				stopTask(follow_wall);
				stopTask(cruise);
				done = 1;
			}
			dir = light_triang();
			switch (dir){
				case 'l':{
					turn('l', 100);
					break;}
				case 'r':{
					turn('r', 100);
					break;}
				}
			reset_encoders();
			setMotorTarget(motorB, 180, cruise_speed);
			setMotorTarget(motorC, 180, cruise_speed);
			wait_finish();
			recover(2);
			pass(1000);
		}
	}
	playSound(soundFastUpwardTones);
	pass(3000);
	stopAllTasks();
}

/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/

task main()
{
	setMotorSpeed(armMotor, 50);

	startTask(exit);
	startTask(search);
	startTask(follow_wall);
	startTask(cruise);

	while(true){
		wait1Msec(1000);
	}
}
