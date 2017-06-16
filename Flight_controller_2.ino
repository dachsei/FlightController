git status
#include <Wire.h>
#include <EEPROM.h>

#define USEC 2;

float pid_p_gain_roll = 1.2;               //Gain setting for the roll P-controller //1.2
float pid_i_gain_roll = 0.02;              //Gain setting for the roll I-controller //0.04
float pid_d_gain_roll = 16.0;              //Gain setting for the roll D-controller //18.0
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

byte eeprom_data[36];
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int battery_voltage;
int gyro_address;
bool gyroCalibrated = false;
uint16_t receiver_input[5];
int acc_axis[4], gyro_axis[4];

long acc_x, acc_y, acc_z, acc_total_vector;
double gyro_pitch, gyro_roll, gyro_yaw;
uint16_t loop_timer;
double gyro_axis_cal[4];
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

void setup()
{
	//Serial.begin(57600);
	//Copy the EEPROM data for fast access data.
	for(int i = 0; i <= 35; i++)eeprom_data[i] = EEPROM.read(i);
	gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.

	Wire.begin();                                                             //Start the I2C as master.
	TWBR = 12;                                                                //Set the I2C clock speed to 400kHz.
	
	TCCR1A = 0;
	TCCR1B = 1 << 1;
	TIMSK1 = 0;

	//Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
	DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
	DDRB |= B00110000;                                                        //Configure digital poort 12 and 13 as output.

	//Use the led on the Arduino for startup indication.
	digitalWrite(12,HIGH);                                                    //Turn on the warning led.

	//Check the EEPROM signature to make sure that the setup program is executed.
	while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

	//The flight controller needs the MPU-6050 with gyro and accelerometer
	//If setup is completed without MPU-6050 stop the flight controller program  
	if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

	set_gyro_registers();                                                     //Set the specific gyro registers.

	for (int i = 0; i < 1250; i++){                           //Wait 5 seconds before continuing.
		PORTD |= B11110000;
		delayMicroseconds(1000);
		PORTD &= B00001111;
		delayMicroseconds(3000);
	}

	//Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
	for (int i = 0; i < 2000; i++){
		if(i % 15 == 0)digitalWrite(12, !digitalRead(12));                //Change the led status to indicate calibration.
		gyro_signalen();
		gyro_axis_cal[1] += gyro_axis[1];
		gyro_axis_cal[2] += gyro_axis[2];
		gyro_axis_cal[3] += gyro_axis[3];
		
		PORTD |= B11110000;
		delayMicroseconds(1000);
		PORTD &= B00001111;
		delayMicroseconds(3000);
	}
	gyro_axis_cal[1] /= 2000;
	gyro_axis_cal[2] /= 2000;
	gyro_axis_cal[3] /= 2000;
	gyroCalibrated = true;

	//Pin change interrupt for receiver signal
	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT0);
	PCMSK0 |= (1 << PCINT1);
	PCMSK0 |= (1 << PCINT2);
	PCMSK0 |= (1 << PCINT3);
	
	//Wait until the receiver is active and the throtle is set to the lower position.
	int i = 0;
	while(receiver_input_channel_3 < 990*USEC || receiver_input_channel_3 > 1020*USEC || receiver_input_channel_4 < 1400*USEC){
		receiver_input_channel_3 = convert_receiver_channel(3);
		receiver_input_channel_4 = convert_receiver_channel(4);
		i++;
		
		//We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
		PORTD |= B11110000;
		delayMicroseconds(1000);
		PORTD &= B00001111;
		delayMicroseconds(3000);
		if(i == 125){							//Blink the LED every 125 loops (~500ms)
			digitalWrite(12, !digitalRead(12));
			i = 0;
		}
	}

	//Load the battery voltage to the battery_voltage variable.
	//65 is the voltage compensation for the diode.
	//12.6V equals ~5V @ Analog 0.
	//12.6V equals 1023 analogRead(0).
	//1260 / 1023 = 1.2317.
	//The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
	//14.80 ~ 5V -> 1480/1023 = 1.4467
	battery_voltage = (analogRead(0) + 65) * 1.4467;

	loop_timer = TCNT1;			//Set the timer for the next loop.
	digitalWrite(12,LOW);		//When everything is done, turn off the led.
}

float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
byte start = 0;

void loop()
{
	//65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
	gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);
	gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
	gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);

	//Gyro angle calculations
	//0.0000611 = 1 / (250Hz / 65.5)
	angle_pitch += gyro_pitch * 0.0000611;
	angle_roll += gyro_roll * 0.0000611;

	//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
	angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);				//If the IMU has yawed transfer the roll angle to the pitch angel.
	angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);				//If the IMU has yawed transfer the pitch angle to the roll angel.

	//Accelerometer angle calculations
	acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

	if(abs(acc_y) < acc_total_vector){										//Prevent the asin function to produce a NaN
		angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
	}
		if(abs(acc_x) < acc_total_vector){									//Prevent the asin function to produce a NaN
		angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
	}

	angle_pitch_acc -= 0.0;		//Accelerometer calibration value for pitch.
	angle_roll_acc -= 0.0;		//Accelerometer calibration value for roll.

	angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;			//Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
	angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;				//Correct the drift of the gyro roll angle with the accelerometer roll angle.

	float pitch_level_adjust = angle_pitch * 15;							//Calculate the pitch angle correction
	float roll_level_adjust = angle_roll * 15;								//Calculate the roll angle correction

	//For starting the motors: throttle low and yaw left (step 1).
	if(receiver_input_channel_3 < 1050*USEC && receiver_input_channel_4 < 1050*USEC)start = 1;
	//When yaw stick is back in the center position start the motors (step 2).
	if(start == 1 && receiver_input_channel_3 < 1050*USEC && receiver_input_channel_4 > 1450*USEC){
		start = 2;

		angle_pitch = angle_pitch_acc;										//Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
		angle_roll = angle_roll_acc;										//Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

		//Reset the PID controllers for a bumpless start.
		pid_i_mem_roll = 0;
		pid_last_roll_d_error = 0;
		pid_i_mem_pitch = 0;
		pid_last_pitch_d_error = 0;
		pid_i_mem_yaw = 0;
		pid_last_yaw_d_error = 0;
	}
	//Stopping the motors: throttle low and yaw right.
	if(start == 2 && receiver_input_channel_3 < 1050*USEC && receiver_input_channel_4 > 1950*USEC)start = 0;

	
	//We need a little dead band of 16us for better results.
	if(receiver_input_channel_1 > 1508*USEC) {
		pid_roll_setpoint = receiver_input_channel_1 - 1508*USEC;
	}
	else if(receiver_input_channel_1 < 1492*USEC) {
		pid_roll_setpoint = receiver_input_channel_1 - 1492*USEC;
	}
	else {
		pid_roll_setpoint = 0;
	}
	
	//In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	pid_roll_setpoint -= roll_level_adjust;
	pid_roll_setpoint /= 3.0;

	
	//We need a little dead band of 16us for better results.
	if(receiver_input_channel_2 > 1508*USEC) {
		pid_pitch_setpoint = receiver_input_channel_2 - 1508*USEC;
	}
	else if(receiver_input_channel_2 < 1492*USEC) {
		pid_pitch_setpoint = receiver_input_channel_2 - 1492*USEC;
	}
	else {
		pid_pitch_setpoint = 0;
	}

	//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	pid_pitch_setpoint -= pitch_level_adjust;
	pid_pitch_setpoint /= 3.0;

	//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	//We need a little dead band of 16us for better results.
	if(receiver_input_channel_3 > 1050*USEC){ //Do not yaw when turning off the motors.
		if(receiver_input_channel_4 > 1508*USEC) {
			pid_yaw_setpoint = (receiver_input_channel_4 - 1508*USEC)/3.0;
		}
		else if(receiver_input_channel_4 < 1492*USEC) {
			pid_yaw_setpoint = (receiver_input_channel_4 - 1492*USEC)/3.0;
		}
		else {
			pid_yaw_setpoint = 0;
		}
	}

	calculate_pid();			//PID inputs are known. So we can calculate the pid output.

	//The battery voltage is needed for compensation.
	//A complementary filter is used to reduce noise.
	//0.09853 = 0.08 * 1.2317.
	//1.4467 * 0.8 = 0.115736
	battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.115736;

	//Turn on the led if battery voltage is to low.
	if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12, HIGH);


	uint16_t esc_1, esc_2, esc_3, esc_4;
	int throttle = receiver_input_channel_3;

	if (start == 2){
		//We need some room to keep full control at full throttle.
		if (throttle > 1800*USEC) throttle = 1800*USEC;
		esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
		esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
		esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
		esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

		//If battery is connected compensate for voltage drop
		if (battery_voltage < 1240 && battery_voltage > 800){
			esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);
			esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);
			esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);
			esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);
		}
		
		//Keep the motors running.
		if (esc_1 < 1100*USEC) esc_1 = 1100*USEC;                                         
		if (esc_2 < 1100*USEC) esc_2 = 1100*USEC;
		if (esc_3 < 1100*USEC) esc_3 = 1100*USEC;
		if (esc_4 < 1100*USEC) esc_4 = 1100*USEC;

		//Limit the esc-1 pulse to 2000us.
		if(esc_1 > 2000*USEC)esc_1 = 2000*USEC;
		if(esc_2 > 2000*USEC)esc_2 = 2000*USEC;
		if(esc_3 > 2000*USEC)esc_3 = 2000*USEC;
		if(esc_4 > 2000*USEC)esc_4 = 2000*USEC;
	}
	else{	//If start is not 2 keep a 1000us pulse
		esc_1 = 1000*USEC;
		esc_2 = 1000*USEC;
		esc_3 = 1000*USEC;
		esc_4 = 1000*USEC;
	}

	if(TCNT1 - loop_timer > (4050*USEC)) digitalWrite(12, HIGH);	//Turn on the LED if the loop time exceeds 4050us.

	//All the information for controlling the motor's is available.
	//The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
	while(TCNT1 - loop_timer < (4000*USEC);
	loop_timer = TCNT1;

	PORTD |= B11110000;
	
	//There is always 1000us of spare time. So let's do something usefull that is very time consuming.
	//Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
	gyro_signalen();

	while(PORTD >= 16){		//Stay in this loop until output 4,5,6 and 7 are low.
		uint16_t onTime = TCNT1 - loop_timer;
		if(onTime >= esc_1)PORTD &= B11101111;
		if(onTime >= esc_2)PORTD &= B11011111;
		if(onTime >= esc_3)PORTD &= B10111111;
		if(onTime >= esc_4)PORTD &= B01111111;
	}
}

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint16_t timer_1, timer_2, timer_3, timer_4;

ISR(PCINT0_vect)
{
	uint16_t current_time = TCNT1;
	if(PINB & B00000001){
		if(last_channel_1 == 0){		//Input 8 changed from 0 to 1.
			last_channel_1 = 1;
			timer_1 = current_time;
		}
	}
	else if(last_channel_1 == 1){		//Input 8 is not high and changed from 1 to 0.
		last_channel_1 = 0;
		receiver_input[1] = current_time - timer_1;
	}
	if(PINB & B00000010 ){
		if(last_channel_2 == 0){		//Input 9 changed from 0 to 1.
			last_channel_2 = 1;
			timer_2 = current_time;
		}
	}
	else if(last_channel_2 == 1){		//Input 9 is not high and changed from 1 to 0.
		last_channel_2 = 0;
		receiver_input[2] = current_time - timer_2;
	}
	if(PINB & B00000100 ){
		if(last_channel_3 == 0){		//Input 10 changed from 0 to 1.
			last_channel_3 = 1;
			timer_3 = current_time;
		}
	}
	else if(last_channel_3 == 1){		//Input 10 is not high and changed from 1 to 0.
		last_channel_3 = 0;
		receiver_input[3] = current_time - timer_3;
	}
	if(PINB & B00001000 ){
		if(last_channel_4 == 0){		//Input 11 changed from 0 to 1.
			last_channel_4 = 1;
			timer_4 = current_time;
		}
	}
	else if(last_channel_4 == 1){		//Input 11 is not high and changed from 1 to 0.
		last_channel_4 = 0;
		receiver_input[4] = current_time - timer_4;
	}
}

int temperature;

void gyro_signalen()
{
	//Read the MPU-6050
	if(eeprom_data[31] == 1){
		Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
		Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
		Wire.endTransmission();                                                 //End the transmission.
		Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.

		//Convert receiver signals
		receiver_input_channel_1 = convert_receiver_channel(1);
		receiver_input_channel_2 = convert_receiver_channel(2);
		receiver_input_channel_3 = convert_receiver_channel(3);
		receiver_input_channel_4 = convert_receiver_channel(4);
		
		while(Wire.available() < 14);		//Wait until the 14 bytes are received.
		acc_axis[1] = Wire.read()<<8|Wire.read();
		acc_axis[2] = Wire.read()<<8|Wire.read();
		acc_axis[3] = Wire.read()<<8|Wire.read();
		temperature = Wire.read()<<8|Wire.read();
		gyro_axis[1] = Wire.read()<<8|Wire.read();
		gyro_axis[2] = Wire.read()<<8|Wire.read();
		gyro_axis[3] = Wire.read()<<8|Wire.read();
	}

	if(gyroCalibrated){						//Only compensate after the calibration.
		gyro_axis[1] -= gyro_axis_cal[1];
		gyro_axis[2] -= gyro_axis_cal[2];
		gyro_axis[3] -= gyro_axis_cal[3];
	}
	//Set correct axes according to Configuration
	gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];
	if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;
	gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
	if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;
	gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];
	if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;

	acc_x = acc_axis[eeprom_data[29] & 0b00000011];
	if(eeprom_data[29] & 0b10000000)acc_x *= -1;
	acc_y = acc_axis[eeprom_data[28] & 0b00000011];
	if(eeprom_data[28] & 0b10000000)acc_y *= -1;
	acc_z = acc_axis[eeprom_data[30] & 0b00000011];
	if(eeprom_data[30] & 0b10000000)acc_z *= -1;
}

void calculate_pid()
{
	//Roll calculations
	float pid_roll_error = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_roll_error;
	if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = pid_p_gain_roll * pid_roll_error + pid_i_mem_roll + pid_d_gain_roll * (pid_roll_error - pid_last_roll_d_error);
	if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_roll_error;

	//Pitch calculations
	float pid_pitch_error = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_pitch_error;
	if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_pitch_error + pid_i_mem_pitch + pid_d_gain_pitch * (pid_pitch_error - pid_last_pitch_d_error);
	if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_pitch_error;

	//Yaw calculations
	float pid_yaw_error = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_yaw_error;
	if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_yaw_error + pid_i_mem_yaw + pid_d_gain_yaw * (pid_yaw_error - pid_last_yaw_d_error);
	if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_yaw_error;
}

int convert_receiver_channel(byte function)
{
	byte channel, reverse;

	channel = eeprom_data[function + 23] & 0b00000111;			//What channel corresponds with the specific function
	if(eeprom_data[function + 23] & 0b10000000)reverse = 1;
	else reverse = 0;

	uint16_t actual = receiver_input[channel];
	
	if(reverse)
		return 3000 - actual;
	else
		return actual;
}

void set_gyro_registers(){
	//Setup the MPU-6050
	if(eeprom_data[31] == 1){
		Wire.beginTransmission(gyro_address);
		Wire.write(0x6B);
		Wire.write(0x00);			//Set PWR_MGMT_1 = 00000000 to activate the gyro
		Wire.endTransmission();

		Wire.beginTransmission(gyro_address);
		Wire.write(0x1B);
		Wire.write(0x08);			//Set GYRO_CONFIG = 00001000 (500dps full scale)
		Wire.endTransmission();

		Wire.beginTransmission(gyro_address);
		Wire.write(0x1C);
		Wire.write(0x10);			//Set ACCEL_CONFIG = 00010000 (+/- 8g full scale range)
		Wire.endTransmission();

		//Let's perform a random register check to see if the values are written correct
		Wire.beginTransmission(gyro_address);
		Wire.write(0x1B);
		Wire.endTransmission();
		Wire.requestFrom(gyro_address, 1);
		while(Wire.available() < 1);
		if(Wire.read() != 0x08){
			digitalWrite(12,HIGH);	//Turn on the warning led
			while(1)delay(10);
		}

		Wire.beginTransmission(gyro_address);
		Wire.write(0x1A);
		Wire.write(0x03);			//Set CONFIG = 00000011 (Set Digital Low Pass Filter to ~43Hz)
		Wire.endTransmission();
	}  
}
