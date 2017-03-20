//Include Libraries
//#include <LiquidCrystal.h> //Include the LCD Display Library

//Analog Ports
#define joy_Xaxis A1 //JOYSTICK X Axis (in)-(Analog)
#define joy_Yaxis A2 //JOYSTICK Y Axis (in)-(Analog)
#define joy_Zaxis A3 //JOYSTICK Z Axis (in)-(Analog)
#define accel_Xaxis A4 //ACCELEROMETER X Axis (in)-(Analog)
#define accel_Yaxis A5 //ACCELEROMETER Y Axis (in)-(Analog)
#define accel_Zaxis A6 //ACCELEROMETER Z Axis (in)-(Analog)
#define VC_voltage A7 //VOLTAGE/CURRENT Sensor Va (in)-(Analog)
#define VC_current A8 //VOLTAGE/CURRENT Sensor Ia (in)-(Analog)

//Digital Ports
#define RFID_reader 0 //Read the value of the RFID tag (in)-(Serial RX)
#define RFID_enable 22 //Enable the RFID reader to read tags (out)-(Digital)
#define motor_front 2 //M1 - Front MOTOR Controller Y-Cabled (out)-(PWM)
#define motor_right 3 //M2 - Right MOTOR Controller Y-Cabled (out)-(PWM)
#define motor_left 4  //M3 - Left  MOTOR Controller Y-Cabled (out)-(PWM)
#define LED_red 5 //RED LED used to tell if the robot is ready or not (out)-(PWM)
#define LED_green 6 //GREEN LED used to tell if the robot is ready or not (out)-(PWM)
#define LED_blue 7 //BLUE LED used to tell if the robot is ready or not (out)-(PWM)
#define tiltSense 23 //TILT Sensor used to tell when the robot has tipped too far (in)-(Digital)
#define floorMat 24 //FLOOR MAT switch that is used to tell when a user is on the robot (in)-(Digital)
#define btn_red 25 //BUTTON used to tell to Emergency Stop (in)-(Digital)
#define btn_yellow 25 //BUTTON used to tell to select standby mode (in)-(Digital)
#define btn_green 26 //BUTTON used to tell to select driver mode (in)-(Digital)
#define btn_blue 27 //BUTTON used to tell to select autonomous mode (in)-(Digital)
#define sonar_echo 00 //SONAR Sensor receive distance from said object (in)-(Digital)
#define sonar_trig 00 //SONAR Sensor send out sonar sounds (out)-(Digital)
#define keypad_r0 00 //KEYPAD Row 0 (in)-(Digital)
#define keypad_r1 00 //KEYPAD Row 0 (in)-(Digital)
#define keypad_r2 00 //KEYPAD Row 0 (in)-(Digital)
#define keypad_r3 00 //KEYPAD Row 0 (in)-(Digital)
#define keypad_c0 00 //KEYPAD Row 0 (in)-(Digital
#define keypad_c1 00 //KEYPAD Row 0 (in)-(Digital)
#define keypad_c2 00 //KEYPAD Row 0 (in)-(Digital)
#define LCD_register 00 //LCD Register Pin (in)-(Digital)
#define LCD_enabler 00 //LCD Enabler Pin (out)-(Digital)
#define LCD_d4 00 //LCD D4 Pin (out)-(Digital)
#define LCD_d5 00 //LCD D5 Pin (out)-(Digital)
#define LCD_d6 00 //LCD D6 Pin (out)-(Digital)
#define LCD_d7 00 //LCD D7 Pin (out)-(Digital)

/*-------------------------------------------------------------------------------
Project Title: Night Crawler V8
Creator: Ikaika
Reason: Project
Driver Description:
-)  Kiwi Drive
-)  Master Control (Manual Override)
-)  On Board Drive Control

CONTROL CONFIGURATION:
On Board Control:
-)  joy_Xaxis controls point turning right and left
-)  joy_Yaxis controls forward and backward movement
-)  joy_Zaxis controls strafing left and right
-)  btn_red tells the robot to insta-stop and change mode to 0 (Standby)
-)	btn_yellow tells the robot to coast to stop and change mode to 0 (Standby)
-)	btn_green tells the robot to change mode to 1 (Driver)
-)	btn_blue tells the robot to change mode to 2 (Autonomous)
-------------------------------------------------------------------------------*/

int weight = 0; //Tell what the weight of the user is
int count = 0; //Keep track of the amount of times the program has cycled through
int eCount = 0; //Keep track of the amount of times the program has cycled through
int weightDigits[3]; //Hold each digit of persons weight
int const deadzone = 16; //Define the dead zone of the controller joysticks
boolean authorizedUser = false; //Tell whether the user is authorized
boolean weightGiven = false; //Tell whether the users weight is given
boolean modeSelected = false; //Tell whether the mode has been selected
boolean lightFlashed = false; //Switch between LED on and off
//-------------------------------------------------------------------------------
//Variables for speed of the motors
int const maxSpeed = 127; //Define the max speed of the motor
int const minSpeed = -127; //Define the min speed of the motor
//-------------------------------------------------------------------------------
//Variables for joystick values
int X1 = 0; //Driver Control - X Axis
int Y1 = 0; //Driver Control - Y Axis
int Z1 = 0; //Driver Control - Z Axis
//-------------------------------------------------------------------------------
//Variables for speed control
int fSpeed = 0; //Front wheel driver
int rSpeed = 0; //Right wheel driver
int lSpeed = 0; //Left wheel driver
//-------------------------------------------------------------------------------
//Variables for color arrays
int black[3]  = { 0, 0, 0 };
int white[3]  = { 100, 100, 100 };//Standby - Wait for Authorization
int red[3]    = { 100, 0, 0 }; //Not Ready - Configuring || Error Sensed (blinking) || Unauthorized
int green[3]  = { 0, 100, 0 }; //Active - Driver Control
int blue[3]   = { 0, 0, 100 };
int yellow[3] = { 40, 95, 0 }; //Authorized - Standby for Weight/Mode Selection
int orange[3] = { 255, 165, 0 }; //Active - Autonomous
//-------------------------------------------------------------------------------
int nControl = 0; //Keep track of the phase of the robot
// 0 - Standby - Wait for authorization
// 1 - Choices - Choose weight/mode
// 2 - Driver - Only driven by driver controls 
// 3 - Autonomous - Follows a person 
// 4 - Termination - Stop all motors
// 5 - Diagnosis - Check for Errors
//-------------------------------------------------------------------------------
int permLvl = 0; //Keep track of permission level
// 0 - No Permission
// 1 - Permit Guest (Set Timer)
// 2 - Permit User (No Timer)
// 3 - Permit Admin  (No Timer, Given Weight)
//-------------------------------------------------------------------------------

/*-------------------------------------------------------------------------------
3 x Motor Controller [Y-Cabled]
2 x Momentary Push Button [ORANGE, GREEN]
2 x LCD Display [2x16]
2 x RGD LED
1 x Bluetooth Module
1 x Emergency Push Button [RED]
1 x Floor Mat Sensor (Load Sensor)
1 x Joystick Module [X-axis, Y-axis, Z-axis]
1 x Keypad [3x4]
1 x IMU [3-axis Gyro, 3-axis Accelerometer]
1 x Image Sensor [X-Axis Only]
1 x Push Button Switch
1 x RGB LED Strip [1 meter]
1 x Sonar Sensor
1 x Temperature Sensor
1 x Voltage & Current Sensor
-------------------------------------------------------------------------------*/

int mapJoy(int pValue)	//Map joystick values to fit within range
{
	int rValue = analogRead(pValue);
	int jValue = map(rValue, 0, 255, -127, 127);
	return jValue;
}

void statusColor(int colorCode[3])	// Change the color of the LEDs
{
	analogWrite(LED_red, colorCode[0]);
	analogWrite(LED_green, colorCode[1]);
	analogWrite(LED_blue, colorCode[2]);
}

void updateMotors()  //Update Motor Speeds
{
	analogWrite(motor_front, fSpeed);
	analogWrite(motor_right, rSpeed);
	analogWrite(motor_left, lSpeed);
}

void setup()//Configure Robot
{
	statusColor(red); //Set status LED to red

	//Configure all analog sensors
	pinMode(joy_Xaxis, INPUT);
	pinMode(joy_Yaxis, INPUT);
	pinMode(joy_Zaxis, INPUT);
	pinMode(accel_Xaxis, INPUT);
	pinMode(accel_Yaxis, INPUT);
	pinMode(accel_Zaxis, INPUT);
	pinMode(VC_voltage, INPUT);
	pinMode(VC_current, INPUT);

	//Configure all digital sensors
	pinMode(motor_front, OUTPUT);
	pinMode(motor_left, OUTPUT);
	pinMode(motor_right, OUTPUT);
	pinMode(LED_red, OUTPUT);
	pinMode(LED_green, OUTPUT);
	pinMode(LED_blue, OUTPUT);
	pinMode(RFID_reader, INPUT);
	pinMode(RFID_enable, OUTPUT);
	pinMode(tiltSense, INPUT);
	pinMode(floorMat, INPUT);
	pinMode(btn_red, INPUT);
	pinMode(btn_yellow, INPUT);
	pinMode(btn_green, INPUT);
	pinMode(btn_blue, INPUT);

	//Set all motor speeds to Zero
	analogWrite(motor_front, 0); //stop all motors
	analogWrite(motor_left, 0); //stop all motors
	analogWrite(motor_right, 0); //stop all motors

	//Setup all sensors
	digitalWrite(RFID_enable, HIGH);//Turn off RFID Reader

	Serial.begin(19200); // Start serial port 19200 bps

	// Setting Auto Read Mode - EM4102 Decoded Mode - No password
	Serial.write(0xFF);  //Header
	Serial.write(0x01);  //Reserved
	Serial.write(0x09);  //Length (Command + Data)
	Serial.write(0x87);  //Command (0x87 sets auto mode behavior
	Serial.write(0x01);  //Data 1: Enable Auto-Read
	Serial.write(0x03);  //Data 2: Mode � Parity decoded � Manchester RF/64
	Serial.write(0x02);  //Data 3: Total number of block to be read (2)
	Serial.write(byte(0x00));  //Data 4: No password expected
	Serial.write(0x10);  //Data 5: Password byte 1
	Serial.write(0x20);  //Data 6: Password byte 2
	Serial.write(0x30);  //Data 7: Password byte 3
	Serial.write(0x40);  //Data 8: Password byte 4
	Serial.write(0x37);  //Checksum

	delay(500);

	while (Serial.available() > 0) //Check to make sure the serial is available
	{
		Serial.read();
	}
	statusColor(white);//Turn status LED to white
}

void loop()
{
	
	//Check for errors every 200th cycle
	if(count == 200)//Check for errors
	{
		
		//QUICK CHECK FOR ERRORS
		
		eCount = 0;
		count++;
	}
	else //Count up
	{	
		eCount++;
		count++;
	}
	
	
	//PUT ROUTINE HERE
	
	
	switch (nControl)//Start switch case "nControl"
	{
		case 1://DRIVER Mode

		statusColor(green); //Turn status LED to green
		digitalWrite(RFID_enable, HIGH); //Turn off RFID Reader
		updateMotors();//Update motors constantly
		if(digitalRead(btn_red)) nControl = 0; //STOP all motors

		//Update Joystick Values
		X1 = abs(mapJoy(joy_Xaxis)) > deadzone ? mapJoy(joy_Xaxis) : 0;
		Y1 = abs(mapJoy(joy_Yaxis)) > deadzone ? mapJoy(joy_Yaxis) : 0;
		Z1 = abs(mapJoy(joy_Zaxis)) > deadzone ? mapJoy(joy_Zaxis) : 0;

		//Update Motor Values
		fSpeed = (X1 + Z1);
		rSpeed = ((.5 * X1) - (sqrt(3 / 2) * Y1) - Z1);
		lSpeed = ((.5 * X1) + (sqrt(3 / 2) * Y1) + Z1);

		break;
		////////////////////////////////////////////////////////////////////
		default://STANDBY Mode

		statusColor(yellow); //Turn status LED to yellow

		//Reset All Variables
		digitalWrite(RFID_enable, HIGH);//Turn off RFID Reader
		authorizedUser = false; //Not Authorized
		weightGiven = false; //Tell that weight hasn't been given
		weight = 0; //Zero Weight

		//Stop all motor movement
		fSpeed = 0;
		rSpeed = 0;
		lSpeed = 0;

		updateMotors(); //Update motors

		//Read RFID for first value (0xff)
		RFID_val = Serial.read();
		while (RFID_val != 0xff)//On Successful read, first byte will always be 0xFF
		{
			RFID_val = Serial.read();
			if (digitalRead(floorMat))	digitalWrite(RFID_enable, LOW);
			else digitalWrite(RFID_enable, HIGH);
			delay(1000);
		}

		Serial.read();              // read reserved
		Serial.read();              // read length
		Serial.read();              // read command (indicates tag data)
		data[0] = Serial.read();    // read data 1
		data[1] = Serial.read();    // read data 2
		data[2] = Serial.read();    // read data 3
		data[3] = Serial.read();    // read data 4
		data[4] = Serial.read();    // read data 5
		Serial.read();              // read checksum

		// Identify RFID Card
		master_tag = true; //Assumed as Master until proven otherwise
		principal_tag = true; //Assumed as Principal until proven otherwise
		teacher_tag = true; //Assumed as Teacher until proven otherwise
		mentor_tag = true; //Assumed as Mentor until proven otherwise
		student_tag = true; //Assumed as Student until proven otherwise
		d1_tag = d2_tag = d3_tag = d4_tag = true; //Assumed as Guest until proven otherwise

		for (int i = 0; i < 5; i++) //Cross check
		{
			if (data[i] != master_RFID[i]) master_tag = false;
			if (data[i] != principal_RFID[i]) principal_tag = false;
			if (data[i] != teacher_RFID[i]) teacher_tag = false;
			if (data[i] != mentor_RFID[i]) mentor_tag = false;
			if (data[i] != student_RFID[i]) student_tag = false;
			if (data[i] != demoOne_RFID[i]) d1_tag = false;
			if (data[i] != demoTwo_RFID[i]) d2_tag = false;
			if (data[i] != demoThree_RFID[i]) d3_tag = false;
			if (data[i] != demoFour_RFID[i]) d4_tag = false;
		}

		//READ PERMISSION CODE & SET AS PERMLVL
		//CONVERT PERMBYTE TO PERMINT
		
		//READ NAME & SET AS NAME
		//CONVERT NAMEBYTE TO NAMESTRING

		if (permLvl == 3)//If permission level is 3 (Program)
		{
			authorizedUser = true; //Set as authorized user
			statusColor(white); //Set status LED to white
		}
		else if (permLvl == 2) //If permission level is 2 (User)
		{
			authorizedUser = true; //Set as authorized user
			statusColor(white); //Set status LED to white
		}
		else if (permLvl == 1)//If permission level is 1 (Guest)
		{
			//SET TIME HERE
			authorizedUser = true; //Set as authorized user
			statusColor(white); //Set status LED to white
		}
		else//If user is Unauthorized (None)
		{
			Serial.println("Not Authorized !");
			authorizedUser = false;
			statusColor(orange); //Set status LED to orange
		}

		if (authorizedUser)//Ability to choose mode for authorized users
		{
			while (!weightGiven) //While weight has not been given
			{
				//TYPE IN 3 DIGIT WEIGHT
				
				
			}
			
			//Update weight
			weight = weightDigits[0] * 100;
			weight = weight + (weightDigits[1] * 10);
			weight = weight + (weightDigits[2]);
			
			while (!modeSelected) //Choose which mode to enter
			{
				if(digitalRead(btn_red) || digitalRead(btn_yellow))
				{
					while(digitalRead(btn_red) || digitalRead(btn_yellow)){}
					delay(5);
					nControl = 0;
					modeSelected = true;
				}
				else if (digitalRead(btn_green))
				{
					while (digitalRead(btn_green)) {}
					delay(5);
					nControl = 1;
					modeSelected = true;
				}
				else if(digitalRead(btn_blue))
				{
					while (digitalRead(btn_blue)) {}
					delay(5);
					nControl = 2;
					modeSelected = true;
				}
				
			}
		}
		break;
	}
}
