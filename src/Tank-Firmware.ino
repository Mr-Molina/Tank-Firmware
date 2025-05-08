
/*

Robotics Club Tank Control Program
Version 5
Uses:
	1 NodeMCU Microcontroller as Tank
	1 to 2 L9110 Motor Diver Modules
	1 Cellphone App
	OR
	1 NodeMCU Powered Remote Control(Configured as REMOTE).

*/

//********************Load Code Libraries ********************
#include "L9110.h"
#include "WIFI_RemoteControl_Comms.h"
#include <string.h>

//********************WIFI Config********************
const char *ssid = "Tank_5"; // You will connect your phone or Remote to this Access Point
const char *pw = "Password"; // and this is the password

//********************NodeMCU PINS Config********************
/*
This Section tells the programm how to use the PINS as they are printed on the ModeMCU chip
*/
#define D0 16
#define D1 5 // I2C Bus SCL (clock)
#define D2 4 // I2C Bus SDA (data)
#define D3 0
#define D4 2  // Same as "LED_BUILTIN", but inverted logic
#define D5 14 // SPI Bus SCK (clock)
#define D6 12 // SPI Bus MISO
#define D7 13 // SPI Bus MOSI
#define D8 15 // SPI Bus SS (CS)
#define D9 3  // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)

//********************Motor Cotroller Config********************
L9110 Body_Motors(D0, D1, D2, D3, 1023);

//********************Network Manager Config********************
WIFI_RemoteControl_Comms Message_Manager;

//********************Timing Config********************
unsigned long aliveSentTime = 0;
unsigned long receivePollSentTime = 0;
unsigned long messageIndex = 0;
//********************Methods********************
// This Method decides what to do when a message arrives from the WiFi
void processMessage(char *command_recipient, char *command_message)
{
	if (strcmp(command_recipient, "motorA") == 0)
	{
		Body_Motors.motor_A(command_message);
	}

	if (strcmp(command_recipient, "motorB") == 0)
	{
		Body_Motors.motor_B(command_message);
	}
	if (strcmp(command_recipient, "alive") == 0)
	{

		Serial.print(command_recipient);
		Serial.print(" ");
		Serial.println(command_message);
	}
}

// This Method Run Exactly Once when the chip first powers up. Here we prepare our variables.
void setup()
{
	// setup PWM Settings (Pulse Width Modulation)
	analogWriteFreq(500); // should give 5Khz
	analogWriteRange(1023);

	// Debug code zone
	Serial.begin(115200);

	// Start Message Center
	Message_Manager.controllerStart(true, ssid, pw);
}

// This Method Runs repeatedly forever. THis is the Main Body of our Program.
void loop()
{
	if (Message_Manager.msgReceive())
	{
		processMessage(Message_Manager.command_recipient(), Message_Manager.command_message());
	}

	if (millis() - aliveSentTime > 10000)
	{
		char buf[14];

		// every 500ms send an (I AM ALIVE) message
		Message_Manager.msgSend("alive", itoa(messageIndex, buf, 10));

		// Set last time alive message was set timer(aliveSentTime) to current time(millis)
		aliveSentTime = millis();

		messageIndex++;
	}
}
