/*
	L9110.cpp is a library for controlling the L9110 Motor driver
*/

#include "L9110.h"

L9110::L9110(int motor_A_A_pin, int motor_A_B_pin, int motor_B_A_pin, int motor_B_B_pin, int pwm_Range)
{
	_motor_B_IA = motor_B_A_pin;
	_motor_B_IB = motor_B_B_pin;
	_motor_A_IA = motor_A_A_pin;
	_motor_A_IB = motor_A_B_pin;
	_pwm_Range = pwm_Range;

	pinMode(_motor_B_IA, OUTPUT);
	pinMode(_motor_B_IB, OUTPUT);
	pinMode(_motor_A_IA, OUTPUT);
	pinMode(_motor_A_IB, OUTPUT);
}

void L9110::motor_A(const char *cmd_Message)
{ // Command recipient [motorA] expects a numerical cmd_message
	Serial.print("Motor A");

	int cmd_Message_Value = atoi(cmd_Message); // Variable to hold raw Numerical Value from Message
	int speed_Value = abs(cmd_Message_Value);  // Variable to hold Calculated Motor Speed

	if (motorDirection(cmd_Message_Value))
	{
		Serial.print("-->Forward :");
		Serial.println(speed_Value);

		motor_A_Forward(speed_Value);
	}
	else
	{
		Serial.print("--> Backward :");
		Serial.println(speed_Value);

		motor_A_Backward(speed_Value);
	}
}

void L9110::motor_A_Forward(int a_Speed)
{ // Motor A Forward at given speed
	if (a_Speed >= 0 && a_Speed <= _pwm_Range)
	{
		analogWrite(_motor_A_IA, a_Speed);
		analogWrite(_motor_A_IB, 0);
	}
}

void L9110::motor_A_Backward(int a_Speed)
{ // Motor A Backward at given speed
	if (a_Speed >= 0 && a_Speed <= _pwm_Range)
	{
		analogWrite(_motor_A_IA, 0);
		analogWrite(_motor_A_IB, a_Speed);
	}
}

void L9110::motor_A_Stop()
{ // Motor A Stop
	analogWrite(_motor_A_IA, 0);
	analogWrite(_motor_A_IB, 0);
}

void L9110::motor_B(const char *cmd_Message)
{ // Command recipient [motorB] expects a numerical cmd_message
	Serial.print("Motor B");

	int cmd_Message_Value = atoi(cmd_Message); // Variable to hold raw Numerical Value from Message
	int speed_Value = abs(cmd_Message_Value);  // Variable to hold Calculated Motor Speed

	if (motorDirection(cmd_Message_Value))
	{
		Serial.print("-->Forward :");
		Serial.println(speed_Value);

		motor_B_Forward(speed_Value);
	}
	else
	{
		Serial.print("--> Backward :");
		Serial.println(speed_Value);

		motor_B_Backward(speed_Value);
	}
}

void L9110::motor_B_Forward(int b_Speed)
{ // Motor B Forward at given speed
	if (b_Speed >= 0 && b_Speed <= _pwm_Range)
	{
		analogWrite(_motor_B_IA, b_Speed);
		analogWrite(_motor_B_IB, 0);
	}
}

void L9110::motor_B_Backward(int b_Speed)
{ // Motor B Backward at given speed
	if (b_Speed >= 0 && b_Speed <= _pwm_Range)
	{
		analogWrite(_motor_B_IA, 0);
		analogWrite(_motor_B_IB, b_Speed);
	}
}

void L9110::motor_B_Stop()
{ // Motor B Stop
	analogWrite(_motor_B_IA, 0);
	analogWrite(_motor_B_IB, 0);
}

boolean L9110::motorDirection(int commandValue)
{ // Direction is determined by positve/negative commandValue
	if (commandValue >= 1 && commandValue <= 1023)
	{
		return true;
	}
	else
	{
		return false;
	}
}