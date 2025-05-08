/*
	L9110.h is a library for controlling the L9110 Motor driver
*/

#ifndef L9110_h
#define L9110_h

#include <Arduino.h>
class L9110
{
private:
	int _motor_B_IA;
	int _motor_B_IB;
	int _motor_A_IA;
	int _motor_A_IB;
	int _pwm_Range;

	// Motor A Methods
	void motor_A_Forward(int a_Speed);
	void motor_A_Backward(int a_Speed);
	void motor_A_Stop();

	// Motor B Methods
	void motor_B_Forward(int b_Speed);
	void motor_B_Backward(int b_Speed);
	void motor_B_Stop();

	boolean motorDirection(int commandValue);

public:
	// Constructor
	L9110(int motor_A_A_pin, int motor_A_B_pin, int motor_B_A_pin, int motor_B_B_pin, int pwm_Range);

	// Motor A Methods
	void motor_A(const char *cmd_Message);

	// Motor B Methods
	void motor_B(const char *cmd_Message);
};
#endif