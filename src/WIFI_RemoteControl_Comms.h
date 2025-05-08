/*
	WIFI_RemoteControl_Comms.h is a library for Sending Commands to and from 2 wifi enabled devices
*/

#ifndef WIFI_RemoteControl_Comms_h
#define WIFI_RemoteControl_Comms_h

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

class WIFI_RemoteControl_Comms
{
private:
	//********************		WIFI Config			********************
	uint16_t myPort = 9876; // Port for Communications
	WiFiServer *myServer;
	WiFiClient myClient;

	//********************		Network Config		********************
	IPAddress myIP;
	IPAddress myServerIP; // the fix IP address of the server
	IPAddress mySubnet;	  //	Netmask for Communication
	IPAddress myGateway;  // Gateway IP address set to Zeros

	//********************		APP Config			********************
	char _cmd[100];			 // Buffer to Store Command chars received from WiFi Peer
	char _cmd_message[80];	 // Buffer that stores the message portion of the command
	char _cmd_recipient[20]; // Buffer to store the Command recipient
	bool _isTank = true;	 // Is this node on the network a Tank or a RemoteControl?

	//********************		Internal Methods	********************
	void setTank(const char *ssid, const char *pw);
	void setRemoteControl(const char *ssid, const char *pw);
	void getCmdRecipient();
	void getCmdMessage();
	void setCmdRecipient(const char *command_recipient);
	void setCmdMessage(const char *command_message);
	void instanceConfiguration(bool isTank, const char *ssid, const char *pw);

public:
	// Constructor
	WIFI_RemoteControl_Comms();

	// Methods
	void controllerStart(bool isTank, const char *ssid, const char *pw);
	void msgSend(const char *command_recipient, const char *command_message);
	bool msgReceive();
	char *command_recipient();
	char *command_message();
};
#endif
