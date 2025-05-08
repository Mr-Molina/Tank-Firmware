/*
	WIFI_RemoteControl_Comms.cpp is is a library for Sending Commands to and from 2 wifi enabled devices
*/

#include "WIFI_RemoteControl_Comms.h"

WIFI_RemoteControl_Comms::WIFI_RemoteControl_Comms()
{
}

void WIFI_RemoteControl_Comms::controllerStart(bool isTank, const char *ssid, const char *pw)
{
	//********************		Network Config		********************
	myServerIP = IPAddress(192, 168, 0, 1); // the fix IP address of the server
	mySubnet = IPAddress(255, 255, 255, 0); //	Netmask for Communication
	myGateway = IPAddress(0, 0, 0, 0);		// Gateway IP address set to Zeros

	myServer = new WiFiServer(myPort);
	_isTank = isTank;

	// Sanity Check
	int passLen = strlen(pw);
	if (passLen < 8)
	{
		Serial.print("Wireless Network Password: ");
		Serial.print(pw);
		Serial.println("  is Too Short!");
	}
	else
	{
		instanceConfiguration(isTank, ssid, pw);
	}
}
void WIFI_RemoteControl_Comms::instanceConfiguration(bool isTank, const char *ssid, const char *pw)
{
	if (isTank)
	{
		setTank(ssid, pw);
	}
	else
	{
		setRemoteControl(ssid, pw);
	}
}

void WIFI_RemoteControl_Comms::setTank(const char *ssid, const char *pw)
{
	myIP = IPAddress(192, 168, 0, 1); // IP Adress for this Server/Tank Device

	//***Configure wifi server object***
	WiFi.softAPConfig(myIP, myGateway, mySubnet); // configure ip address, gateway, and subnet for softAP	network

	//***Start wifi server***
	if (WiFi.softAP(ssid, pw)) // configure/begin softAP with given ssid and password
	{

		myServer->begin(); // start TCP server

		// Debug code zone
		Serial.println("ESP8266 Wifi Connection Online!");
		Serial.print("SSID: ");
		Serial.print(ssid);
		Serial.print("  PASS: ");
		Serial.println(pw);

		Serial.print("Remote app/device must connect to ");
		Serial.print(myIP);
		Serial.print(":");
		Serial.println(myPort);
	}
	else
	{
		// Debug code zone
		Serial.println("ESP8266 Wifi Connection FAIL!");
		Serial.print("SSID: ");
		Serial.print(ssid);
		Serial.print("  PASS: ");
		Serial.println(pw);
	}
}

void WIFI_RemoteControl_Comms::setRemoteControl(const char *ssid, const char *pw)
{
	WiFi.begin(ssid, pw); // Start the wifi connection
	while (WiFi.status() != WL_CONNECTED)
	{
		Serial.print(".");
		delay(500);
	}
	myClient.connect(myServerIP, myPort); // Connection to the server
}

void WIFI_RemoteControl_Comms::msgSend(const char *command_recipient, const char *command_message)
{
	bool sendFail = false;
	if (_isTank)
	{
		// Check server status
		if (!myClient.connected())
		{
			myClient = myServer->available(); // grab available client. In this case RemoteControl
			delay(200);
		}
		int i = 0;
		while (!myClient.availableForWrite() && !sendFail)
		{
			delay(20);
			if (i > 4)
			{
				sendFail = true;
			}
			i++;
		}
		if (!sendFail)
		{
			strcpy(_cmd, command_recipient);
			strcat(_cmd, " ");
			strcat(_cmd, command_message);
			strcat(_cmd, "\n");
			myClient.write(_cmd); // sends the message to the RemoteControl
			myClient.flush();
			Serial.print("Sent ");
			Serial.print("Tank --> RemoteControl :");
			Serial.print(command_recipient);
			Serial.print(" ");
			Serial.println(command_message);
		}
		else
		{
			Serial.println("Send Failed by timeout");
		}
	}
	else
	{
		int i = 0;
		while (!myClient.availableForWrite() && !sendFail)
		{
			delay(20);
			if (i > 4)
			{
				sendFail = true;
			}
			i++;
		}
		if (!sendFail)
		{
			strcpy(_cmd, command_recipient);
			strcat(_cmd, " ");
			strcat(_cmd, command_message);
			strcat(_cmd, "\n");
			myClient.write(_cmd); // sends the message to the Tank
			myClient.flush();
			Serial.print("Sent ");
			Serial.print("RemoteControl --> Tank :");
			Serial.print(command_recipient);
			Serial.print(" ");
			Serial.println(command_message);
		}
		else
		{
			Serial.println("Send Failed by timeout");
		}
	}
}

bool WIFI_RemoteControl_Comms::msgReceive()
{
	bool newCommand = false;

	// Check server status
	if (!myClient.connected())
	{
		myClient = myServer->available();
		return newCommand;
	}

	if (myClient.available())
	{
		//_cmd = myClient.readStringUntil('\n').c_str();
		strncpy(_cmd, myClient.readStringUntil('\n').c_str(), sizeof(_cmd) - 1); // read the command
		if (sizeof(_cmd) > 1)
		{
			getCmdRecipient(); // Get Recipient out of Command
			getCmdMessage();   // Get Message out of Command
			newCommand = true;
			Serial.print("Received ");
			if (_isTank)
			{
				Serial.print("RemoteControl --> Tank :");
			}
			else
			{
				Serial.print("Tank --> RemoteControl :");
			}
		}
	}
	return newCommand;
}

void WIFI_RemoteControl_Comms::getCmdRecipient()
{
	char *temp;					// Create Temporary varriable to hold pointer record
	int index;					// Create Temporary varriable to hold index of translated pointer record
	temp = strchr(_cmd, ' ');	// Get the Pointer of the first ' ' character in the cmd[]
	index = (int)(temp - _cmd); // Translate the Pointer of the first ' ' character in the cmd[] into an int
	if (index > sizeof(_cmd_recipient))
	{
		strncpy(_cmd_recipient, _cmd, sizeof(_cmd_recipient) - 1); // copy the substring that equals the recipient to the _cmd_recipient value
	}
	else
	{
		strncpy(_cmd_recipient, _cmd, index); // copy the substring that equals the recipient to the _cmd_recipient value
	}
}

void WIFI_RemoteControl_Comms::getCmdMessage()
{												 // checks if command starts with the cmd_Recipient in compareString
	strcpy(_cmd_message, strchr(_cmd, ' ') + 1); // Load cmd_message into buffer
}

char *WIFI_RemoteControl_Comms::command_recipient()
{
	return _cmd_recipient;
}
char *WIFI_RemoteControl_Comms::command_message()
{
	return _cmd_message;
}

void WIFI_RemoteControl_Comms::setCmdRecipient(const char *command_recipient)
{
	if (sizeof(command_recipient) > 20)
	{
		strncpy(_cmd_recipient, command_recipient, 20); // copy the substring that equals the recipient to the _cmd_recipient buffer
	}
	else
	{
		strncpy(_cmd_recipient, command_recipient, sizeof(command_recipient)); // copy the substring that equals the recipient to the _cmd_recipient buffer
	}
}

void WIFI_RemoteControl_Comms::setCmdMessage(const char *command_message)
{
	if (sizeof(command_message) > 80)
	{
		strncpy(_cmd_message, command_message, 80); // copy the substring that equals the recipient to the _cmd_recipient buffer
	}
	else
	{
		strncpy(_cmd_message, command_message, sizeof(command_message)); // copy the substring that equals the recipient to the _cmd_recipient buffer
	}
}
