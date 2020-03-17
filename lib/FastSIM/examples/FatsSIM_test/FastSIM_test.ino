#include "FastSIM.h"

#define SIM_BAUD 115200
FastSIM GSM;

char numero[20];
char texto[100];

unsigned long timer_ok;

void setup()
{
	Serial.begin(115200);
	GSM.init();
	timer_ok = millis();
}

void loop()
{
	switch(GSM.update())
	{
		case IDLE:			if(millis()-timer_ok > 10000)
							{
								GSM.CheckOK();
								timer_ok = millis();
							}
							break;
		case SMS_RECEIVED:	GSM.GetSMS(numero, texto, 20);
							Serial.print("Mensaje recibido de ");
							Serial.print(numero);
							Serial.print(": ");
							Serial.println(texto);
							GSM.SendSMS(numero, texto);
							break;
		case SMS_SEND:		if(GSM.SendSMS(numero, texto))
							{
								Serial.println("Mensaje enviado");
							}
							break;
		case CHECK_OK:		if(GSM.CheckOK())
							{
								Serial.println("Modulo OK");
							}
							break;
		default:			break;
	}
}

