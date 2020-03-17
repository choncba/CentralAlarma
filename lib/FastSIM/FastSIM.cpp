// FastSIM.h - Clase para controlar modulos GSM por comandos AT con funciones NO Bloqueantes
// Orientada al envio y recepcion de SMS 
// Utiliza Hardware Serial1 (ATMEGA2560, 1284, 680, etc.)
// No utiliza delay, ni corrige errores, solo es... rapida

// Parametros SIM800L
// * NO_SAVE    --> No se guarda el comando
// * AUTO_SAVE  --> Se guarda automaticamente, no se pierde al reiniciar
// * AT&W_SAVE  --> El comando se guarda solo al enviar despues el comando AT&W
//
// CONFIGURACION SIN ALMACENAMIENTO DE LOS SMS, AL RECIBIRLOS EL MODULO LOS ENVIA DIRECTAMENTE AL Serial
// - AT+CNMI=3,2,0,0,0    		- Setea el modo en que se reciben los SMS -> Con 3,2 no se almacena, envia directo al serial
// - AT&W                 		- Guarda el comando 
// - AT+CPMS="ME","ME","ME"		- Setea elalmacenamiento de mensajes al telefono (Pero con AT+CNMI=3,2,0,0,0 el almacenamiento esta desactivado)
// Asi al recibir un SMS, el modulo envia directamente al serial:
// <CR><LF>+CMT: "+543516510632","Luciano","18/10/09,13:01:48-12"<CR><LF>.... mensaje ....<CR><LF>

#include "Arduino.h"
#include "FastSIM.h"

void FastSIM::init()
{
	switch(init_status)
	{
		case NOT_STARTED:	Serial1.begin(SIM_BAUD);
							init_timer = millis();
							init_status = STARTING;
							start_attemp = 0;
#ifdef DEBUG_SIM							
							Serial.println("Iniciando SIM800");
#endif							
							break;
		case STARTING:		if( millis()-init_timer > 5000 )	// Espero 5 segundos a que levante el modulo
							{
								Serial1.println(AT);
								timeout = millis();
								prev_init_status = STARTING;
								init_status = WAIT_OK;
								RxInit();
#ifdef DEBUG_SIM								
								Serial.print("Enviando AT...");
#endif								
							}
							break;
		case SETTING:		Serial1.println(INIT_COMMAND);
							timeout = millis();
							init_status = WAIT_OK;
							prev_init_status = SETTING;
							RxInit();
#ifdef DEBUG_SIM							
							Serial.print("Enviando comando Inicio...");
#endif							
							break;
		case WAIT_OK:		if(millis()-timeout > 1000)
							{
								if(start_attemp > 5)
								{
									init_status = STARTED_ERROR;
#ifdef DEBUG_SIM									
									Serial.println("ERROR en el inicio");
#endif									
								}
								else
								{
									init_status = prev_init_status;
#ifdef DEBUG_SIM									
									Serial.println("Timeout");
#endif									
									start_attemp++;
								}
							}
							else
							{
								if(IsRxFinished() == RX_FINISHED)
								{
									if(IsStringReceived(OK))
									{
										if(prev_init_status == STARTING)
										{
											init_status = SETTING;
											start_attemp = 0;
#ifdef DEBUG_SIM											
											Serial.println("OK");
#endif											
										}
										if(prev_init_status == SETTING)
										{
#ifdef DEBUG_SIM											
											Serial.println("INICIADO OK");
#endif											
											init_status = STARTED_OK;
											sim_status = IDLE;
											RxInit();
										}
									}
								}
							}
							break;
		default:			break;
	}
}

byte FastSIM::update()
{
	// if(init_status == STARTED_OK)
	// {
		// if(sim_status == IDLE)
		// {
			if(IsRxFinished() == RX_FINISHED)
			{
				if(IsStringReceived("+CMT"))	// Devuelve <CR><LF>+CMT: "+543516510632","Luciano","18/10/09,13:01:48-12"<CR><LF>.... mensaje ....<CR><LF>
				{
					sim_status = SMS_RECEIVED;				
				}
			}
			else
			{
				if(IsRxFinished() == RX_TMOUT_ERR)
				{
					RxInit();
				}
			} 
		// }
	// }
	// else
	// {
	// 	if(init_status != STARTED_ERROR)
	// 	{
	// 		init();
	// 	}
	// }

	return sim_status;
}

bool FastSIM::CheckOK()
{
	bool retvalue = false;
	if(sim_status == IDLE)
	{
		Serial1.println(AT);
		RxInit();
		sim_status = CHECK_OK;
	}
	else
	{
		if(sim_status == CHECK_OK)
		{
			if(IsRxFinished() == RX_FINISHED)
			{
				if(IsStringReceived(OK))
				{
					sim_status = IDLE;
					retvalue = true;			
				}
				RxInit();
			}
		}
	}
	return retvalue;
}

byte FastSIM::IsStringReceived(char const *compare_string)
{
	char *ch;
	byte ret_val = 0;

	if (comm_buf_len)
	{
		ch = strstr((char *)comm_buf, compare_string);
		if (ch != NULL)
		{
			ret_val = 1;
		}
	}

	return (ret_val);
}

// Busca el texto ingresado en cmd en el buff, retorna TRUE si lo encuentra
bool FastSIM::StringFind(const char *buff, const char *cmd)
{
	char *ptr = strstr(buff, cmd);								// *ptr = primer letra de cmd encontrada en buff
	if (strncmp(ptr, cmd, strlen(cmd)) == 0)	return true;   // cmd encontrado
	else										return false;  // cmd No encontrado
}

/* Busca en el string str_origen el texto delimitado entre el string inicio y el string fin
* y copia el resultado en el string str_destino
*/
bool FastSIM::BuscarEntre(char *str_origen, char *str_destino, char *str_inicio, char *str_fin)
{
	char *inicio;
	char *fin;
	bool respuesta = false;

	inicio = strstr(str_origen, str_inicio);

	if (inicio)
	{
		inicio += strlen(str_inicio);
		fin = strstr(inicio, str_fin);
		if (fin)
		{
			memcpy(str_destino, inicio, fin - inicio);
			str_destino[fin - inicio] = '\0';
			respuesta = true;
		}
	}

	return respuesta;
}

void FastSIM::RxInit()
{
	rx_state = RX_NOT_STARTED;
	prev_time = millis();
	comm_buf[0] = 0x00; // end of string
	p_comm_buf = &comm_buf[0];
	comm_buf_len = 0;
	Serial1.flush(); // erase rx circular buffer
}

byte FastSIM::IsRxFinished(void)
{
	byte num_of_bytes;
	byte ret_val = RX_NOT_FINISHED;  // default not finished

	if (rx_state == RX_NOT_STARTED)
	{
		if (!Serial1.available())
		{
			if ((unsigned long)(millis() - prev_time) >= start_reception_tmout)
			{
				comm_buf[comm_buf_len] = 0x00;
				ret_val = RX_TMOUT_ERR;
			}
		}
		else
		{
			prev_time = millis(); // init tmout for inter-character space
			rx_state = RX_ALREADY_STARTED;
		}
	}

	if (rx_state == RX_ALREADY_STARTED)
	{
		num_of_bytes = Serial1.available();
		if (num_of_bytes) prev_time = millis();

		while (num_of_bytes)
		{
			num_of_bytes--;
			if (comm_buf_len < BUFFER_SIZE)
			{
				*p_comm_buf = Serial1.read();
				p_comm_buf++;
				comm_buf_len++;
				comm_buf[comm_buf_len] = 0x00;
			}
			else
			{
				Serial1.read();
			}
		}

		if ((unsigned long)(millis() - prev_time) >= interchar_tmout)
		{
			comm_buf[comm_buf_len] = 0x00;  // for sure finish string again
											// but it is not necessary
			ret_val = RX_FINISHED;
		}
	}

	return (ret_val);
}

bool FastSIM::SendSMS(char *number_str, char *message_str)
{
	bool retvalue = false;
	char end[2];
	end[0] = 0x1a;
	end[1] = '\0';
	
	switch(send_status)
	{
		case NOT_SEND: 	Serial1.print("AT+CMGS=\"");
						Serial1.print(number_str);
						Serial1.println("\"");
						RxInit();
						sim_status = SMS_SEND;
						send_status = WAIT_PROMPT;
						break;
		case WAIT_PROMPT:	if(IsRxFinished() == RX_FINISHED)
							{
								if(IsStringReceived(">"))	
								{
									Serial1.print(message_str);
									Serial1.println(end);
									RxInit();
									send_status = WAIT_CMGS;
								}
							}
							break;
		case WAIT_CMGS:	if(IsRxFinished() == RX_FINISHED)
						{
							if(IsStringReceived("+CMGS"))	
							{
								Serial1.print(message_str);
								Serial1.println(end);
								RxInit();
								send_status = NOT_SEND;
								sim_status = IDLE;
								retvalue = true;
							}
						}
						break;
		default: break;
	}
	return retvalue;
}

// Recibe <CR><LF>+CMT: "+543516510632","Luciano","18/10/09,13:01:48-12"<CR><LF>.... mensaje ....<CR><LF>
void FastSIM::GetSMS(char *phone_number, char *SMS_text, byte max_SMS_len)
{
	char *p_char;
	char *p_char1;
	byte len;
	phone_number[0] = 0;

	// Primero extraigo el numero de telefono
	p_char = strchr((char *)(comm_buf), '"');
	p_char1 = p_char + 1; // we are on the first phone number character
	p_char = strchr((char *)(p_char1), '"');
	if (p_char != NULL)
	{
		*p_char = 0; // end of string
		strcpy(phone_number, (char *)(p_char1));
	}

	// Ahora el texto
	p_char = strchr(p_char + 1, LF);  // find <LF>
	if (p_char != NULL)
	{
		p_char++; // now we are on the first SMS character 
		p_char1 = strchr((char *)(p_char), CR );
		if (p_char1 != NULL)
		{
			*p_char1 = 0;
		}

		len = strlen(p_char);

		if (len < MAX_SMS_LEN)
		{
			strcpy(SMS_text, (char *)(p_char));
		}
		else
		{
			memcpy(SMS_text, (char *)(p_char), (max_SMS_len - 1));
			SMS_text[max_SMS_len] = 0; // finish string
		}
	}

	RxInit();
	sim_status = IDLE;
}

