// FastSIM.h - Clase para controlar modulos GSM por comandos AT con funciones NO Bloqueantes
// Orientada al envio y recepcion de SMS 
// Utiliza Hardware Serial1 (ATMEGA2560, 1284, 680, etc.)
// No utiliza delay

#ifndef _SIM_SMS_h
#define _SIM_SMS_h

#include "Arduino.h"

#ifndef SIM_BAUD
#define SIM_BAUD 115200
#endif

#define DEBUG_SIM

#define BUFFER_SIZE 200
#define RX_NOT_STARTED  0
#define RX_ALREADY_STARTED  1
#define MAX_SMS_LEN 100

#define start_reception_tmout 1000
#define interchar_tmout 100

#define CR      0x0D    // \r 
#define LF      0x0A    // \n 
#define END     "\r\n"
#define CTRLZ   0x1A
#define OK      "OK"
#define ERROR   "ERROR"
#define AT      "AT"   
//#define INIT_COMMAND "ATE0;+CMGF=1;+CNMI=3,2,0,0,0;+CMGDA=\"DEL ALL\";&W"
#define INIT_COMMAND "ATE0;+CMGF=1;+CNMI=3,2,0,0,0;+CMGDA=\"DEL ALL\";&W"

enum INIT_STATUS_ENUM 
{
	NOT_STARTED = 0,
	STARTING,
	SETTING,
	WAIT_OK,
	STARTED_OK,
	STARTED_ERROR
};

enum SIM_STATUS_ENUM
{
	IDLE = 0,
	SMS_RECEIVED,
	SMS_SEND,
	CHECK_OK
};

enum SEND_SMS_STATUS
{
	NOT_SEND=0,
	WAIT_PROMPT,
	WAIT_CMGS
};

enum rx_state_enum
{
	RX_NOT_FINISHED = 0,      // not finished yet
	RX_FINISHED,              // finished, some character was received
	RX_FINISHED_STR_RECV,     // finished and expected string received
	RX_FINISHED_STR_NOT_RECV, // finished, but expected string not received
	RX_TMOUT_ERR,             // finished, no character received 
							  // initial communication tmout occurred
	RX_LAST_ITEM
};

class FastSIM
{
private:
	byte rx_state;
	unsigned long prev_time;
	byte comm_buf[BUFFER_SIZE + 1];
	byte *p_comm_buf;
	byte comm_buf_len;
	byte comm_line_status;

	byte init_num_command;
	byte init_status = NOT_STARTED;
	byte prev_init_status;
	byte start_attemp = 0;
	unsigned long timeout, init_timer;
	byte sim_status = 0;
	byte send_status = NOT_SEND;
public:
	void init(void);
	byte update(void);
	byte IsStringReceived(char const *);
	bool StringFind(const char *, const char *);
	bool BuscarEntre(char *, char *, char *, char *);
	void RxInit();
	byte IsRxFinished(void);
	bool SendSMS(char *, char *);
	void GetSMS(char *, char *, byte);
	bool CheckOK(void);
};

#endif

