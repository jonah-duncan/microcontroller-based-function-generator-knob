/*****************************************************************************************
* EECE444 Lab 3 Main

* 02/09/2023 Blake Conner
* 02/09/2023 Mitchell Prentice
* 02/09/2023 Jonah Duncan
* 02/09/2023 Taylor Inman
*****************************************************************************************/
#include "os.h"
#include "app_cfg.h"
#include "MCUType.h"
#include "K65TWR_ClkCfg.h"
#include "K65TWR_GPIO.h"
#include "LcdLayered.h"
#include "uCOSKey.h"
#include "Display.h"
#include "WaveGen.h"
#include "Memory.h"

/*****************************************************************************************
* Macros & Defines
*****************************************************************************************/
#define FTM1_TOF_DIR (FTM1->QDCTRL & FTM_QDCTRL_TOFDIR_MASK)
#define FTM1_TOF_SET (FTM1->SC & FTM_SC_TOF_MASK)
#define FTM1_TOF_CLR() (FTM1->SC &= ~FTM_SC_TOF_MASK)
#define MIN_ROT_CNT 1U
#define MAX_ROT_CNT 20U
#define CCW 0U
#define CLR 0U

/*****************************************************************************************
* Allocate task control blocks.
*****************************************************************************************/
static OS_TCB appTaskStartTCB;
static OS_TCB appStateCntrlTaskTCB;
static OS_TCB appSendPendTaskTCB;

/*****************************************************************************************
* Allocate task stack space.
*****************************************************************************************/
static CPU_STK appTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static CPU_STK appStateCntrlTaskStk[APP_CFG_STATE_CNTRL_TASK_STK_SIZE];
static CPU_STK appSendPendTaskStk[APP_CFG_SEND_PEND_TASK_STK_SIZE];

/*****************************************************************************************
* Task Function Prototypes
*   - Private if in the same module as startup task. Otherwise public.
*****************************************************************************************/
static void appTaskStart(void *p_arg);
static void appStateCntrlTask(void *p_arg);
static void appSendPendTask(void *p_arg);

/*****************************************************************************************
* All other declarations.
*****************************************************************************************/
static INT16U appRotCntSine;
static INT16U appRotCntPulse;
static INT8U rotChange = 0;

typedef enum {SINEWAVE,PULSETRAIN} APP_STATES_T;                                //Defining the states for the state machine
static APP_STATES_T appState;

typedef struct{                                                                 //Create the buffer used to pass count
    OS_SEM flag;
}APP_STATE_CNTRL;
static APP_STATE_CNTRL appStateCntrl;

static void AppRotInit(void);

void FTM1_IRQHandler(void);

void AppDefaultSettings(INT8U dfault);

static INT16U defltFreq = 1000;
static INT16U defltAmp = 10;

static INT16U currentVal = 0;

static INT16U sineFreq = 0;
static INT16U pulseFreq = 0;
static INT16U sinRot = 0;
static INT16U pulseRot = 0;

static INT8U kPress = 0;

/*****************************************************************************************
* main()
* Creates start task.
*****************************************************************************************/
void main(void) {

    OS_ERR  os_err;

    K65TWR_BootClock();
    CPU_IntDis();                                                               //Disable all interrupts, OS will enable them

    OSInit(&os_err);                                                            //Initialize uC/OS-III

    OSTaskCreate(&appTaskStartTCB,                                              //Address of TCB assigned to task
                 "Start Task",                                                  //Name you want to give the task
                 appTaskStart,                                                  //Address of the task itself
                 (void *) 0,                                                    //p_arg is not used so null ptr
                 APP_CFG_TASK_START_PRIO,                                       //Priority you assign to the task
                 &appTaskStartStk[0],                                           //Base address of task s stack
                 (APP_CFG_TASK_START_STK_SIZE/10u),                             //Watermark limit for stack growth
                 APP_CFG_TASK_START_STK_SIZE,                                   //Stack size
                 0,                                                             //Size of task message queue
                 0,                                                             //Time quanta for round robin
                 (void *) 0,                                                    //Extension pointer is not used
                 (OS_OPT_TASK_NONE),                                            //Options
                 &os_err);                                                      //Ptr to error code destination

    OSStart(&os_err);                                                           //Start multitasking(i.e. give control to uC/OS)

    while(1){                                                                   //Error Trap - should never get here
    }
}

/*****************************************************************************************
* STARTUP TASK
* This should run once and be deleted. Could restart everything by creating.
*****************************************************************************************/
static void appTaskStart(void *p_arg) {

    OS_ERR os_err;
    (void)p_arg;                                                                //Avoid compiler warning for unused variable

    WaveGenInit();
    LcdInit();
    KeyInit();
    GpioDBugBitsInit();
    AppRotInit();
    OS_CPU_SysTickInitFreq(SYSTEM_CLOCK);
    MemInit();

    OSTaskCreate(&appStateCntrlTaskTCB,
                "State Control Task",
				appStateCntrlTask,
                (void *) 0,
                APP_CFG_STATE_CNTRL_TASK_PRIO,
                &appStateCntrlTaskStk[0],
                (APP_CFG_STATE_CNTRL_TASK_STK_SIZE / 10u),
                APP_CFG_STATE_CNTRL_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_NONE),
                &os_err);

    OSTaskCreate(&appSendPendTaskTCB,
				"Send Pend Task",
				appSendPendTask,
				(void *) 0,
				APP_CFG_SEND_PEND_TASK_PRIO,
				&appSendPendTaskStk[0],
				(APP_CFG_SEND_PEND_TASK_STK_SIZE / 10u),
				APP_CFG_SEND_PEND_TASK_STK_SIZE,
				0,
				0,
				(void *) 0,
				(OS_OPT_TASK_NONE),
				&os_err);

    OSSemCreate(&(appStateCntrl.flag),"Run State Control",0,&os_err);

    OSTaskDel((OS_TCB *)0, &os_err);
}

/*****************************************************************************************
* appStateCntrlTask()
* Controls the overall state of our project based off of the user key input. Also,
* Passes the current key values to LCD module as well as the Sine/ Pulse train Module.
* Created by: Mitchell Prentice 03/01/2023
*****************************************************************************************/
static void appStateCntrlTask(void *p_arg){

    OS_ERR os_err;
    INT8U a_key = DC1;
    INT8U b_key = DC2;
    INT8U nine_key = 0x39;
    INT8U zero_key = 0x30;
    INT8U d_key = DC4;
    INT32U count = 0;
    INT8U number_size = 0;
    INT8U valid = 0xff;
    INT8U checksum = 0;
    INT8U invalid = 0;

    checksum = GetCheckSum(0);
	if(checksum == valid){
		AppDefaultSettings(checksum);
		count = 0;
		number_size = 0;
	}else{
		LcdDispString(1,1, LCD_LAYER_CURR_INPUT,"INVALID CS\0");
		OSTimeDly(2000,OS_OPT_TIME_DLY,&os_err);
		AppDefaultSettings(checksum);
		count = 0;
		number_size = 0;
	}

	if(appState == SINEWAVE){
		DispUpdateDcVol(sinRot,SINEWAVE,0);
		DispUpdateFreq(sineFreq,0);
		DispUpdateMode(SINEWAVE, 0);

		Spi2MemWrite16(SAV_SINE_FREQ, sineFreq);
		Spi2MemWrite16(SAV_SINE_AMP, sinRot);
		Spi2MemWrite16(SAV_STATE, (INT16U)appState);
        GetCheckSum(1);

	}else if(appState == PULSETRAIN){
		DispUpdateDcVol((pulseRot*5),PULSETRAIN,0);
		DispUpdateFreq(pulseFreq,0);
		DispUpdateMode(PULSETRAIN,0);

		Spi2MemWrite16(SAV_PULSE_FREQ, pulseFreq);
        Spi2MemWrite16(SAV_PULSE_AMP, pulseRot);
        Spi2MemWrite16(SAV_STATE, (INT16U)appState);
		GetCheckSum(1);
	}

    (void)p_arg;

    while(1){
        DB1_TURN_OFF();
        OSSemPend(&(appStateCntrl.flag),0,OS_OPT_PEND_BLOCKING,(CPU_TS *)0,&os_err);
        DB1_TURN_ON();

        if(rotChange == 1){
            Spi2MemWrite16(SAV_SINE_AMP, appRotCntSine);
            Spi2MemWrite16(SAV_PULSE_AMP, appRotCntPulse);
            GetCheckSum(1);
            rotChange = 0;
        }else{
            //Do Nothing
        }
/*******************************************************************************
 * Section used for Backspace key press
 ******************************************************************************/
    	if(kPress == '*'){														//BACK SPACE
    		number_size = number_size - 1;
    		if(number_size == 0){
    			count = 0;
    			currentVal  = currentVal/10;
    			DispUpdateCurrInput(currentVal,0);
    		}else{
    			currentVal = currentVal/10;
    			DispUpdateCurrInput(currentVal,0);
    		}

/*******************************************************************************
 * Section used for Enter key press
 ******************************************************************************/

    	}else if (kPress == '#'){ 												//ENTER KEY
    		if(currentVal < 10){												//Invalid Hz entry
    			currentVal = 0;
    			number_size = 0;
    			count = 0;
    			DispUpdateCurrInput(currentVal,(OS_ERR *) 1);

    		}else if(currentVal > 10000){										//Invalid Hz entry
    			currentVal = 0;
    			number_size = 0;
    			count = 0;
    			DispUpdateCurrInput(currentVal,(OS_ERR *) 1);

    		}else if(number_size > 5){											//number_size too great
				number_size = 0;
				count = 0;
				currentVal = 0;
				DispUpdateCurrInput(currentVal,(OS_ERR *) 1);

    		}else{																// valid Hz entry
    			WGFreqSet(currentVal);											//Sends frequency value to wavegen
				DispUpdateFreq(currentVal,0);

				if(appState == SINEWAVE){
				    WGRotSet(appRotCntSine);
					WGOutputUpdate(SINEWAVE);									//sends state to wavegen every frequency update
					sineFreq = currentVal;
					Spi2MemWrite16(SAV_SINE_FREQ, sineFreq);                    //writes to EEPROM
					GetCheckSum(1);
				}else if(appState == PULSETRAIN){
				    WGRotSet(appRotCntPulse);
					WGOutputUpdate(PULSETRAIN);									//sends state to wavegen every frequency update
					pulseFreq = currentVal;
					Spi2MemWrite16(SAV_PULSE_FREQ, pulseFreq);                  //writes to EEPROM
					GetCheckSum(1);
				}else{
					//Do Nothing
				}

				number_size = 0;
				count = 0;
				currentVal = 0;
				DispUpdateCurrInput(currentVal,0);
    		}

/*******************************************************************************
 * Section used for number presses ranging from 0:9
 ******************************************************************************/
    	}else if(kPress <= nine_key && kPress >= zero_key){					    // valid number entry
    		number_size = number_size + 1;
    		kPress = kPress - zero_key;
    		if(count == 0){														// First key entry
    			currentVal = kPress;
    			count = count +1;
    			DispUpdateCurrInput(currentVal,0);

    		}else{																// following key entries
    			currentVal = (currentVal*10 + kPress);
    			DispUpdateCurrInput(currentVal,0);

    		}

/*******************************************************************************
 * Section used for number presses A, B, D
 ******************************************************************************/
    	}else if(kPress == a_key){												//Sinewave state change
    		appState = SINEWAVE;
    		Spi2MemWrite16(SAV_STATE,(INT16U)appState);                         //writes to EEPROM
    		GetCheckSum(1);
    		count = 0;
    		number_size = 0;

    	}else if(kPress == b_key){												// Pulse Train state change
    		appState = PULSETRAIN;
    		Spi2MemWrite16(SAV_STATE,(INT16U)appState);                         //writes to EEPROM
    		GetCheckSum(1);
    		count = 0;
    		number_size = 0;

    	}else if(kPress == d_key){												//Manual reset by user

    		AppDefaultSettings(invalid);
    		count = 0;
    		number_size = 0;
    		DispUpdateCurrInput(currentVal,0);

    	}else{
    		//do nothing
    	}
    	kPress = 0;
/*******************************************************************************
 * State machine for display values
 ******************************************************************************/
    	switch(appState){
    	case SINEWAVE:
    		DispUpdateDcVol(appRotCntSine,SINEWAVE,0);
    		DispUpdateFreq(sineFreq,0);
    		DispUpdateMode(appState, 0);
		break;
    	case PULSETRAIN:
    		DispUpdateDcVol((appRotCntPulse*5),PULSETRAIN,0);
    		DispUpdateFreq(pulseFreq,0);
    		DispUpdateMode(appState,0);
		break;
    	default:
    		appState = SINEWAVE;
    	break;
    	}
    }
}

/*******************************************************************************

 * Default can be used to set initial variable components to varying destinations
 * and display those variables:
 *          State,
 *          Sinewave Frequency,
 *          Pulsetrain frequency,
 *          Sinewave Amplitude,
 *          Pulsetrain Amplitude,
 *
 * Created by: Mitchell Prentice 03/11/2023
 ******************************************************************************/
void AppDefaultSettings(INT8U dfault){
	//if default != 0xff then  we load default settings
	//if default == 0xff then  we load saved data from EEPROM
	OS_ERR os_err;
	if(dfault == 0xff){
		appState = DataGetEEPROM(SAV_STATE);

		sineFreq =  DataGetEEPROM(SAV_SINE_FREQ);
		pulseFreq = DataGetEEPROM(SAV_PULSE_FREQ);
		sinRot = DataGetEEPROM(SAV_SINE_AMP);
		pulseRot = DataGetEEPROM(SAV_PULSE_AMP);

		appRotCntPulse = pulseRot;
        appRotCntSine = sinRot;
		currentVal = 0;

        WGFreqSet(sineFreq);
        WGRotSet(sinRot);
        WGOutputUpdate(SINEWAVE);
        OSTimeDly(1,OS_OPT_TIME_DLY,&os_err);
        WGFreqSet(pulseFreq);
        WGRotSet(pulseRot);
        WGOutputUpdate(PULSETRAIN);

		switch (appState){
		case 0:
		    WGFreqSet(sineFreq);
		    WGRotSet(sinRot);
		    DispUpdateDcVol(sinRot,SINEWAVE,0);
            DispUpdateFreq(sineFreq,0);
		    break;
		case 1:
		    WGFreqSet(pulseFreq);
		    WGRotSet(pulseRot);
            DispUpdateDcVol((pulseRot*5),PULSETRAIN,0);
            DispUpdateFreq(pulseFreq,0);
            break;
		default:
		    DispUpdateDcVol(sinRot,SINEWAVE,0);
            DispUpdateFreq(sineFreq,0);
		}

        DispUpdateMode(appState, 0);
        DispUpdateCurrInput(currentVal,0);


		if(appState == SINEWAVE){
			appRotCntSine = sinRot;
			Spi2MemWrite16(SAV_SINE_AMP, sinRot);
			GetCheckSum(1);
		}else if(appState == PULSETRAIN){
			appRotCntPulse = pulseRot;
			Spi2MemWrite16(SAV_PULSE_AMP, pulseRot);
			GetCheckSum(1);
		}else{
			//do nothing
		}

		GetCheckSum(1);

	}else if(dfault != 0xff){

		appState = SINEWAVE;

		appRotCntPulse = defltAmp;
		appRotCntSine = defltAmp;

		sinRot = defltAmp;
		pulseRot = defltAmp;
		sineFreq = defltFreq;
		pulseFreq = defltFreq;
		currentVal = 0;

		WGFreqSet(sineFreq);
		WGRotSet(defltAmp);
		WGOutputUpdate(SINEWAVE);
        OSTimeDly(1,OS_OPT_TIME_DLY,&os_err);
        WGOutputUpdate(PULSETRAIN);

		DispUpdateDcVol(sinRot,SINEWAVE,0);
        DispUpdateFreq(sineFreq,0);
        DispUpdateMode(SINEWAVE, 0);
		DispUpdateCurrInput(currentVal,0);

		Spi2MemWrite16(SAV_SINE_FREQ,defltFreq);
		Spi2MemWrite16(SAV_SINE_AMP,defltAmp);
		Spi2MemWrite16(SAV_PULSE_FREQ,defltFreq);
		Spi2MemWrite16(SAV_PULSE_AMP, defltAmp);
		Spi2MemWrite16(SAV_STATE,(INT16U)appState);
		GetCheckSum(1);

	}else{
		//do nothing
	}
}

/*******************************************************************************
* appSendPendTask()
* posts to a semaphore when key is pressed.
*
* Created by Blake Conner 03/12/2023
*******************************************************************************/
static void appSendPendTask(void *p_arg){

	OS_ERR os_err;
	(void)p_arg;

	while(1){
		kPress = KeyPend(0,&os_err);
		OSSemPost(&(appStateCntrl.flag),OS_OPT_POST_1,&os_err);
	}
}

/*******************************************************************************
* AppRotInit() - Initializes and configures quadrature encoder.
*
* Jonah Duncan, 03/08/2023 Created
*******************************************************************************/
static void AppRotInit(void){

    /* Enable FTM1 and PORTA clock gates */
    SIM->SCGC6 |= SIM_SCGC6_FTM1(1);
    SIM->SCGC5 |= SIM_SCGC5_PORTA(1);

    /* MUX PTA[8,9] to FTM1_QD_PH[A,B]. [EN,SEL] PTA[8,9] pullup resistors. */
    PORTA->PCR[8] = PORT_PCR_MUX(6)|PORT_PCR_PS(1)|PORT_PCR_PE(1);
    PORTA->PCR[9] = PORT_PCR_MUX(6)|PORT_PCR_PS(1)|PORT_PCR_PE(1);

    /* MOD = ‘6 cnts per interrupt’ - 1 = 5 */
    FTM1->MOD = FTM_MOD_MOD(5);

    /* Write to count to reset count */
    FTM1->CNT = FTM_CNT_COUNT(0);

    /* Set PH[A,B] filter values to maximum */
    FTM1->FILTER = FTM_FILTER_CH0FVAL(0x0F)|FTM_FILTER_CH1FVAL(0x0F);

    /* Quadrature [filter,mode] configuration and enable */
    FTM1->QDCTRL = FTM_QDCTRL_PHAPOL(0)|FTM_QDCTRL_PHBPOL(0)|FTM_QDCTRL_PHAFLTREN(1)|FTM_QDCTRL_PHBFLTREN(1)|FTM_QDCTRL_QUADMODE(0)|FTM_QDCTRL_QUADEN(1);

    /* Enable overflow interrupt */
    FTM1->SC = FTM_SC_TOIE(1);

    /* Enable overflow interrupt at NVIC (disable arbitrary since real-time) */
    FTM1_TOF_CLR();
    NVIC_ClearPendingIRQ(FTM1_IRQn);
    NVIC_EnableIRQ(FTM1_IRQn);
}

/*******************************************************************************
* FTM1_IRQHandler() - Handles FTM1 TOF interrupts, quadrature decoder turns.
*
* Jonah Duncan, 03/08/2023 Created
*******************************************************************************/
void FTM1_IRQHandler(void){

	OS_ERR os_err;

    if(appState != PULSETRAIN){
        if(FTM1_TOF_SET != CLR){
            FTM1_TOF_CLR();

            if(FTM1_TOF_DIR != CCW){
                if(appRotCntSine < MAX_ROT_CNT){
                    appRotCntSine++;
                }else{}
            }else{
                if(appRotCntSine >= MIN_ROT_CNT){
                    appRotCntSine--;
                }else{}
            }

            WGFreqSet(sineFreq);
            WGRotSet(appRotCntSine);
            WGOutputUpdate(appState);
            rotChange = TRUE;
    		OSSemPost(&(appStateCntrl.flag),OS_OPT_POST_1,&os_err);
    		DispUpdateDcVol(appRotCntSine, appState, &os_err);

        }else{
            /* ignore other interrupts for now */
        }

    }else{
        if(FTM1_TOF_SET != CLR){
            FTM1_TOF_CLR();

            if(FTM1_TOF_DIR != CCW){
                if(appRotCntPulse < MAX_ROT_CNT){
                    appRotCntPulse++;
                }else{}
            }else{
                if(appRotCntPulse >= MIN_ROT_CNT){
                    appRotCntPulse--;
                }else{}
            }

            WGFreqSet(pulseFreq);
            WGRotSet(appRotCntPulse);
            WGOutputUpdate(appState);
            rotChange = TRUE;
    		OSSemPost(&(appStateCntrl.flag),OS_OPT_POST_1,&os_err);
    		DispUpdateDcVol((appRotCntPulse*5), appState, &os_err);

        }else{
            /* ignore other interrupts for now */
        }
    }
}
