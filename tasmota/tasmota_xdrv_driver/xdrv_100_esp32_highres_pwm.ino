#ifdef USE_HIGHRES_PWM
#define XDRV_100           100
#ifdef ESP32

#warning **** XDRV_100_esp32_highres_pwm driver is included... ****


/*********************************************************************************************\
 * Variables & Commands
\*********************************************************************************************/

// This variable will be set to true after initialization
bool initSuccess = false;

char * payload = nullptr;
size_t payload_size = 100;
char * topic = nullptr;
size_t topic_size = 30;

bool fade_ended = true;  // status of LEDC fade
bool fade_on = true;

void ARDUINO_ISR_ATTR LED_FADE_ISR() {
  fade_ended = true;
}

#define D_CMND_HRPWMHELP      "HrPwmHelp"
#define D_CMND_HRPWM          "HrPwm"
#define D_CMND_HRPWMFREQUENCY "HrPwmFrequency"
#define D_CMND_HRPWMRANGE     "HrPwmRange"

#define HRPWM_PIN 27
#define LEDC_FREQ 4883
#define LEDC_RES 14

#define LEDC_START_DUTY 15236
#define LEDC_MAX_DUTY 16383


const char kPwmCommands[] PROGMEM = "|"
  D_CMND_HRPWMHELP "|"  D_CMND_HRPWM "|" D_CMND_HRPWMFREQUENCY "|" D_CMND_HRPWMRANGE ;

void (* const PwmCommand[])(void) PROGMEM = {
  &CmndHrPwmHelp, &CmndHrPwm, &CmndHrPwmfrequency, &CmndHrPwmrange };

/*********************************************************************************************\
 * My IoT Device Functions
\*********************************************************************************************/
void CmndHrPwmHelp(void) {
  ResponseCmndDone();
}

void CmndHrPwm(void) {
  AddLog(LOG_LEVEL_INFO, PSTR("PWM Cmnd"));
  uint32_t curr_duty = ledcRead(HRPWM_PIN);
  if ((1 == XdrvMailbox.payload) || ((XdrvMailbox.payload >= 0) && (XdrvMailbox.payload < LEDC_MAX_DUTY+1))) {
    uint32_t hrpwmduty = XdrvMailbox.payload;  // 0 - 16383
    if (hrpwmduty != curr_duty) {  // On ESP32 this prevents loss of duty state
      ledcWrite(HRPWM_PIN, hrpwmduty);
      curr_duty = hrpwmduty;
    }
  }
  ResponseCmndNumber(curr_duty);
}

void CmndHrPwmfrequency(void) {
  AddLog(LOG_LEVEL_INFO, PSTR("PWM Frequency Cmnd"));
  ResponseCmndDone();
}

void CmndHrPwmrange(void) {
  AddLog(LOG_LEVEL_INFO, PSTR("PWM Range Cmnd"));
  ResponseCmndDone();
}

/*********************************************************************************************\
 * Tasmota Functions
\*********************************************************************************************/

void HrPwmInit()
{
ledcAttach(HRPWM_PIN, LEDC_FREQ, LEDC_RES);
  initSuccess = true;

  //AddLog(LOG_LEVEL_INFO, PSTR("My Project init is successful..."));

}

void HrPwmProcessing(void)
{
/* 
  int duty = ledcRead(HRPWM_PIN);
   if (fade_ended) {

    //AddLog(LOG_LEVEL_INFO, PSTR("Fade ended"));
    //fade_ended = false;
    if (fade_on) {
      //ledcFadeWithInterrupt(HRPWM_PIN, LEDC_START_DUTY, LEDC_TARGET_DUTY, LEDC_FADE_TIME, LED_FADE_ISR);
      ledcWrite(HRPWM_PIN, duty+4);
      if (duty >= LEDC_TARGET_DUTY) {
        fade_on = false;
      }
      //AddLog(LOG_LEVEL_INFO, PSTR("Fade on"));
      //fade_on = false;
      // Wait for fade to end
      //delay(LEDC_FADE_TIME+2000);
    } else {
      // Setup and start fade off led and use ISR (duty from 4095 to 0)
      //ledcFadeWithInterrupt(HRPWM_PIN, LEDC_TARGET_DUTY, LEDC_START_DUTY, LEDC_FADE_TIME, LED_FADE_ISR);
      ledcWrite(HRPWM_PIN, duty-4);
      if (duty <= LEDC_START_DUTY) {
        fade_on = true;
      }
      //AddLog(LOG_LEVEL_INFO, PSTR("Fade off"));
      //fade_on = true;
      //AddLog(LOG_LEVEL_INFO, PSTR("Help Cmnd %d"), att);
    }
  }
 */
}




/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
bool Xdrv100(uint32_t function)
{


  bool result = false;

  if (FUNC_INIT == function) {
    HrPwmInit();
  }
  else if (initSuccess) {

    switch (function) {
      // Select suitable interval for polling your function
      case FUNC_EVERY_SECOND:
//    case FUNC_EVERY_250_MSECOND:
//    case FUNC_EVERY_200_MSECOND:
//      case FUNC_EVERY_100_MSECOND:
        HrPwmProcessing();
        break;

      // Command support
      case FUNC_COMMAND:
        AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("Calling My Project Command..."));
        result = DecodeCommand(kPwmCommands, PwmCommand);
        break;

    }

  }

  return result;
}


#endif  // ESP32
#endif  // USE_HIGHRES_PWM
