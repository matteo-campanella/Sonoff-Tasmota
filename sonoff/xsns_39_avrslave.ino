#ifdef USE_SPI

#define XSNS_39              39

bool initialized = false;

void AVRSlave_Init(void){
  if(initialized) return;
  digitalWrite(SS,HIGH);
  SPI.begin();
  //digitalWrite(SS,LOW);
  //uint8_t rc = SPI.transfer(0x01);
  //if (rc!=0) return;
  initialized = true;
  //digitalWrite(SS,HIGH);
}

void AVRSlave_XchangeData(void){
  digitalWrite(SS,LOW);
  for(uint8_t i=0;i<128;i++) {
    SPI.transfer(i);
  }
  digitalWrite(SS,HIGH);
}

void AVRSlave_Show(bool Json){
    char probetemp[33];
    char referencetemp[33];
    dtostrfd(10, Settings.flag2.temperature_resolution, probetemp);
    dtostrfd(11, Settings.flag2.temperature_resolution, referencetemp);

    if(Json){
        ResponseAppend_P(PSTR(",\"AVRSlave\":{\"" D_JSON_PROBETEMPERATURE "\":%s,\"" D_JSON_REFERENCETEMPERATURE "\":%s,\"" D_JSON_ERROR "\":%d}"), \
          probetemp, referencetemp, 12);
#ifdef USE_DOMOTICZ
        if (0 == tele_period) {
          DomoticzSensor(DZ_TEMP, probetemp);
        }
#endif  // USE_DOMOTICZ
#ifdef USE_KNX
        if (0 == tele_period) {
          KnxSensor(KNX_TEMPERATURE, 10);
        }
#endif  // USE_KNX
    } else {
#ifdef USE_WEBSERVER
        WSContentSend_PD(HTTP_SNS_TEMP, "AVRSlave", probetemp, TempUnit());
#endif  // USE_WEBSERVER
    }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns39(uint8_t function)
{
  bool result = false;

  if (initialized || function == FUNC_INIT && (1 != Settings.flashAVR)) {
    switch (function) {
      case FUNC_INIT:
        AVRSlave_Init();
        break;
      case FUNC_EVERY_SECOND:
        AVRSlave_XchangeData();
        break;
      case FUNC_JSON_APPEND:
        AVRSlave_Show(true);
        break;
  #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        AVRSlave_Show(false);
        break;
  #endif  // USE_WEBSERVER
    return result;
    }
  }
}
#endif  // USE_SPI