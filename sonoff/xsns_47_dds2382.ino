/*
  xsns_47_dds2382.ino - Eastron dds2382-Modbus energy meter support for Sonoff-Tasmota

  Copyright (C) 2019  Gennaro Tortone

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//#define USE_DDS2382
#ifdef USE_DDS2382

/*********************************************************************************************\
 * Hiking dds238-2 Modbus energy meter
 *
 * Based on: https://github.com/reaper7/SDM_Energy_Meter
\*********************************************************************************************/

#define XSNS_47             47

// can be user defined in my_user_config.h
#ifndef DDS2382_SPEED
  #define DDS2382_SPEED      9600    // default dds2382 Modbus address
#endif
// can be user defined in my_user_config.h
#ifndef DDS2382_ADDR
  #define DDS2382_ADDR       1       // default dds2382 Modbus address
#endif


#include <TasmotaSerial.h>

enum DDS2382_Error {DDS2382_ERR_NO_ERROR=0, DDS2382_ERR_CRC_ERROR, DDS2382_ERR_WRONG_BYTES, DDS2382_ERR_NOT_ENOUGHT_BYTES};

TasmotaSerial *DDS2382Serial;

uint8_t dds2382_type = 1;
//uint8_t dds2382_state = 0;

float dds2382_voltage = 0;
float dds2382_current = 0;
float dds2382_active_power = 0;
float dds2382_reactive_power = 0;
float dds2382_power_factor = 0;
float dds2382_frequency = 0;
float dds2382_energy_total = 0;
float dds2382_import_active = 0;
float dds2382_export_active = 0;

bool DDS2382_ModbusReceiveReady(void)
{
  return (DDS2382Serial->available() > 1);
}

void DDS2382_ModbusSend(uint8_t function_code, uint16_t start_address, uint16_t register_count)
{
  uint8_t frame[8];

  frame[0] = DDS2382_ADDR;
  frame[1] = function_code;
  frame[2] = (uint8_t)(start_address >> 8);
  frame[3] = (uint8_t)(start_address);
  frame[4] = (uint8_t)(register_count >> 8);
  frame[5] = (uint8_t)(register_count);

  uint16_t crc = DDS2382_calculateCRC(frame, 6);  // calculate out crc only from first 6 bytes
  frame[6] = lowByte(crc);
  frame[7] = highByte(crc);

  while (DDS2382Serial->available() > 0)  {  // read serial if any old data is available
    DDS2382Serial->read();
  }

  DDS2382Serial->flush();
  DDS2382Serial->write(frame, sizeof(frame));
}

uint8_t DDS2382_ModbusReceive(float *value)
{
  uint8_t buffer[9];

  *value = NAN;
  uint8_t len = 0;
  while (DDS2382Serial->available() > 0) {
    buffer[len++] = (uint8_t)DDS2382Serial->read();
  }

  if (len < 9) {
    return DDS2382_ERR_NOT_ENOUGHT_BYTES;
  }

  if (9 == len) {
    if (0x01 == buffer[0] && 0x04 == buffer[1] && 4 == buffer[2]) {   // check node number, op code and reply bytes count
      if((DDS2382_calculateCRC(buffer, 7)) == ((buffer[8] << 8) | buffer[7])) {  //calculate crc from first 7 bytes and compare with received crc (bytes 7 & 8)

        ((uint8_t*)value)[3] = buffer[3];
        ((uint8_t*)value)[2] = buffer[4];
        ((uint8_t*)value)[1] = buffer[5];
        ((uint8_t*)value)[0] = buffer[6];

      } else {
        return DDS2382_ERR_CRC_ERROR;
      }

    } else {
      return DDS2382_ERR_WRONG_BYTES;
    }
  }

  return DDS2382_ERR_NO_ERROR;
}

uint16_t DDS2382_calculateCRC(uint8_t *frame, uint8_t num)
{
  uint16_t crc, flag;
  crc = 0xFFFF;
  for (uint32_t i = 0; i < num; i++) {
    crc ^= frame[i];
    for (uint32_t j = 8; j; j--) {
      if ((crc & 0x0001) != 0) {        // If the LSB is set
        crc >>= 1;                      // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                          // Else LSB is not set
        crc >>= 1;                      // Just shift right
      }
    }
  }
  return crc;
}

/*********************************************************************************************/

const uint16_t dds2382_start_addresses[] {
  0x000C,   // DDS2382C_VOLTAGE  [V]
  0x000D,   // DDS2382C_CURRENT  [A]
  0x000E,   // DDS2382C_POWER    [W]
  0x000F,   // DDS2382C_REACTIVE_POWER  [VAR]
  0x0010,   // DDS2382C_POWER_FACTOR
  0x0011,   // DDS2382C_FREQUENCY  [Hz]
  0x0000,   // DDS2382C_TOTAL_ACTIVE_ENERGY  [dWh]
  0X000A, // SDM220_IMPORT_ACTIVE [dWh]
  0X0008, // SDM220_EXPORT_ACTIVE [dWh]
};

uint8_t dds2382_read_state = 0;
uint8_t dds2382_send_retry = 0;
uint8_t dds2382_nodata_count = 0;

void DDS2382250ms(void)              // Every 250 mSec
{
//  dds2382_state++;
//  if (6 == dds2382_state) {     // Every 300 mSec
//    dds2382_state = 0;

    float value = 0;
    bool data_ready = DDS2382_ModbusReceiveReady();

    if (data_ready) {
      dds2382_nodata_count = 0;
      uint8_t error = DDS2382_ModbusReceive(&value);
      if (error) {
        AddLog_P2(LOG_LEVEL_DEBUG, PSTR(D_LOG_DEBUG "dds2382 response error %d"), error);
      } else {
        switch(dds2382_read_state) {
          case 0:
            dds2382_voltage = value;
            break;

          case 1:
            dds2382_current = value;
            break;

          case 2:
            dds2382_active_power = value;
            break;

          case 3:
            dds2382_reactive_power = value;
            break;

          case 4:
            dds2382_power_factor = value;
            break;

          case 5:
            dds2382_frequency = value;
            break;

          case 6:
            dds2382_energy_total = value;
            break;

          case 7:
            dds2382_import_active = value;
            break;

          case 8:
            dds2382_export_active = value;
            break;
        } // end switch

        dds2382_read_state++;

        if (sizeof(dds2382_start_addresses)/2 == dds2382_read_state) {
          dds2382_read_state = 0;
        }
      }
    } // end data ready
    else {
      if (dds2382_nodata_count <= (1000/250) * 4) {  // max. 4 sec without data
        dds2382_nodata_count++;
      } else if (dds2382_nodata_count != 255) {
        // no data from modbus, reset values to 0
        dds2382_nodata_count = 255;
        dds2382_voltage = dds2382_current = dds2382_active_power = dds2382_reactive_power = dds2382_power_factor = dds2382_frequency = dds2382_energy_total = dds2382_import_active = dds2382_export_active = 0;
      }
    }

    if (0 == dds2382_send_retry || data_ready) {
      dds2382_send_retry = 5;
       DDS2382_ModbusSend(0x03, dds2382_start_addresses[dds2382_read_state], 2);
    } else {
      dds2382_send_retry--;
    }
//  } // end 300 ms
}

void DDS2382Init(void)
{
  dds2382_type = 0;
  if ((pin[GPIO_DDS2382_RX] < 99) && (pin[GPIO_DDS2382_TX] < 99)) {
    DDS2382Serial = new TasmotaSerial(pin[GPIO_DDS2382_RX], pin[GPIO_DDS2382_TX], 1);
    if (DDS2382Serial->begin(DDS2382_SPEED)) {
      if (DDS2382Serial->hardwareSerial()) { ClaimSerial(); }
      dds2382_type = 1;
    }
  }
}

#ifdef USE_WEBSERVER
const char HTTP_SNS_DDS2382_DATA[] PROGMEM =
  "{s}dds2382 " D_VOLTAGE "{m}%s " D_UNIT_VOLT "{e}"
  "{s}dds2382 " D_CURRENT "{m}%s " D_UNIT_AMPERE "{e}"
  "{s}dds2382 " D_POWERUSAGE_ACTIVE "{m}%s " D_UNIT_WATT "{e}"
  "{s}dds2382 " D_POWERUSAGE_REACTIVE "{m}%s " D_UNIT_VAR "{e}"
  "{s}dds2382 " D_POWER_FACTOR "{m}%s{e}"
  "{s}dds2382 " D_FREQUENCY "{m}%s " D_UNIT_HERTZ "{e}"
  "{s}dds2382 " D_ENERGY_TOTAL "{m}%s " D_UNIT_KILOWATTHOUR "{e}"
  "{s}dds2382 " D_IMPORT_ACTIVE "{m}%s " D_UNIT_KILOWATTHOUR "{e}"
  "{s}dds2382 " D_EXPORT_ACTIVE "{m}%s " D_UNIT_KILOWATTHOUR "{e}"
  ;
#endif  // USE_WEBSERVER

void DDS2382Show(bool json)
{
  char voltage[33];
  dtostrfd(dds2382_voltage,        Settings.flag2.voltage_resolution, voltage);
  char current[33];
  dtostrfd(dds2382_current,        Settings.flag2.current_resolution, current);
  char active_power[33];
  dtostrfd(dds2382_active_power,   Settings.flag2.wattage_resolution, active_power);
  char reactive_power[33];
  dtostrfd(dds2382_reactive_power, Settings.flag2.wattage_resolution, reactive_power);
  char power_factor[33];
  dtostrfd(dds2382_power_factor,   2, power_factor);
  char frequency[33];
  dtostrfd(dds2382_frequency,      Settings.flag2.frequency_resolution, frequency);
  char energy_total[33];
  dtostrfd(dds2382_energy_total,   Settings.flag2.energy_resolution, energy_total);
  char import_active[33];
  dtostrfd(dds2382_import_active,  Settings.flag2.wattage_resolution, import_active);
  char export_active[33];
  dtostrfd(dds2382_export_active,  Settings.flag2.wattage_resolution, export_active);
  if (json) {
    ResponseAppend_P(PSTR(",\"" D_RSLT_ENERGY "\":{\"" D_JSON_TOTAL "\":%s,\"" D_JSON_ACTIVE_POWERUSAGE "\":%s,\"" D_JSON_REACTIVE_POWERUSAGE "\":%s,\"" D_JSON_FREQUENCY "\":%s,\"" D_JSON_POWERFACTOR "\":%s,\"" D_JSON_VOLTAGE "\":%s,\"" D_JSON_CURRENT  "\":%s,\"" D_JSON_IMPORT_ACTIVE "\":%s,\"" D_JSON_EXPORT_ACTIVE "\":%s}"),
      energy_total, active_power, reactive_power, frequency, power_factor, voltage, current, import_active, export_active);
#ifdef USE_DOMOTICZ
    if (0 == tele_period) {
      char energy_total_chr[33];
      dtostrfd(dds2382_energy_total * 1000, 1, energy_total_chr);
      DomoticzSensor(DZ_VOLTAGE, voltage);
      DomoticzSensor(DZ_CURRENT, current);
      DomoticzSensorPowerEnergy((int)dds2382_active_power, energy_total_chr);
    }
#endif  // USE_DOMOTICZ
#ifdef USE_WEBSERVER
  } else {
    WSContentSend_PD(HTTP_SNS_DDS2382_DATA, voltage, current, active_power, reactive_power, power_factor, frequency, energy_total,import_active,export_active);
#endif  // USE_WEBSERVER
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns47(uint8_t function)
{
  bool result = false;

  if (dds2382_type) {
    switch (function) {
      case FUNC_INIT:
        DDS2382Init();
        break;
      case FUNC_EVERY_250_MSECOND:
        DDS2382250ms();
        break;
      case FUNC_JSON_APPEND:
        DDS2382Show(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        DDS2382Show(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif   // USE_DDS2382
