/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * opentx is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "../opentx.h"

#define PRIM_REQ_POWERUP    (0)
#define PRIM_REQ_VERSION    (1)
#define PRIM_CMD_DOWNLOAD   (3)
#define PRIM_DATA_WORD      (4)
#define PRIM_DATA_EOF       (5)

#define PRIM_ACK_POWERUP    (0x80)
#define PRIM_ACK_VERSION    (0x81)
#define PRIM_REQ_DATA_ADDR  (0x82)
#define PRIM_END_DOWNLOAD   (0x83)
#define PRIM_DATA_CRC_ERR   (0x84)

void setBaroAltitude(int32_t baroAltitude)
{
  // First received barometer altitude => Altitude offset
  if (!frskyData.hub.baroAltitudeOffset)
    frskyData.hub.baroAltitudeOffset = -baroAltitude;

  baroAltitude += frskyData.hub.baroAltitudeOffset;
  frskyData.hub.baroAltitude = baroAltitude;

  baroAltitude /= 100;
  if (baroAltitude > frskyData.hub.maxAltitude)
    frskyData.hub.maxAltitude = baroAltitude;
  if (baroAltitude < frskyData.hub.minAltitude)
    frskyData.hub.minAltitude = baroAltitude;
}

void processHubPacket(uint8_t id, uint16_t value)
{
  if (id > FRSKY_LAST_ID)
    return;

  if (id == GPS_LAT_BP_ID) {
    if (value)
      frskyData.hub.gpsFix = 1;
    else if (frskyData.hub.gpsFix > 0 && frskyData.hub.gpsLatitude_bp > 1)
      frskyData.hub.gpsFix = 0;
  }
  else if (id == GPS_LONG_BP_ID) {
    if (value)
      frskyData.hub.gpsFix = 1;
    else if (frskyData.hub.gpsFix > 0 && frskyData.hub.gpsLongitude_bp > 1)
      frskyData.hub.gpsFix = 0;
  }

  if (id == GPS_ALT_BP_ID || (id >= GPS_ALT_AP_ID && id <= GPS_LAT_NS_ID && id != BARO_ALT_BP_ID && id != BARO_ALT_AP_ID)) {
    // if we don't have a fix, we may discard the value
    if (frskyData.hub.gpsFix <= 0)
      return;
  }

  ((uint16_t*)&frskyData.hub)[id] = value;

  switch (id) {

    case RPM_ID:
      frskyData.hub.rpm *= (uint8_t)60/(g_model.frsky.blades+2);
      if (frskyData.hub.rpm > frskyData.hub.maxRpm)
        frskyData.hub.maxRpm = frskyData.hub.rpm;
      break;
      
    case TEMP1_ID:
      if (frskyData.hub.temperature1 > frskyData.hub.maxTemperature1)
        frskyData.hub.maxTemperature1 = frskyData.hub.temperature1;
      break;

    case TEMP2_ID:
      if (frskyData.hub.temperature2 > frskyData.hub.maxTemperature2)
        frskyData.hub.maxTemperature2 = frskyData.hub.temperature2;
      break;

    case CURRENT_ID:
      if ((int16_t)frskyData.hub.current > 0 && ((int16_t)frskyData.hub.current + g_model.frsky.fasOffset) > 0)
        frskyData.hub.current += g_model.frsky.fasOffset;
      else
        frskyData.hub.current = 0;
      if (frskyData.hub.current > frskyData.hub.maxCurrent)
        frskyData.hub.maxCurrent = frskyData.hub.current;
      break;

    case VOLTS_AP_ID:
#if defined(FAS_BSS)
      frskyData.hub.vfas = (frskyData.hub.volts_bp * 10 + frskyData.hub.volts_ap);
#else
      frskyData.hub.vfas = ((frskyData.hub.volts_bp * 100 + frskyData.hub.volts_ap * 10) * 21) / 110;
#endif
      if (!frskyData.hub.minVfas || frskyData.hub.vfas < frskyData.hub.minVfas)
        frskyData.hub.minVfas = frskyData.hub.vfas;
      break;

    case BARO_ALT_AP_ID:
      if (frskyData.hub.baroAltitude_ap > 9)
        frskyData.hub.varioHighPrecision = true;
      if (!frskyData.hub.varioHighPrecision)
        frskyData.hub.baroAltitude_ap *= 10;
      setBaroAltitude((int32_t)100 * frskyData.hub.baroAltitude_bp + (frskyData.hub.baroAltitude_bp >= 0 ? frskyData.hub.baroAltitude_ap : -frskyData.hub.baroAltitude_ap));
      break;

    case GPS_ALT_AP_ID:
    {
      frskyData.hub.gpsAltitude = (frskyData.hub.gpsAltitude_bp * 100) + frskyData.hub.gpsAltitude_ap;
      if (!frskyData.hub.gpsAltitudeOffset)
        frskyData.hub.gpsAltitudeOffset = -frskyData.hub.gpsAltitude;
      if (!frskyData.hub.baroAltitudeOffset) {
        int altitude = TELEMETRY_RELATIVE_GPS_ALT_BP;
        if (altitude > frskyData.hub.maxAltitude)
          frskyData.hub.maxAltitude = altitude;
        if (altitude < frskyData.hub.minAltitude)
          frskyData.hub.minAltitude = altitude;
      }
      if (!frskyData.hub.pilotLatitude && !frskyData.hub.pilotLongitude) {
        // First received GPS position => Pilot GPS position
        getGpsPilotPosition();
      }
      else if (frskyData.hub.gpsDistNeeded || g_menuStack[g_menuStackPtr] == menuTelemetryFrsky) {
        getGpsDistance();
      }
      break;
    }

    case GPS_SPEED_BP_ID:
      // Speed => Max speed
      if (frskyData.hub.gpsSpeed_bp > frskyData.hub.maxGpsSpeed)
        frskyData.hub.maxGpsSpeed = frskyData.hub.gpsSpeed_bp;
      break;

    case VOLTS_ID:
      frskyUpdateCells();
      break;

    case GPS_HOUR_MIN_ID:
      frskyData.hub.hour = ((uint8_t)(frskyData.hub.hour + g_eeGeneral.timezone + 24)) % 24;
      break;

    case ACCEL_X_ID:
    case ACCEL_Y_ID:
    case ACCEL_Z_ID:
      ((int16_t*)(&frskyData.hub))[id] /= 10;
      break;

  }
}

bool checkSportPacket(uint8_t *packet)
{
  short crc = 0;
  for (int i=1; i<FRSKY_SPORT_PACKET_SIZE; ++i) {
    crc += packet[i]; // 0-1FE
    crc += crc >> 8;  // 0-1FF
    crc &= 0x00ff;    // 0-FF
  }
  // TRACE("crc: 0x%02x", crc);
  return (crc == 0x00ff);
}

#define SPORT_DATA_U8(packet)   (packet[4])
#define SPORT_DATA_S32(packet)  (*((int32_t *)(packet+4)))
#define SPORT_DATA_U32(packet)  (*((uint32_t *)(packet+4)))
#define HUB_DATA_U16(packet)    (*((uint16_t *)(packet+4)))

enum SportUpdateState {
  SPORT_IDLE,
  SPORT_POWERUP_REQ,
  SPORT_POWERUP_ACK,
  SPORT_VERSION_REQ,
  SPORT_VERSION_ACK,
  SPORT_DATA_TRANSFER,
  SPORT_DATA_REQ,
  SPORT_COMPLETE,
  SPORT_FAIL
};

uint8_t  sportUpdateState = SPORT_IDLE;
uint32_t sportUpdateAddr = 0;
bool intPwr, extPwr;

void processSportUpdatePacket(uint8_t *packet)
{
  if (packet[0]==0x5E && packet[1]==0x50) {
    switch (packet[2]) {
      case PRIM_ACK_POWERUP :
        if (sportUpdateState == SPORT_POWERUP_REQ) {
          sportUpdateState = SPORT_POWERUP_ACK;
        }
        break;

      case PRIM_ACK_VERSION:
        if (sportUpdateState == SPORT_VERSION_REQ) {
          sportUpdateState = SPORT_VERSION_ACK;
          // SportVersion[0] = packet[3] ;
          // SportVersion[1] = packet[4] ;
          // SportVersion[2] = packet[5] ;
          // SportVersion[3] = packet[6] ;
          // SportVerValid = 1 ;
        }
        break;

      case PRIM_REQ_DATA_ADDR :
        if (sportUpdateState == SPORT_DATA_TRANSFER) {
          sportUpdateAddr = *((uint32_t *)(&packet[3]));
          sportUpdateState = SPORT_DATA_REQ;
        }
        break;

      case PRIM_END_DOWNLOAD :
        sportUpdateState = SPORT_COMPLETE ;
        break;
				
      case PRIM_DATA_CRC_ERR :
        sportUpdateState = SPORT_FAIL ;
        break;
    }
  }
}

void frskySportProcessPacket(uint8_t *packet)
{
  uint8_t  dataId = packet[0];
  uint8_t  prim   = packet[1];
  uint16_t appId  = *((uint16_t *)(packet+2));

#if defined(PCBTARANIS) && !defined(SIMU)
  if (sportUpdateState != SPORT_IDLE) {
    processSportUpdatePacket(packet);	// Uses different chksum
    return;
  }
#endif

  if (!checkSportPacket(packet)) {
    TRACE("frskySportProcessPacket(): checksum error ");
    DUMP(packet, FRSKY_SPORT_PACKET_SIZE);
    return;
  }

  // TRACE("frskySportProcessPacket(): ");
  // DUMP(packet, FRSKY_SPORT_PACKET_SIZE);

  switch (prim)
  {
    case DATA_FRAME:

      if (appId == RSSI_ID) {
        frskyStreaming = FRSKY_TIMEOUT10ms; // reset counter only if valid frsky packets are being detected
        link_counter += 256 / FRSKY_SPORT_AVERAGING;
        frskyData.rssi[0].set(SPORT_DATA_U8(packet));
      }
#if defined(PCBTARANIS) && defined(REVPLUS)
      if (appId == XJT_VERSION_ID) {
        frskyData.xjtVersion = HUB_DATA_U16(packet);
        if (!IS_VALID_XJT_VERSION()) {
          frskyData.swr.set(0x00);
        }
      } 
      else if (appId == SWR_ID) {
        if (IS_VALID_XJT_VERSION())
          frskyData.swr.set(SPORT_DATA_U8(packet));
        else
          frskyData.swr.set(0x00);
      }
#else
      if (appId == XJT_VERSION_ID) {
        frskyData.xjtVersion = HUB_DATA_U16(packet);
      }
      else if (appId == SWR_ID) {
        frskyData.swr.set(SPORT_DATA_U8(packet));
      }
#endif
      else if (frskyData.rssi[0].value > 0) {
        if (appId == ADC1_ID || appId == ADC2_ID) {
          // A1/A2 of DxR receivers
          uint8_t idx = appId - ADC1_ID;
          frskyData.analog[idx].set(SPORT_DATA_U8(packet), g_model.frsky.channels[idx].type);
  #if defined(VARIO)
          uint8_t varioSource = g_model.frsky.varioSource - VARIO_SOURCE_A1;
          if (varioSource == appId-ADC1_ID) {
            frskyData.hub.varioSpeed = applyChannelRatio(varioSource, frskyData.analog[varioSource].value);
          }
  #endif
        }
        else if (appId == BATT_ID) {
          frskyData.analog[TELEM_ANA_A1].set(SPORT_DATA_U8(packet), UNIT_VOLTS);
        }
        else if ((appId >> 8) == 0) {
          // The old FrSky IDs
          uint8_t  id = (uint8_t)appId;
          uint16_t value = HUB_DATA_U16(packet);
          processHubPacket(id, value);
        }
        else if (appId >= T1_FIRST_ID && appId <= T1_LAST_ID) {
          frskyData.hub.temperature1 = SPORT_DATA_S32(packet);
          if (frskyData.hub.temperature1 > frskyData.hub.maxTemperature1)
            frskyData.hub.maxTemperature1 = frskyData.hub.temperature1;
        }
        else if (appId >= T2_FIRST_ID && appId <= T2_LAST_ID) {
          frskyData.hub.temperature2 = SPORT_DATA_S32(packet);
          if (frskyData.hub.temperature2 > frskyData.hub.maxTemperature2)
            frskyData.hub.maxTemperature2 = frskyData.hub.temperature2;
        }
        else if (appId >= RPM_FIRST_ID && appId <= RPM_LAST_ID) {
          frskyData.hub.rpm = SPORT_DATA_U32(packet) / (g_model.frsky.blades+2);
          if (frskyData.hub.rpm > frskyData.hub.maxRpm)
            frskyData.hub.maxRpm = frskyData.hub.rpm;
        }
        else if (appId >= FUEL_FIRST_ID && appId <= FUEL_LAST_ID) {
          frskyData.hub.fuelLevel = SPORT_DATA_U32(packet);
        }
        else if (appId >= ALT_FIRST_ID && appId <= ALT_LAST_ID) {
          setBaroAltitude(SPORT_DATA_S32(packet));
        }
        else if (appId >= VARIO_FIRST_ID && appId <= VARIO_LAST_ID) {
          frskyData.hub.varioSpeed = SPORT_DATA_S32(packet);
        }
        else if (appId >= ACCX_FIRST_ID && appId <= ACCX_LAST_ID) {
          frskyData.hub.accelX = SPORT_DATA_S32(packet);
        }
        else if (appId >= ACCY_FIRST_ID && appId <= ACCY_LAST_ID) {
          frskyData.hub.accelY = SPORT_DATA_S32(packet);
        }
        else if (appId >= ACCZ_FIRST_ID && appId <= ACCZ_LAST_ID) {
          frskyData.hub.accelZ = SPORT_DATA_S32(packet);
        }
        else if (appId >= CURR_FIRST_ID && appId <= CURR_LAST_ID) {
          frskyData.hub.current = SPORT_DATA_U32(packet);
          if (((int16_t)frskyData.hub.current + g_model.frsky.fasOffset)>0)
            frskyData.hub.current += g_model.frsky.fasOffset;
          else
            frskyData.hub.current = 0;
          if (frskyData.hub.current > frskyData.hub.maxCurrent)
            frskyData.hub.maxCurrent = frskyData.hub.current;
        }
        else if (appId >= VFAS_FIRST_ID && appId <= VFAS_LAST_ID) {
          frskyData.hub.vfas = SPORT_DATA_U32(packet)/10;   //TODO: remove /10 and display with PREC2 when using SPORT
          if (!frskyData.hub.minVfas || frskyData.hub.vfas < frskyData.hub.minVfas)
            frskyData.hub.minVfas = frskyData.hub.vfas;
        }
        else if (appId >= AIR_SPEED_FIRST_ID && appId <= AIR_SPEED_LAST_ID) {
          frskyData.hub.airSpeed = SPORT_DATA_U32(packet);
          if (frskyData.hub.airSpeed > frskyData.hub.maxAirSpeed)
            frskyData.hub.maxAirSpeed = frskyData.hub.airSpeed;
        }
        else if (appId >= GPS_SPEED_FIRST_ID && appId <= GPS_SPEED_LAST_ID) {
          frskyData.hub.gpsSpeed_bp = (uint16_t) (SPORT_DATA_U32(packet)/1000);
          if (frskyData.hub.gpsSpeed_bp > frskyData.hub.maxGpsSpeed)
            frskyData.hub.maxGpsSpeed = frskyData.hub.gpsSpeed_bp;
        }
        else if (appId >= GPS_TIME_DATE_FIRST_ID && appId <= GPS_TIME_DATE_LAST_ID) {
          uint32_t gps_time_date = SPORT_DATA_U32(packet);
          if (gps_time_date & 0x000000ff) {
            frskyData.hub.year = (uint16_t) ((gps_time_date & 0xff000000) >> 24);
            frskyData.hub.month = (uint8_t) ((gps_time_date & 0x00ff0000) >> 16);
            frskyData.hub.day = (uint8_t) ((gps_time_date & 0x0000ff00) >> 8);
          }
          else {
            frskyData.hub.hour = (uint8_t) ((gps_time_date & 0xff000000) >> 24);
            frskyData.hub.min = (uint8_t) ((gps_time_date & 0x00ff0000) >> 16);
            frskyData.hub.sec = (uint16_t) ((gps_time_date & 0x0000ff00) >> 8);
            frskyData.hub.hour = ((uint8_t) (frskyData.hub.hour + g_eeGeneral.timezone + 24)) % 24;
          }
        }
        else if (appId >= GPS_COURS_FIRST_ID && appId <= GPS_COURS_LAST_ID) {
          uint32_t course = SPORT_DATA_U32(packet);
          frskyData.hub.gpsCourse_bp = course / 100;
          frskyData.hub.gpsCourse_ap = course % 100;
        }
        else if (appId >= GPS_ALT_FIRST_ID && appId <= GPS_ALT_LAST_ID) {
          frskyData.hub.gpsAltitude = SPORT_DATA_S32(packet);

          if (!frskyData.hub.gpsAltitudeOffset)
            frskyData.hub.gpsAltitudeOffset = -frskyData.hub.gpsAltitude;

          if (!frskyData.hub.baroAltitudeOffset) {
            int altitude = TELEMETRY_RELATIVE_GPS_ALT_BP;
            if (altitude > frskyData.hub.maxAltitude)
              frskyData.hub.maxAltitude = altitude;
            if (altitude < frskyData.hub.minAltitude)
              frskyData.hub.minAltitude = altitude;
          }

          if (frskyData.hub.gpsFix > 0) {
            if (!frskyData.hub.pilotLatitude && !frskyData.hub.pilotLongitude) {
              // First received GPS position => Pilot GPS position
              getGpsPilotPosition();
            }
            else if (frskyData.hub.gpsDistNeeded || g_menuStack[g_menuStackPtr] == menuTelemetryFrsky) {
              getGpsDistance();
            }
          }
        }
        else if (appId >= GPS_LONG_LATI_FIRST_ID && appId <= GPS_LONG_LATI_LAST_ID) {
          uint32_t gps_long_lati_data = SPORT_DATA_U32(packet);
          uint32_t gps_long_lati_b1w, gps_long_lati_a1w;
          gps_long_lati_b1w = (gps_long_lati_data & 0x3fffffff) / 10000;
          gps_long_lati_a1w = (gps_long_lati_data & 0x3fffffff) % 10000;
          switch ((gps_long_lati_data & 0xc0000000) >> 30) {
            case 0:
              frskyData.hub.gpsLatitude_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
              frskyData.hub.gpsLatitude_ap = gps_long_lati_a1w;
              frskyData.hub.gpsLatitudeNS = 'N';
              break;
            case 1:
              frskyData.hub.gpsLatitude_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
              frskyData.hub.gpsLatitude_ap = gps_long_lati_a1w;
              frskyData.hub.gpsLatitudeNS = 'S';
              break;
            case 2:
              frskyData.hub.gpsLongitude_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
              frskyData.hub.gpsLongitude_ap = gps_long_lati_a1w;
              frskyData.hub.gpsLongitudeEW = 'E';
              break;
            case 3:
              frskyData.hub.gpsLongitude_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
              frskyData.hub.gpsLongitude_ap = gps_long_lati_a1w;
              frskyData.hub.gpsLongitudeEW = 'W';
              break;
          }
          if (frskyData.hub.gpsLongitudeEW && frskyData.hub.gpsLatitudeNS) {
            frskyData.hub.gpsFix = 1;
          }
          else if (frskyData.hub.gpsFix > 0) {
            frskyData.hub.gpsFix = 0;
          }
        }
        else if (appId >= A3_FIRST_ID && appId <= A3_LAST_ID) {
          frskyData.analog[TELEM_ANA_A3].set((SPORT_DATA_U32(packet)*255+165)/330, UNIT_VOLTS);
        }
        else if (appId >= A4_FIRST_ID && appId <= A4_LAST_ID) {
          frskyData.analog[TELEM_ANA_A4].set((SPORT_DATA_U32(packet)*255+165)/330, UNIT_VOLTS);
        }
        else if (appId >= CELLS_FIRST_ID && appId <= CELLS_LAST_ID) {
          uint32_t data = SPORT_DATA_U32(packet);
          uint8_t battnumber = data & 0xF;
          uint8_t cells = (data & 0xF0) >> 4;
          bool useSecondCell = (battnumber+1 < cells);

          if (dataId == DATA_ID_FLVSS) {
            // first sensor, remember its cell count
            frskyData.hub.sensorCellsCount[0] = cells;
            cells += frskyData.hub.sensorCellsCount[1];
          }
          else {
            // second sensor connected
            frskyData.hub.sensorCellsCount[1] = cells;
            cells += frskyData.hub.sensorCellsCount[0];
            battnumber += frskyData.hub.sensorCellsCount[0];
          }

          if (cells != frskyData.hub.cellsCount) {
            frskySetCellsCount(cells);
          }

          frskySetCellVoltage(battnumber, (frskyCellVoltage_t) ((data & 0x000FFF00) >>  8) / 5);
          if (useSecondCell) {
            frskySetCellVoltage(battnumber+1, (frskyCellVoltage_t) ((data & 0xFFF00000) >> 20) / 5);
          }
        }
      }
      break;
  }
}

#if defined(PCBTARANIS)
bool sportWaitState(SportUpdateState state, int timeout)
{
  for (int i=timeout/2; i>=0; i--) {
    uint8_t data ;
    while (telemetryFifo.pop(data)) {
      processSerialData(data);
    }
    if (sportUpdateState == state) {
      return true;
    }
    else if (sportUpdateState == SPORT_FAIL) {
      return false;
    }
#if defined(SIMU)
    SIMU_SLEEP_NORET(1);
    return true;
#endif
    CoTickDelay(1);
  }
  return false;
}

void blankPacket(uint8_t *packet)
{
  memset(packet+2, 0, 6);
}

/* CRC16 implementation acording to CCITT standards */

static const unsigned short crc16tab[256]= {
  0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
  0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
  0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
  0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
  0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
  0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
  0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
  0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
  0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
  0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
  0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
  0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
  0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
  0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
  0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
  0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
  0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
  0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
  0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
  0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
  0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
  0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
  0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
  0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
  0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
  0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
  0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
  0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
  0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
  0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
  0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
  0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

uint16_t crc16_ccitt(uint8_t *buf, uint32_t len)
{
  uint16_t crc = 0;
  for (uint32_t counter=0; counter<len; counter++) {
    crc = (crc<<8) ^ crc16tab[ ((crc>>8) ^ *buf++ ) &0x00FF];
  }
  return crc;
}

uint8_t sportUpdatePacket[16];

void writePacket(uint8_t *packet)
{
  uint8_t *ptr = sportUpdatePacket;
  *ptr++ = 0x7E;
  *ptr++ = 0xFF;
  packet[7] = crc16_ccitt(packet, 7);
  for (int i=0; i<8; i++) {
    if (packet[i] == 0x7E || packet[i] == 0x7D) {
      *ptr++ = 0x7D;
      *ptr++ = 0x20 ^ packet[i];
    }
    else {
      *ptr++ = packet[i];
    }
  }
  sportSendBuffer(sportUpdatePacket, ptr-sportUpdatePacket);
}

bool sportUpdatePowerOn(ModuleIndex module)
{
  uint8_t packet[8];

  sportUpdateState = SPORT_POWERUP_REQ;

#if defined(PCBTARANIS)
  intPwr = IS_INTERNAL_MODULE_ON();
  extPwr = IS_EXTERNAL_MODULE_ON();
  INTERNAL_MODULE_OFF();
  EXTERNAL_MODULE_OFF();
#endif

  sportWaitState(SPORT_IDLE, 500);

  telemetryPortInit(FRSKY_SPORT_BAUDRATE);

#if defined(PCBTARANIS)
  if (module == INTERNAL_MODULE)
    INTERNAL_MODULE_ON();
  else
    EXTERNAL_MODULE_ON();
#endif

  sportWaitState(SPORT_IDLE, 50);

  for (int i=0; i<10; i++) {
    // max 10 attempts
    blankPacket(packet);
    packet[0] = 0x50 ;
    packet[1] = PRIM_REQ_POWERUP;
    writePacket(packet);
    if (sportWaitState(SPORT_POWERUP_ACK, 100))
      return true;
  }
  return false;
}

bool sportUpdateReqVersion()
{
  uint8_t packet[8];
  sportWaitState(SPORT_IDLE, 20);
  sportUpdateState = SPORT_VERSION_REQ;
  for (int i=0; i<10; i++) {
    // max 10 attempts
    blankPacket(packet) ;
    packet[0] = 0x50 ;
    packet[1] = PRIM_REQ_VERSION ;
    writePacket(packet);
    if (sportWaitState(SPORT_VERSION_ACK, 200))
      return true;
  }
  return false;
}

bool sportUpdateUploadFile(const char *filename)
{
  FIL file;
  uint32_t buffer[1024/4];
  UINT count;
  uint8_t packet[8];

  if (f_open(&file, filename, FA_READ) != FR_OK) {
    return false;
  }

  sportWaitState(SPORT_IDLE, 200);
  sportUpdateState = SPORT_DATA_TRANSFER;
  blankPacket(packet) ;
  packet[0] = 0x50 ;
  packet[1] = PRIM_CMD_DOWNLOAD ;
  // Stop here for testing
  writePacket(packet);

  while(1) {
    if (f_read(&file, buffer, 1024, &count) != FR_OK) {
      f_close(&file);
      return false;
    }

    count >>= 2;

    for (UINT i=0; i<count; i++) {
      if (!sportWaitState(SPORT_DATA_REQ, 2000)) {
        return false;
      }
      packet[0] = 0x50 ;
      packet[1] = PRIM_DATA_WORD ;
      packet[6] = sportUpdateAddr & 0x000000FF;
      uint32_t offset = ( sportUpdateAddr & 1023 ) >> 2;           // 32 bit word offset into buffer
      uint32_t *data = (uint32_t *)(&packet[2]);
      *data = buffer[offset];
      sportUpdateState = SPORT_DATA_TRANSFER,
      writePacket(packet);
      if (i==0) {
        updateProgressBar(file.fptr, file.fsize);
      }
    }

    if (count < 256) {
      f_close(&file);
      return true;
    }
  }
}

bool sportUpdateEnd()
{
  uint8_t packet[8];
  if (!sportWaitState(SPORT_DATA_REQ, 2000))
    return false;
  blankPacket(packet);
  packet[0] = 0x50 ;
  packet[1] = PRIM_DATA_EOF;
  writePacket(packet);
  return sportWaitState(SPORT_COMPLETE, 2000);
}

void sportFirmwareUpdate(ModuleIndex module, const char *filename)
{
  bool result = sportUpdatePowerOn(module);
  if (result)
  	result = sportUpdateReqVersion();
  if (result)
  	result = sportUpdateUploadFile(filename);
  if (result)
  	result = sportUpdateEnd();
  
  if (result == false) {
    POPUP_WARNING("Firmware Update Error");
  }
  
#if defined(PCBTARANIS)
  INTERNAL_MODULE_OFF();
  EXTERNAL_MODULE_OFF();
  
  sportWaitState(SPORT_IDLE, 1000);

  if (intPwr) 
    INTERNAL_MODULE_ON();
  if (extPwr)
    EXTERNAL_MODULE_ON();
#endif
  sportUpdateState = SPORT_IDLE;
}

#endif
