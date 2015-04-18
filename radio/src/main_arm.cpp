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

#include "opentx.h"

uint8_t currentSpeakerVolume = 255;
uint8_t requiredSpeakerVolume = 255;
uint8_t requestScreenshot = false;

void handleUsbConnection()
{
#if defined(PCBTARANIS) && !defined(SIMU)
  static bool usbStarted = false;
  if (!usbStarted && usbPlugged()) {
#if defined(USB_MASS_STORAGE)
    opentxClose();
#endif
    usbStart();
#if defined(USB_MASS_STORAGE)
    usbPluggedIn();
#endif
    usbStarted = true;
  }
  
#if defined(USB_JOYSTICK)
  if (usbStarted) {
    if (!usbPlugged()) {
      //disable USB
      usbStop();
      usbStarted = false;
    }
    else {
      usbJoystickUpdate();
    }
  }
#endif
#endif //#if defined(PCBTARANIS) && !defined(SIMU)
}

void checkSpeakerVolume()
{
  if (currentSpeakerVolume != requiredSpeakerVolume) {
    currentSpeakerVolume = requiredSpeakerVolume;
    setVolume(currentSpeakerVolume);
  }
}

void checkEeprom()
{
  if (!usbPlugged()) {
    // TODO merge these 2 branches
#if defined(PCBSKY9X)
    if (eepromWriteState != EEPROM_IDLE)
      eepromWriteProcess();
    else if (TIME_TO_WRITE())
      eeCheck(false);
#else
    if (theFile.isWriting())
      theFile.nextWriteStep();
    else if (TIME_TO_WRITE())
      eeCheck(false);
#endif
  }
}

void perMain()
{
#if defined(PCBSKY9X) && !defined(REVA)
  calcConsumption();
#endif
  checkSpeakerVolume();
  checkEeprom();
  sdMountPoll();
  writeLogs();
  handleUsbConnection();
  checkTrainerSettings();
  checkBattery();

  uint8_t evt = getEvent(false);
  if (evt && (g_eeGeneral.backlightMode & e_backlight_mode_keys)) backlightOn(); // on keypress turn the light on
  checkBacklight();
#if defined(NAVIGATION_STICKS)
  uint8_t sticks_evt = getSticksNavigationEvent();
  if (sticks_evt) evt = sticks_evt;
#endif

#if defined(USB_MASS_STORAGE)
  if (usbPlugged()) {
    // disable access to menus
    lcd_clear();
    menuMainView(0);
    lcdRefresh();
    return;
  }
#endif

#if defined(LUA)
  uint32_t t0 = get_tmr10ms();
  static uint32_t lastLuaTime = 0;
  uint16_t interval = (lastLuaTime == 0 ? 0 : (t0 - lastLuaTime));
  lastLuaTime = t0;
  if (interval > maxLuaInterval) {
    maxLuaInterval = interval;
  }

  // run Lua scripts that don't use LCD (to use CPU time while LCD DMA is running)
  luaTask(0, RUN_MIX_SCRIPT | RUN_FUNC_SCRIPT | RUN_TELEM_BG_SCRIPT, false);

  // wait for LCD DMA to finish before continuing, because code from this point 
  // is allowed to change the contents of LCD buffer
  // 
  // WARNING: make sure no code above this line does any change to the LCD display buffer!
  //
  lcdRefreshWait();

  // draw LCD from menus or from Lua script
  // run Lua scripts that use LCD 
  bool scriptWasRun = luaTask(evt, RUN_TELEM_FG_SCRIPT | RUN_STNDAL_SCRIPT, true);

  t0 = get_tmr10ms() - t0;
  if (t0 > maxLuaDuration) {
    maxLuaDuration = t0;
  }

  if (!scriptWasRun)
  {
#else
  lcdRefreshWait();   // WARNING: make sure no code above this line does any change to the LCD display buffer!
  {
#endif
    // normal GUI from menus
    const char *warn = s_warning;
    uint8_t menu = s_menu_count;
    lcd_clear();
    if (menuEvent) {
      m_posVert = menuEvent == EVT_ENTRY_UP ? g_menuPos[g_menuStackPtr] : 0;
      m_posHorz = 0;
      evt = menuEvent;
      menuEvent = 0;
      AUDIO_MENUS();
    }
    g_menuStack[g_menuStackPtr]((warn || menu) ? 0 : evt);
    if (warn) DISPLAY_WARNING(evt);
    if (menu) {
      const char * result = displayMenu(evt);
      if (result) {
        menuHandler(result);
        putEvent(EVT_MENU_UP);
      }
    }
    drawStatusLine();
  }

#if defined(REV9E) && !defined(SIMU)
  uint32_t pwr_pressed_duration = pwrPressedDuration();
  if (pwr_pressed_duration > 0) {
    displayShutdownProgress(pwr_pressed_duration);
  }
#endif

  lcdRefresh();

#if defined(REV9E) && !defined(SIMU)
  topLcdRefreshStart();
  setTopFirstTimer(getValue(MIXSRC_FIRST_TIMER+g_model.topLcdTimer));
  setTopSecondTimer(g_eeGeneral.globalTimer + sessionTimer);
  setTopRssi(TELEMETRY_RSSI());
  setTopBatteryValue(g_vbat100mV);
  int state = 5 * (g_vbat100mV - g_eeGeneral.vBatMin - 90) / (30 + g_eeGeneral.vBatMax - g_eeGeneral.vBatMin);
  setTopBatteryState(state);
  topLcdRefreshEnd();
#endif

#if defined(REV9E) && !defined(SIMU)
  bt_wakeup();
#endif

#if defined(PCBTARANIS)
  if (requestScreenshot) {
    requestScreenshot = false;
    writeScreenshot();
  }
#endif

}
