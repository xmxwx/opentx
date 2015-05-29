#ifndef debugtimers_h
#define debugtimers_h


#include "opentx.h"

#if defined(DEBUG_TIMERS)


typedef uint32_t debug_timer_t;

class DebugTimer
{
private:
  debug_timer_t min;
  debug_timer_t max;
  // debug_timer_t avg;
  debug_timer_t last;   //unit 0.01ms

  uint16_t _start_hiprec;
  uint32_t _start_loprec;

  void evalStats() {
    if (min > last) min = last;
    if (max < last) max = last;
    //todo avg
  }

public:
  DebugTimer(): min(-1), max(0), /*avg(0),*/ last(0), _start_hiprec(0), _start_loprec(0) {};
  // ~DebugTimer();

  void start() {
#if defined(CPUARM)
    _start_hiprec = getTmr2MHz();
#else
    _start_hiprec = getTmr16KHz();
#endif
    _start_loprec = get_tmr10ms();
  }
  void stop() {
    // getTmr2MHz is 16 bit timer, resolution 0.5us, max measurable value 32.7675 milli seconds
    // getTmr16KHz is 16 bit timer, resolution 64us, max measurable value 4194.304 milli seconds
    // tmr10ms_t tmr10ms = get_tmr10ms(); 32 bit timer, resolution 10ms, max measurable value: 42949672.95 s = 1.3 years
    if ((_start_hiprec == 0) && (_start_loprec == 0)) return;

    last = get_tmr10ms() - _start_loprec;  //use low precision timer
#if defined(CPUARM)
    if (last < 3) {
      // if time difference is less than 30ms, use high resolution timer
      last = (uint16_t)(getTmr2MHz() - _start_hiprec) / 20;
    }
#else
    if (last < 419) {
      // if time difference is less than 4190ms, use high resolution timer
      last = (uint32_t)(uint16_t)(getTmr16KHz() - _start_hiprec) * 64 / 10;
    }
#endif
    else {
      last *= 1000ul; //adjust unit to 0.01ms
    }

    evalStats();
  }
  void sample() { stop(); start(); }

  void reset() { min = -1;  max = last = 0; }

  debug_timer_t getMin() const { return min;}
  debug_timer_t getMax() const { return max;}
  debug_timer_t getLast() const { return last;}

};


#define DEBUG_TIMER_START(timer)  timer.start()
#define DEBUG_TIMER_STOP(timer)   timer.stop()
#define DEBUG_TIMER_SAMPLE(timer) timer.sample()


#else //#if defined(DEBUG_TIMERS)

#define DEBUG_TIMER_START(timer)
#define DEBUG_TIMER_STOP(timer)
#define DEBUG_TIMER_SAMPLE(timer)

#endif //#if defined(DEBUG_TIMERS)

#endif
