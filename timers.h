/*
 * timers.h: Timer handling
 *
 * See the main source file 'vdr.c' for copyright information and
 * how to reach the author.
 *
 * $Id: timers.h 1.15 2005/03/19 14:22:11 kls Exp $
 */

#ifndef __TIMERS_H
#define __TIMERS_H

#include "channels.h"
#include "config.h"
#include "epg.h"
#include "tools.h"

enum eTimerFlags { tfNone      = 0x0000,
                   tfActive    = 0x0001,
                   tfInstant   = 0x0002,
                   tfVps       = 0x0004,
                   tfAll       = 0xFFFF,
                 };
enum eTimerMatch { tmNone, tmPartial, tmFull };

class cTimer : public cListObject {
  friend class cMenuEditTimer;
private:
  mutable time_t startTime, stopTime;
  bool recording, pending, inVpsMargin;
  int flags;
  cChannel *channel;
  mutable time_t day;   /// midnight of the day this timer shall hit, or of the first day it shall hit in case of a repeating timer
  int weekdays; /// bitmask, lowest bits: SSFTWTM  (the 'M' is the LSB)
  int start;
  int stop;
  int priority;
  int lifetime;
  char file[MaxFileName];
  char *summary;
  const cEvent *event;
public:
  cTimer(bool Instant = false, bool Pause = false);
  cTimer(const cEvent *Event);
  virtual ~cTimer();
  cTimer& operator= (const cTimer &Timer);
  virtual int Compare(const cListObject &ListObject) const;
  bool Recording(void) { return recording; }
  bool Pending(void) { return pending; }
  bool InVpsMargin(void) { return inVpsMargin; }
  int Flags(void) { return flags; }
  const cChannel *Channel(void) { return channel; }
  time_t Day(void) { return day; }
  int WeekDays(void) { return weekdays; }
  int Start(void) { return start; }
  int Stop(void) { return stop; }
  int Priority(void) { return priority; }
  int Lifetime(void) { return lifetime; }
  const char *File(void) { return file; }
  time_t FirstDay(void) { return weekdays ? day : 0; }
  const char *Summary(void) { return summary; }
  cString ToText(bool UseChannelID = false);
  const cEvent *Event(void) { return event; }
  bool Parse(const char *s);
  bool Save(FILE *f);
  bool IsSingleEvent(void) const;
  static int GetMDay(time_t t);
  static int GetWDay(time_t t);
  bool DayMatches(time_t t) const;
  static time_t IncDay(time_t t, int Days);
  static time_t SetTime(time_t t, int SecondsFromMidnight);
  char *SetFile(const char *File);
  bool Matches(time_t t = 0, bool Directly = false) const;
  int Matches(const cEvent *Event, int *Overlap = NULL);
  time_t StartTime(void) const;
  time_t StopTime(void) const;
  void SetEvent(const cEvent *Event);
  void SetRecording(bool Recording);
  void SetPending(bool Pending);
  void SetInVpsMargin(bool InVpsMargin);
  void SetFlags(int Flags);
  void ClrFlags(int Flags);
  void InvFlags(int Flags);
  bool HasFlags(int Flags) const;
  void Skip(void);
  void OnOff(void);
  cString PrintFirstDay(void);
  static int TimeToInt(int t);
  static bool ParseDay(const char *s, time_t &Day, int &WeekDays);
  static cString PrintDay(time_t Day, int WeekDays);
  };

class cTimers : public cConfig<cTimer> {
private:
  bool modified;
  int beingEdited;
  time_t lastSetEvents;
public:
  cTimers(void);
  cTimer *GetTimer(cTimer *Timer);
  cTimer *GetMatch(time_t t);
  cTimer *GetMatch(const cEvent *Event, int *Match = NULL);
  cTimer *GetNextActiveTimer(void);
  int BeingEdited(void) { return beingEdited; }
  void IncBeingEdited(void) { beingEdited++; }
  void DecBeingEdited(void) { if (!--beingEdited) lastSetEvents = 0; }
  void SetModified(void);
  bool Modified(void);
      ///< Returns true if any of the timers have been modified.
      ///< Calling this function resets the 'modified' flag to false.
  void SetEvents(void);
  };

extern cTimers Timers;

#endif //__TIMERS_H
