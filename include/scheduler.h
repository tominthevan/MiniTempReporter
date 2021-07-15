//scheduler simple time derived activity scheduling adapted from J.C. Wippler's Ports library
// developed initially for his Jeenode processors, based on the Arduino ecosystem
// 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#ifndef scheduler_h
#define scheduler_h

/// @file
/// Ports library definitions.

#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif
#include <stdint.h>
#include <avr/pgmspace.h>

/// The millisecond timer can be used for timeouts up to 60000 milliseconds.
/// Setting the timeout to zero disables the timer.
///
/// * for periodic use, poll the timer object with "if (timer.poll(123)) ..."
/// * for one-shot use, call "timer.set(123)" and poll as "if (timer.poll())"

class MilliTimer {
    word next;
    byte armed;
public:
    MilliTimer () : armed (0) {}
    
    /// poll until the timer fires
    /// @param ms Periodic repeat rate of the time, omit for a one-shot timer.
    byte poll(word ms =0);
    /// Return the number of milliseconds before the timer will fire
    word remaining() const;
    /// Returns true if the timer is not armed
    byte idle() const { return !armed; }
    /// set the one-shot timeout value
    /// @param ms Timeout value. Timer stops once the timer has fired.
    void set(word ms);
};



/// simple task scheduler for times up to 6000 seconds
class Scheduler {
    word* tasks;
    word remaining;
    byte maxTasks;
    MilliTimer ms100;
public:
    /// initialize for a specified maximum number of tasks
    Scheduler (byte max);
    Scheduler (word* buf, byte max);

    /// Return next task to run, -1 if there are none ready to run, but there
    /// are tasks waiting, or -2 if there are no tasks waiting (i.e. all idle)
    char poll();
    ///pollWaiting() not available on ESP8266 (yet anyway)
    ////// same as poll, but wait for event in power-down mode.
    ////// Uses Sleepy::loseSomeTime() - see comments there re requiring the
    ////// watchdog timer. 
    ///char pollWaiting();
    
    /// set a task timer, in tenths of seconds
    void timer(byte task, word tenths);
    /// cancel a task timer
    void cancel(byte task);
    
    /// return true if a task timer is not running
    byte idle(byte task) { return tasks[task] == ~0U; }
};


#endif
