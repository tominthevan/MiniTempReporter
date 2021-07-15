// Two simple activity timing mechanisms borrowed from JC Wippler's
// Jeelib library. I have repurposed these for use with my ESP8266 projects.
// JC's oroginal source can be found on github.
// JC's description of these 2 services can be found here:
//              https://jeelabs.org/2010/10/17/scheduling-multiple-tasks/index.html
//
// 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include "scheduler.h"

// MilliTimer provides periodic activities with a maximum interval of about 60 seconds.
// This a simple low overhead scheduler with no provision for longer intervals or oneshot activities.
// See Scheduler below for longer intervals and 1 shot tasks.

byte MilliTimer::poll(word ms) {
    byte ready = 0;
    if (armed) {
        word remain = next - millis();
        // since remain is unsigned, it will overflow to large values when
        // the timeout is reached, so this test works as long as poll() is
        // called no later than 5535 millisecs after the timer has expired
        if (remain <= 60000)
            return 0;
        // return a value between 1 and 255, being msecs+1 past expiration
        // note: the actual return value is only reliable if poll() is
        // called no later than 255 millisecs after the timer has expired
        ready = -remain;
    }
    set(ms);
    return ready;
}

word MilliTimer::remaining() const {
    word remain = armed ? next - millis() : 0;
    return remain <= 60000 ? remain : 0;
}

void MilliTimer::set(word ms) {
    armed = ms != 0;
    if (armed)
        next = millis() + ms - 1;
}

//
// Simple periodic task scheduler. The minimum interval is 0.1 sec and the max is 6000 seconds.
// tasks
//
Scheduler::Scheduler (byte size) : remaining (~0), maxTasks (size) {
    byte bytes = size * sizeof *tasks;
    tasks = (word*) malloc(bytes);
    memset(tasks, 0xFF, bytes);
}

Scheduler::Scheduler (word* buf, byte size) : tasks (buf), remaining(~0), maxTasks (size) {
    byte bytes = size * sizeof *tasks;
    memset(tasks, 0xFF, bytes);
}

char Scheduler::poll() {
    // all times in the tasks array are relative to the "remaining" value
    // i.e. only remaining counts down while waiting for the next timeout
    if (remaining == 0) {
        word lowest = ~0;
        for (byte i = 0; i < maxTasks; ++i) {
            if (tasks[i] == 0) {
                tasks[i] = ~0;
                return i;
            }
            if (tasks[i] < lowest)
                lowest = tasks[i];
        }
        if (lowest != ~0U) {
            for (byte i = 0; i < maxTasks; ++i) {
                if(tasks[i] != ~0U) {
                    tasks[i] -= lowest;
                }
            }
        } else {
            // must turn off timer or it might overflow if its poll-method
            // is not called within 5535 ms, i.e. if no tasks are scheduled
            ms100.set(0);
        }
        remaining = lowest;
    } else if (remaining == ~0U) //remaining == ~0 means nothing running
        return -2;
    else if (ms100.poll(100))
        --remaining;
    return -1;
}
/// ESP8266 Deep sleep works differently than Arduino (and Jeenodes)
/// For now disable this capability
///char Scheduler::pollWaiting() {
///    if(remaining == ~0U)  // Nothing running!
///        return -2;
///    // first wait until the remaining time we need to wait is less than 0.1s
///    while (remaining > 0) {
///        word step = remaining > 600 ? 600 : remaining;
///        if (!Sleepy::loseSomeTime(100 * step)) // uses least amount of power
///            return -1;
///        remaining -= step;
///    }
///    // now lose some more time until that 0.1s mark
///    if (!Sleepy::loseSomeTime(ms100.remaining()))
///        return -1;
///    // lastly, just ignore the 0..15 ms still left to go until the 0.1s mark
///    return poll();
///}

void Scheduler::timer(byte task, word tenths) {
    // if new timer will go off sooner than the rest, then adjust all entries
    if (tenths < remaining) {
        word diff = remaining - tenths;
        for (byte i = 0; i < maxTasks; ++i)
            if (tasks[i] != ~0U)
                tasks[i] += diff;
        remaining = tenths;
    }
    tasks[task] = tenths - remaining;
}

void Scheduler::cancel(byte task) {
    tasks[task] = ~0;
}
