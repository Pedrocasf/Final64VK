#ifndef FINAL64_SCHEDULER_H
#define FINAL64_SCHEDULER_H
typedef enum Event{
    Boot,
};
typedef struct ScheduledEvent {
    Event e;
    uint32_t cyclesUntilEvent;
    ScheduledEvent* next;
};
typedef struct Scheduler{
    ScheduledEvent* list;
};
void schedule(Event e, int cyclesUntilEvent, Scheduler* s);
#endif
