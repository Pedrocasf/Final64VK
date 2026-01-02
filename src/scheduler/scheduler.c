#include "scheduler.h"
void schedule(Event e, int cyclesUntilEvent, Scheduler* s){

}
ScheduledEvent popNextEvent(Scheduler *s){
    ScheduledEvent* current = s->list;
    ScheduledEvent* previous = s->list;
    uint32_t smallest_val = current->cyclesUntilEvent;
    ScheduledEvent* smallest_node = s->list;
    ScheduledEvent* previous_to_smallest = NULL;
    while (current->next != NULL){
        if(current.cyclesUntilEvent < smallest_val){
            smallest_val = current.cyclesUntilEvent;
            smallest_node = current;
            previous_to_smallest = previous
        }
        previous = current;
        current = current->next;
    }
    if(smallest_node == s->list){
        s->list = s->list.next;
    }else {
        previous_to_smallest.next = smallest_node.next;
    }
    return smallest_node;
}
