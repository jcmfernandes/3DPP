#ifndef STOPWATCH_H_
#define STOPWATCH_H_

/** Starts time measure */
void start_stopwatch();

/** Returns the time in microseconds (1 * 10^-6) since last call of start_stopwatch */
long stop_stopwatch();

long getTimestamp() ;

#endif /* STOPWATCH_H_ */

/*
#ifndef STOPWATCH_H_
#define STOPWATCH_H_

struct stopwatch_handle;

stopwatch_handle* stopwatch_create(void);
void stopwatch_delete( struct stopwatch_handle*);

void stopwatch_start(struct stopwatch_handle*);
void stopwatch_stop(struct stopwatch_handle*);
void stopwatch_reset(struct stopwatch_handle*);
void stopwatch_get_elapsed(struct stopwatch_handle*);

#endif
*/
