#include <sys/time.h>
#include <stdio.h>

struct timeval start, end, now;

/** Starts time measure */
void start_stopwatch() {
	gettimeofday(&start, NULL);
}

/** Returns the time in microseconds (1 * 10^-6) since last call of start_stopwatch */
long stop_stopwatch() {
	gettimeofday(&end, NULL);

	long mtime, seconds, useconds;

	seconds = end.tv_sec - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;

	mtime = (seconds) * 1000000 + useconds;

	return mtime;
}

/** Returns timestamp in microseconds - multiply by 1000000 to get seconds */
long getTimestamp() {
	gettimeofday(&now, NULL);

	long mtime, seconds, useconds;

	seconds = now.tv_sec;
	useconds = now.tv_usec;
	mtime = (seconds) * 1000000 + useconds;

	return mtime;
}

/*
#include <sys/time.h>
#include <stdio.h>

struct timeval start, end, now;

static inline unsigned long long rdtsc(void)
{
	unsigned long long int x;
	__asm__ __volatile__ (".byte 0x0f, 0x31" : "=A" (x));
	return x;
}

void start_stopwatch() {
	gettimeofday(&start, NULL);
}

long stop_stopwatch() {
	gettimeofday(&end, NULL);

	long mtime, seconds, useconds;

	seconds = end.tv_sec - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;

	mtime = (seconds) * 1000000 + useconds;

	return mtime;
}

long getTimestamp() {
	gettimeofday(&now, NULL);

	long mtime, seconds, useconds;

	seconds = now.tv_sec;
	useconds = now.tv_usec;
	mtime = (seconds) * 1000000 + useconds;

	return mtime;
}
*/
