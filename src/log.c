#include <stdio.h>
#include <stdlib.h>

#include "log.h"

#include "stopwatch.h"

#define DO_LOG_START 1
#define DO_LOG_END 1

long timestack[255];
int timestack_pointer = 0;

void logStart(char * text) {
	if (DO_LOG_START || DO_LOG_END) {
		timestack[timestack_pointer] = getTimestamp();
		timestack_pointer++;

		if (timestack_pointer > 254) {
			printf("Recursion too deep - EXIT\n");
			exit(1);
		}

		if (DO_LOG_START)
			printf("%d LOG START | %s\n", timestack_pointer, text);
	}
}

void logEnd(char * text) {
	if (DO_LOG_START || DO_LOG_END) {
		timestack_pointer--;

		if (timestack_pointer < 0) {
			printf("logEnd called too often - EXIT\n");
			exit(1);
		}

		if (DO_LOG_END) {
			long duration = getTimestamp() - timestack[timestack_pointer];
			printf("%d LOG END   | %s [%ld us]\n", timestack_pointer + 1, text,
					duration);
		}
	}
}

void logStartA(char * text, long a) {
	if (DO_LOG_END || DO_LOG_START) {
		char buffer[255];
		sprintf(buffer, text, a);
		logStart(buffer);
	}
}

void logEndA(char * text, long a) {
	if (DO_LOG_END || DO_LOG_START) {
		char buffer[255];
		sprintf(buffer, text, a);
		logEnd(buffer);
	}
}

/* void logEnd(char * text, long duration) {
 printf("LOG END %s [%d ms]", text, duration);
 } */

// void log(char * text);
