/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
*/
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#if	IBM
#include <windows.h>
#include <dbghelp.h>
#else	/* !IBM */
#include <execinfo.h>   /* used for stack tracing */
#endif	/* !IBM */

#ifndef	TEST_STANDALONE_BUILD
#include "XPLMUtilities.h"
#endif

#include "assert.h"
#include "helpers.h"
#include "log.h"

#define	PREFIX		"X-TCAS"
#define	DATE_FMT	"%Y-%m-%d %H:%M:%S"
#ifndef	TEST_STANDALONE_BUILD
#define	PREFIX_FMT	"%s %s[%s:%d]: ", timedate, PREFIX, filename, line
#else	/* TEST_STANDALONE_BUILD */
#define	PREFIX_FMT	"%s:%d: ", filename ? filename : "", line
#endif	/* TEST_STANDALONE_BUILD */

dbg_info_t xtcas_dbg = {
	.all = 0, .snd = 0, .wav = 0, .tcas = 0, .xplane = 0, .test = 0,
	.ra = 0, .cpa = 0, .contact = 0, .sl = 0, .threat = 0
};

void
xtcas_log_impl(const char *filename, int line, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	xtcas_log_impl_v(filename, line, fmt, ap);
	va_end(ap);
}

void
xtcas_log_impl_v(const char *filename, int line, const char *fmt, va_list ap)
{
	va_list ap_copy;
	char timedate[32];
	char *buf;
	size_t prefix_len, len;
	struct tm *tm;
	time_t t;

	t = time(NULL);
	tm = localtime(&t);
	VERIFY(strftime(timedate, sizeof (timedate), DATE_FMT, tm) != 0);

	prefix_len = snprintf(NULL, 0, PREFIX_FMT);
	va_copy(ap_copy, ap);
	len = vsnprintf(NULL, 0, fmt, ap_copy);

	buf = (char *)calloc(prefix_len + len + 2, 1);

	(void) snprintf(buf, prefix_len + 1, PREFIX_FMT);
	(void) vsnprintf(&buf[prefix_len], len + 1, fmt, ap);
	strncat(buf, "\n", prefix_len + len + 2);

#ifndef	TEST_STANDALONE_BUILD
	XPLMDebugString(buf);
#endif
	printf("%s", buf);
	fflush(stdout);

	free(buf);
}

#define	MAX_STACK_FRAMES	128
#define	BACKTRACE_STR		"Backtrace is:\n"
#if	defined(__GNUC__) || defined(__clang__)
#define	BACKTRACE_STRLEN	__builtin_strlen(BACKTRACE_STR)
#else	/* !__GNUC__ && !__clang__ */
#define	BACKTRACE_STRLEN	strlen(BACKTRACE_STR)
#endif	/* !__GNUC__ && !__clang__ */

#if	IBM
/*
 * Since while dumping stack we are most likely in a fairly compromised
 * state, we statically pre-allocate these buffers to try and avoid having
 * to call into the VM subsystem.
 */
#define	MAX_SYM_NAME_LEN	1024
static char backtrace_buf[4096] = { 0 };
static char symbol_buf[sizeof (SYMBOL_INFO) +
    MAX_SYM_NAME_LEN * sizeof (TCHAR)];
static char line_buf[sizeof (IMAGEHLP_LINE64)];
#endif	/* IBM */

void
xtcas_log_backtrace(void)
{
#if	IBM

	unsigned frames;
	void *stack[MAX_STACK_FRAMES];
	SYMBOL_INFO *symbol;
	HANDLE process;
	DWORD displacement;
	IMAGEHLP_LINE64 *line;

	process = GetCurrentProcess();
	SymInitialize(process, NULL, TRUE);

	frames = CaptureStackBackTrace(0, MAX_STACK_FRAMES, stack, NULL);

	memset(symbol_buf, 0, sizeof (symbol_buf));
	memset(line_buf, 0, sizeof (line_buf));

	symbol = (SYMBOL_INFO *)symbol_buf;
	symbol->MaxNameLen = MAX_SYM_NAME_LEN - 1;
	symbol->SizeOfStruct = sizeof (SYMBOL_INFO);

	line = (IMAGEHLP_LINE64 *)line_buf;
	line->SizeOfStruct = sizeof (*line);

	backtrace_buf[0] = '\0';
	my_strlcpy(backtrace_buf, BACKTRACE_STR, sizeof (backtrace_buf));

	for (unsigned i = 0; i < frames; i++) {
		/*
		 * This is needed because some dunce at Microsoft thought
		 * it'd be a swell idea to design the SymFromAddr function to
		 * always take a DWORD64 rather than a native pointer size.
		 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
		DWORD64 address = (DWORD64)(stack[i]);
#pragma GCC diagnostic pop
		int fill = strlen(backtrace_buf);

		memset(symbol_buf, 0, sizeof (symbol_buf));
		/*
		 * Try to grab the symbol name from the stored %rip data.
		 */
		if (!SymFromAddr(process, address, 0, symbol)) {
			snprintf(&backtrace_buf[fill], sizeof (backtrace_buf) -
			    fill, "%u %p\n", i, stack[i]);
			continue;
		}
		/*
		 * See if we have debug info available with file names and
		 * line numbers.
		 */
		if (SymGetLineFromAddr64(process, address, &displacement,
		    line)) {
			snprintf(&backtrace_buf[fill], sizeof (backtrace_buf) -
			    fill, "%u: %s (0x%x) [%s:%d]\n", i, symbol->Name,
			    symbol->Address, line->FileName, line->LineNumber);
		} else {
			snprintf(&backtrace_buf[fill], sizeof (backtrace_buf) -
			    fill, "%u: %s - 0x%x\n", i, symbol->Name,
			    symbol->Address);
		}
	}

#ifndef	TEST_STANDALONE_BUILD
	XPLMDebugString(backtrace_buf);
#endif
	fprintf(stderr, "%s", backtrace_buf);

#else	/* !IBM */

	char *msg;
	size_t msg_len;
	void *trace[MAX_STACK_FRAMES];
	size_t i, j, sz;
	char **fnames;

	sz = backtrace(trace, MAX_STACK_FRAMES);
	fnames = backtrace_symbols(trace, sz);

	for (i = 1, msg_len = BACKTRACE_STRLEN; i < sz; i++)
		msg_len += snprintf(NULL, 0, "%s\n", fnames[i]);

	msg = (char *)malloc(msg_len + 1);
	strcpy(msg, BACKTRACE_STR);
	for (i = 1, j = BACKTRACE_STRLEN; i < sz; i++)
		j += sprintf(&msg[j], "%s\n", fnames[i]);

#ifndef	TEST_STANDALONE_BUILD
	XPLMDebugString(msg);
#endif
	fprintf(stderr, "%s", msg);
	fflush(stderr);

	free(msg);
	free(fnames);

#endif	/* !IBM */
}
