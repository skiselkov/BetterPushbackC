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
#include <math.h>
#include <ctype.h>

#include "assert.h"
#include "helpers.h"

/*
 * How to turn to get from hdg1 to hdg2 with positive being right and negative
 * being left. Always turns the shortest way around (<= 180 degrees).
 */
double
rel_hdg(double hdg1, double hdg2)
{
	ASSERT(is_valid_hdg(hdg1) && is_valid_hdg(hdg2));
	if (hdg1 > hdg2) {
		if (hdg1 > hdg2 + 180)
			return (360 - hdg1 + hdg2);
		else
			return (-(hdg1 - hdg2));
	} else {
		if (hdg2 > hdg1 + 180)
			return (-(360 - hdg2 + hdg1));
		else
			return (hdg2 - hdg1);
	}
}

/*
 * strlcpy is a BSD function not available on Windows, so we roll a simple
 * version of it ourselves.
 */
void
my_strlcpy(char *restrict dest, const char *restrict src, size_t cap)
{
	dest[cap - 1] = '\0';
	strncpy(dest, src, cap - 1);
}

#if	IBM

/*
 * C getline is a POSIX function, so on Windows, we need to roll our own.
 */
ssize_t
getline(char **line_p, size_t *cap_p, FILE *fp)
{
	ASSERT(line_p != NULL);
	ASSERT(cap_p != NULL);
	ASSERT(fp != NULL);

	char *line = *line_p;
	size_t cap = *cap_p, n = 0;

	do {
		if (n + 1 >= cap) {
			cap += 256;
			line = realloc(line, cap);
		}
		ASSERT(n < cap);
		if (fgets(&line[n], cap - n, fp) == NULL) {
			if (n != 0) {
				break;
			} else {
				*line_p = line;
				*cap_p = cap;
				return (-1);
			}
		}
		n = strlen(line);
	} while (n > 0 && line[n - 1] != '\n');

	*line_p = line;
	*cap_p = cap;

	return (n);
}

#endif	/* IBM */

/*
 * Creates a file path string from individual path components. The
 * components are provided as separate filename arguments and the list needs
 * to be terminated with a NULL argument. The returned string can be freed
 * via free().
 */
char *
mkpathname(const char *comp, ...)
{
	char *res;
	va_list ap;

	va_start(ap, comp);
	res = mkpathname_v(comp, ap);
	va_end(ap);

	return (res);
}

char *
mkpathname_v(const char *comp, va_list ap)
{
	size_t n = 0, len = 0;
	char *str;
	va_list ap2;

	ASSERT(ap != NULL);

	va_copy(ap2, ap);
	len = strlen(comp);
	for (const char *c = va_arg(ap2, const char *); c != NULL;
	    c = va_arg(ap2, const char *)) {
		len += 1 + strlen(c);
	}
	va_end(ap2);

	str = malloc(len + 1);
	n += snprintf(str, len + 1, "%s", comp);
	for (const char *c = va_arg(ap, const char *); c != NULL;
	    c = va_arg(ap, const char *)) {
		ASSERT(n < len);
		n += snprintf(&str[n], len - n + 1, "%c%s", DIRSEP, c);
		/* kill a trailing directory separator */
		if (str[n - 1] == DIRSEP) {
			str[n - 1] = 0;
			n--;
		}
	}

	return (str);
}
