/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_XRAAS_RIFF_H_
#define	_XRAAS_RIFF_H_

#include <stdint.h>

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * This macro constructs a 32-bit RIFF chunk ID from a 4-character string
 * (e.g. FOURCC("WAVE")). The returned 32-bit chunk ID is implicitly in
 * little endian, since RIFF files are normally little endian. The RIFF
 * parser performs chunk ID byteswapping internally if it determines that
 * the file's endianness doesn't match ours, so that direct comparisons
 * with this macro's output work regardless of machine & file byte order.
 */
#define	FOURCC(str) \
	((unsigned)((str)[0] | ((str)[1] << 8u) | ((str)[2] << 16u) | \
	((str)[3] << 24u)))

typedef struct riff_chunk {
	uint32_t	fourcc;		/* In machine-native endian */

	/*
	 * Indicates the file was in reverse endian to the machine's native
	 * byte order. Although the fourcc and listcc fields are in native
	 * order, anything in `data' is left as-is by the parser. So use
	 * this field to determine if you need to byteswap `data' contents.
	 */
	bool_t		bswap;
	uint8_t		*data;
	uint32_t	datasz;

	/* Populated if fourcc is RIFF_ID or LIST_ID */
	uint32_t	listcc;		/* In machine-native endian */
	list_t		subchunks;

	list_node_t	node;
} riff_chunk_t;

void riff_free_chunk(riff_chunk_t *c);
riff_chunk_t *riff_parse(uint32_t filetype, uint8_t *buf, size_t bufsz);
riff_chunk_t *riff_find_chunk(riff_chunk_t *topchunk, ...);
char *riff_dump(const riff_chunk_t *topchunk);

#ifdef	__cplusplus
}
#endif

#endif	/* _XRAAS_RIFF_H_ */
