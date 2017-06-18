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

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "assert.h"
#include "list.h"

#include "riff.h"

#define	RIFF_ID	FOURCC("RIFF")
#define	LIST_ID	FOURCC("LIST")
#define	IS_LIST(chunk)	(chunk->fourcc == RIFF_ID || chunk->fourcc == LIST_ID)

/*
 * Frees a RIFF chunk and all its subchunks. Call this function on the
 * top-level chunk returned by riff_parse to free the whole structure.
 */
void
riff_free_chunk(riff_chunk_t *c)
{
	if (c->fourcc == RIFF_ID || c->fourcc == LIST_ID) {
		for (riff_chunk_t *sc = list_head(&c->subchunks); sc != NULL;
		    sc = list_head(&c->subchunks)) {
			list_remove(&c->subchunks, sc);
			riff_free_chunk(sc);
		}
		list_destroy(&c->subchunks);
	}
	free(c);
}

/*
 * Recursive chunk parser. This is the actual worker of riff_parse. It is
 * invoked for every chunk found in the RIFF file.
 */
static riff_chunk_t *
riff_parse_chunk(uint8_t *buf, size_t bufsz, bool_t bswap)
{
	riff_chunk_t *c = calloc(1, sizeof (*c));

	memcpy(&c->fourcc, buf, sizeof (c->fourcc));
	memcpy(&c->datasz, buf + 4, sizeof (c->datasz));
	if (bswap) {
		c->fourcc = BSWAP32(c->fourcc);
		c->datasz = BSWAP32(c->datasz);
		c->bswap = B_TRUE;
	}
	if (c->datasz > bufsz - 8) {
		free(c);
		return (NULL);
	}
	if (c->fourcc == RIFF_ID || c->fourcc == LIST_ID) {
		size_t consumed = 0;
		uint8_t *subbuf;

		/* check there's enough data for a listcc field */
		if (c->datasz < 4) {
			free(c);
			return (NULL);
		}
		memcpy(&c->listcc, buf + 8, sizeof (c->listcc));
		if (bswap)
			c->listcc = BSWAP32(c->listcc);

		/* we exclude the listcc field from our data pointer */
		c->data = buf + 12;
		c->datasz -= 4;

		list_create(&c->subchunks, sizeof (riff_chunk_t),
		    offsetof(riff_chunk_t, node));

		subbuf = c->data;
		while (consumed != c->datasz) {
			riff_chunk_t *sc = riff_parse_chunk(subbuf,
			    c->datasz - consumed, bswap);

			if (sc == NULL) {
				riff_free_chunk(c);
				return (NULL);
			}
			list_insert_tail(&c->subchunks, sc);
			consumed += sc->datasz + 8;
			subbuf += sc->datasz + 8;
			if (IS_LIST(sc)) {
				/* because we exclude the listcc from datasz */
				consumed += 4;
				subbuf += 4;
			}
			if (consumed & 1) {
				/* realign to two-byte boundary */
				consumed++;
				subbuf++;
			}
			ASSERT(consumed <= c->datasz);
			ASSERT(subbuf <= buf + bufsz);
		}
	} else {
		/* plain data chunk */
		c->data = buf + 8;
	}

	return (c);
}

/*
 * Parses a RIFF file and returns its top-level chunk. The parsed structure
 * contains references to the input buffer, so it must not be freed by the
 * caller. Chunk's fourcc's are automatically endian-swapped if the file's
 * endianness doesn't match ours. Chunk contents AREN'T byteswapped. Instead,
 * the caller should use the `bswap' field in the riff_chunk_t to determine
 * if the file was found to be reverse endian and thus if it needs to
 * perform byteswapping of chunk contents.
 *
 * @param filetype The 32-bit little-endian fourcc code identifying the
 *	RIFF file type (e.g. 'WAVE').
 * @param buf Input buffer to parse which contains the entire RIFF file.
 * @param bufsz Input buffer size.
 *
 * If the file isn't of the appropriate type or is malformed, NULL is
 * returned instead.
 */
riff_chunk_t *
riff_parse(uint32_t filetype, uint8_t *buf, size_t bufsz)
{
	bool_t bswap;
	const uint32_t *buf32 = (uint32_t *)buf;
	uint32_t ftype;

	/* check the buffer isn't too small to be a valid RIFF file */
	if (bufsz < 3 * sizeof (uint32_t))
		return (NULL);

	/* make sure the header signature matches & determine endianness */
	if (buf32[0] == RIFF_ID)
		bswap = B_FALSE;
	else if (buf32[0] == BSWAP32(RIFF_ID))
		bswap = B_TRUE;
	else
		return (NULL);

	/* check the file size fits in our buffer */
	if ((bswap ? BSWAP32(buf32[1]) : buf32[1]) > bufsz - 8)
		return (NULL);
	/* check the file type requested by the caller */
	ftype = (bswap ? BSWAP32(buf32[2]) : buf32[2]);
	if (ftype != filetype)
		return (NULL);

	/* now we can be reasonably sure that this is a somewhat valid file */
	return (riff_parse_chunk(buf, bufsz, bswap));
}

/*
 * Locates a specific chunk in a RIFF file. In `topchunk' pass the top-level
 * RIFF file chunk. The remaining arguments must be a 0-terminated list of
 * uint32_t FourCCs of the nested chunks. Don't include the top-level chunk ID.
 * Returns a pointer to a riff_chunk_t (if found), or NULL if not found.
 */
riff_chunk_t *
riff_find_chunk(riff_chunk_t *topchunk, ...)
{
	va_list ap;
	uint32_t fourcc;

	ASSERT(topchunk != NULL);
	ASSERT(topchunk->fourcc == RIFF_ID);

	va_start(ap, topchunk);
	while ((fourcc = va_arg(ap, uint32_t)) != 0) {
		riff_chunk_t *sc;

		ASSERT(fourcc != LIST_ID && fourcc != RIFF_ID);
		if (topchunk->fourcc != LIST_ID &&
		    topchunk->fourcc != RIFF_ID)
			return (NULL);
		for (sc = list_head(&topchunk->subchunks); sc != NULL;
		    sc = list_next(&topchunk->subchunks, sc)) {
			if (sc->fourcc == fourcc || (sc->listcc == fourcc &&
			    (sc->fourcc == LIST_ID || sc->fourcc == RIFF_ID)))
				break;
		}
		if (sc == NULL)
			return (NULL);
		topchunk = sc;
	}
	va_end(ap);

	ASSERT(topchunk != NULL);

	return (topchunk);
}

/*
 * The actual recursive implementation of riff_dump with variable indent.
 */
static char *
riff_dump_impl(const riff_chunk_t *chunk, int indent)
{
	char fourcc[5] = {
	    chunk->fourcc & 0xff, (chunk->fourcc >> 8) & 0xff,
	    (chunk->fourcc >> 16) & 0xff, (chunk->fourcc >> 24) & 0xff, 0
	};
	char listcc[5] = {
	    chunk->listcc & 0xff, (chunk->listcc >> 8) & 0xff,
	    (chunk->listcc >> 16) & 0xff, (chunk->listcc >> 24) & 0xff, 0
	};
	char indent_str[indent + 1];
	int len;
	char *buf;

	memset(indent_str, ' ', indent);
	indent_str[indent] = 0;

#define	DUMP_FMT_ARGS "%s%s%s%s%s%08x\n", indent_str, fourcc, \
	    IS_LIST(chunk) ? " " : "       ", IS_LIST(chunk) ? listcc : "", \
	    IS_LIST(chunk) ? "  " : "", chunk->datasz
	len = snprintf(NULL, 0, DUMP_FMT_ARGS);
	buf = malloc(len + 1);
	snprintf(buf, len + 1, DUMP_FMT_ARGS);
#undef	DUMP_FMT_ARGS

	if (!IS_LIST(chunk))
		return (buf);

	for (const riff_chunk_t *sc = list_head(&chunk->subchunks); sc != NULL;
	    sc = list_next(&chunk->subchunks, sc)) {
		char *subbuf = riff_dump_impl(sc, indent + 2);

		len += strlen(subbuf);
		buf = realloc(buf, len + 1);
		strcat(buf, subbuf);
		free(subbuf);
	}

	return (buf);
}

/*
 * Returns a malloc'd string containing the format dump of a RIFF file
 * (for debugging). The string is formatted into individual lines:
 *
 * RIFF FTYP ChunkSz
 *   CHNK      ChunkSz
 *   LIST CHNK ChunkSz
 *     CHNK      ChunkSz
 *     CHNK      ChunkSz
 *   CHNK      ChunkSz
 *   ...
 */
char *
riff_dump(const riff_chunk_t *topchunk)
{
	return (riff_dump_impl(topchunk, 0));
}
