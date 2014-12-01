/*-
 * Copyright (c) 2014 Adrian Chadd<adrian@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/* $FreeBSD$ */

#ifndef	__IWA_DEBUG_H__
#define	__IWA_DEBUG_H__

/* XXX until there's opt_iwa.h */
#define	IWA_DEBUG

#define	IWA_DPRINTF(sc, m, fmt, ...)		\
	do {					\
		if ((sc)->sc_debug & (m))	\
			device_printf((sc)->sc_dev, fmt, __VA_ARGS__); \
	} while (0)

enum {
	IWA_DEBUG_FIRMWARE = 0x00000001,
	IWA_DEBUG_RESET = 0x00000002,
	IWA_DEBUG_TX = 0x00000004,
	IWA_DEBUG_RX = 0x00000008,

	IWA_DEBUG_TRACE = 0x40000000,
};

#endif	/* __IWA_DEBUG_H__ */
