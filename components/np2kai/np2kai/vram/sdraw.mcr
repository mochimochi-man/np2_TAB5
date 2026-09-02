/*
 * ESP32-P4 note: every one of these loops writes through a byte or halfword
 * pointer that the compiler must assume can alias the SDRAW it is also reading
 * its bounds from - so sdraw->width, ->xalign, ->xbytes and ->yalign were being
 * reloaded on every single pixel, the loop bound included. The bodies below
 * hoist them into locals first. Nothing writes to the struct until the function
 * returns, so this is a pure code-generation fix.
 */

// ---- plasma display

// vram off
static void SCRNCALL SDSYM(p_0)(SDRAW sdraw, int maxy) {

	UINT8	*p;
	int		y;
	int		x;

	p = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			for (x=0; x<sd_width; x++) {
				SDSETPIXEL(p, NP2PAL_TEXT2);
				p += sd_xalign;
			}
			p -= sd_xbytes;
		}
		p += sd_yalign;
	} while(++y < maxy);

	sdraw->dst = p;
	sdraw->y = y;
}

// text or grph 1プレーン
static void SCRNCALL SDSYM(p_1)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || !bVFImport) {
					SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(q, x, y);
				}
#else
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
#endif
				q += sd_xalign;
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// text + grph
static void SCRNCALL SDSYM(p_2)(SDRAW sdraw, int maxy) {

const UINT8	*p;
const UINT8	*q;
	UINT8	*r;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->src2;
	r = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || q[x]) {
                    SDSETPIXEL(r, p[x] + q[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(r, x, y);
				}
#else
				SDSETPIXEL(r, p[x] + q[x] + NP2PAL_GRPH);
#endif
				r += sd_xalign;
			}
			r -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += SURFACE_WIDTH;
		r += sd_yalign;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->src2 = q;
	sdraw->dst = r;
	sdraw->y = y;
}

// text + (grph:interleave)
static void SCRNCALL SDSYM(p_ti)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || !bVFImport) {
					SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(q, x, y);
				}
#else
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
#endif
				q += sd_xalign;
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;

		if (sd_dirty[y+1]) {
			for (x=0; x<sd_width; x++) {
				SDSETPIXEL(q, (p[x] >> 4) + NP2PAL_TEXT);
				q += sd_xalign;
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// grph:interleave
static void SCRNCALL SDSYM(p_gi)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable) {
					SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(q, x, y);
				}
#else
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
#endif
				q += sd_xalign;
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;

		if (sd_dirty[y+1]) {
			for (x=0; x<sd_width; x++) {
				SDSETPIXEL(q, NP2PAL_TEXT);
				q += sd_xalign;
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// text + grph:interleave
static void SCRNCALL SDSYM(p_2i)(SDRAW sdraw, int maxy) {

const UINT8	*p;
const UINT8	*q;
	UINT8	*r;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->src2;
	r = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || q[x]) {
                    SDSETPIXEL(r, p[x] + q[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(r, x, y);
				}
#else
				SDSETPIXEL(r, p[x] + q[x] + NP2PAL_GRPH);
#endif
				r += sd_xalign;
			}
			r -= sd_xbytes;
		}
		q += SURFACE_WIDTH;
		r += sd_yalign;

		if (sd_dirty[y+1]) {
			for (x=0; x<sd_width; x++) {
				SDSETPIXEL(r, (q[x] >> 4) + NP2PAL_TEXT);
				r += sd_xalign;
			}
			r -= sd_xbytes;
		}
		p += (SURFACE_WIDTH * 2);
		q += SURFACE_WIDTH;
		r += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->src2 = q;
	sdraw->dst = r;
	sdraw->y = y;
}

//	grph:interleave ex
static void SCRNCALL SDSYM(p_gie)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			sd_dirty[y+1] |= 0xff;
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || !bVFImport) {
					SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(q, x, y);
				}
#else
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
#endif
				q += sd_xalign;
			}
			q -= sd_xbytes;
		}
		q += sd_yalign;

		if (sd_dirty[y+1]) {
			for (x=0; x<sd_width; x++) {
				SDSETPIXEL(q, p[x] + NP2PAL_SKIP);
				q += sd_xalign;
			}
			q -= sd_xbytes;
		}
		p += (SURFACE_WIDTH * 2);
		q += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

//	text + grph:interleave ex
static void SCRNCALL SDSYM(p_2ie)(SDRAW sdraw, int maxy) {

const UINT8	*p;
const UINT8	*q;
	UINT8	*r;
	int		y;
	int		x;
	UINT8	c;

	p = sdraw->src;
	q = sdraw->src2;
	r = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			sd_dirty[y+1] |= 0xff;
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || q[x]) {
                    SDSETPIXEL(r, p[x] + q[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(r, x, y);
				}
#else
				SDSETPIXEL(r, p[x] + q[x] + NP2PAL_GRPH);
#endif
				r += sd_xalign;
			}
			r -= sd_xbytes;
		}
		q += SURFACE_WIDTH;
		r += sd_yalign;

		if (sd_dirty[y+1]) {
			for (x=0; x<sd_width; x++) {
				c = q[x] >> 4;
				if (!c) {
					c = p[x] + NP2PALS_TXT;
				}
				SDSETPIXEL(r, c + NP2PAL_TEXT);
				r += sd_xalign;
			}
			r -= sd_xbytes;
		}
		p += (SURFACE_WIDTH * 2);
		q += SURFACE_WIDTH;
		r += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->src2 = q;
	sdraw->dst = r;
	sdraw->y = y;
}

#if defined(SUPPORT_CRT15KHZ)
// text or grph 1プレーン(15kHz)
static void SCRNCALL SDSYM(p_1d)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		a;
	int		y;
	int		x;
	int		c;

	p = sdraw->src;
	q = sdraw->dst;
	a = sdraw->yalign;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || !bVFImport) {
					c = p[x] + NP2PAL_GRPH;
					SDSETPIXEL(q, c);
					SDSETPIXEL((q + a), c);
				} else {
					VFPUTPIXEL(q,     x, y);
					VFPUTPIXEL(q + a, x, y);
				}
#else
				c = p[x] + NP2PAL_GRPH;
				SDSETPIXEL(q, c);
				SDSETPIXEL((q + a), c);
#endif
				q += sd_xalign;
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += a * 2;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// text + grph (15kHz)
static void SCRNCALL SDSYM(p_2d)(SDRAW sdraw, int maxy) {

const UINT8	*p;
const UINT8	*q;
	UINT8	*r;
	int		a;
	int		y;
	int		x;
	int		c;

	p = sdraw->src;
	q = sdraw->src2;
	r = sdraw->dst;
	a = sdraw->yalign;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			for (x=0; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || q[x]) {
                    c = p[x] + q[x] + NP2PAL_GRPH;
					SDSETPIXEL(r, c);
					SDSETPIXEL((r + a), c);
				} else {
					VFPUTPIXEL(r,     x, y);
					VFPUTPIXEL(r + a, x, y);
				}
#else
				c = p[x] + q[x] + NP2PAL_GRPH;
				SDSETPIXEL(r, c);
				SDSETPIXEL((r + a), c);
#endif
				r += sd_xalign;
			}
			r -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += SURFACE_WIDTH;
		r += a * 2;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->src2 = q;
	sdraw->dst = r;
	sdraw->y = y;
}
#endif

static const SDRAWFN SDSYM(p)[] = {
		SDSYM(p_0),		SDSYM(p_1),		SDSYM(p_1),		SDSYM(p_2),
		SDSYM(p_0),		SDSYM(p_ti),	SDSYM(p_gi),	SDSYM(p_2i),
		SDSYM(p_0),		SDSYM(p_ti),	SDSYM(p_gie),	SDSYM(p_2ie),
#if defined(SUPPORT_CRT15KHZ)
		SDSYM(p_0),		SDSYM(p_1d),	SDSYM(p_1d),	SDSYM(p_2d),
#endif
	};

// ---- normal display

#ifdef SUPPORT_NORMALDISP

// vram off
static void SCRNCALL SDSYM(n_0)(SDRAW sdraw, int maxy) {

	UINT8	*p;
	int		y;
	int		x;

	p = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			SDSETPIXEL(p, NP2PAL_TEXT3);
			for (x=0; x<sd_width; x++) {
				p += sd_xalign;
				SDSETPIXEL(p, NP2PAL_TEXT2);
			}
			p -= sd_xbytes;
		}
		p += sd_yalign;
	} while(++y < maxy);

	sdraw->dst = p;
	sdraw->y = y;
}

// text 1プレーン
static void SCRNCALL SDSYM(n_t)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			SDSETPIXEL(q, (p[0] >> 4) + NP2PAL_TEXT3);
			q += sd_xalign;
			for (x=1; x<sd_width; x++) {
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				q += sd_xalign;
			}
			SDSETPIXEL(q, NP2PAL_TEXT2);
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// grph 1プレーン
static void SCRNCALL SDSYM(n_g)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			SDSETPIXEL(q, NP2PAL_TEXT3);
			for (x=0; x<sd_width; x++) {
				q += sd_xalign;
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable) {
					SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(q, x, y);
				}
#else
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
#endif
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// text + grph
static void SCRNCALL SDSYM(n_2)(SDRAW sdraw, int maxy) {

const UINT8	*p;
const UINT8	*q;
	UINT8	*r;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->src2;
	r = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			SDSETPIXEL(r, (q[0] >> 4) + NP2PAL_TEXT3);
			r += sd_xalign;
			for (x=1; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || q[x]) {
                    SDSETPIXEL(r, p[x-1] + q[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(r, x - 1, y);
				}
#else
				SDSETPIXEL(r, p[x-1] + q[x] + NP2PAL_GRPH);
#endif
				r += sd_xalign;
			}
#if defined(SUPPORT_VIDEOFILTER)
			if(!bVFEnable) {
				SDSETPIXEL(r, p[x-1] + NP2PAL_GRPH);
			} else {
				VFPUTPIXEL(r, x - 1, y);
			}
#else
			SDSETPIXEL(r, p[x-1] + NP2PAL_GRPH);
#endif
			r -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += SURFACE_WIDTH;
		r += sd_yalign;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->src2 = q;
	sdraw->dst = r;
	sdraw->y = y;
}

// text + (grph:interleave)
static void SCRNCALL SDSYM(n_ti)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			SDSETPIXEL(q, (p[0] >> 4) + NP2PAL_TEXT3);
			q += sd_xalign;
			for (x=1; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || !bVFImport) {
					SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(q, x, y);
				}
#else
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
#endif
				q += sd_xalign;
			}
			SDSETPIXEL(q, NP2PAL_GRPH);
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;

		if (sd_dirty[y+1]) {
			SDSETPIXEL(q, (p[0] >> 4) + NP2PAL_TEXT3);
			q += sd_xalign;
			for (x=1; x<sd_width; x++) {
				SDSETPIXEL(q, (p[x] >> 4) + NP2PAL_TEXT);
				q += sd_xalign;
			}
			SDSETPIXEL(q, NP2PAL_TEXT);
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// grph:interleave
static void SCRNCALL SDSYM(n_gi)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			SDSETPIXEL(q, NP2PAL_TEXT3);
			for (x=0; x<sd_width; x++) {
				q += sd_xalign;
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable) {
					SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(q, x, y);
				}
#else
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
#endif
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;

		if (sd_dirty[y+1]) {
			SDSETPIXEL(q, NP2PAL_TEXT3);
			for (x=0; x<sd_width; x++) {
				q += sd_xalign;
				SDSETPIXEL(q, NP2PAL_TEXT);
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// text + grph:interleave
static void SCRNCALL SDSYM(n_2i)(SDRAW sdraw, int maxy) {

const UINT8	*p;
const UINT8	*q;
	UINT8	*r;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->src2;
	r = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			SDSETPIXEL(r, (q[0] >> 4) + NP2PAL_TEXT3);
			r += sd_xalign;
			for (x=1; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || q[x]) {
                    SDSETPIXEL(r, p[x-1] + q[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(r, x - 1, y);
				}
#else
				SDSETPIXEL(r, p[x-1] + q[x] + NP2PAL_GRPH);
#endif
				r += sd_xalign;
			}
#if defined(SUPPORT_VIDEOFILTER)
			if(!bVFEnable) {
				SDSETPIXEL(r, p[x-1] + NP2PAL_GRPH);
			} else {
				VFPUTPIXEL(r, x - 1, y);
			}
#else
			SDSETPIXEL(r, p[x-1] + NP2PAL_GRPH);
#endif
			r -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += SURFACE_WIDTH;
		r += sd_yalign;

		if (sd_dirty[y+1]) {
			SDSETPIXEL(r, (q[0] >> 4) + NP2PAL_TEXT3);
			r += sd_xalign;
			for (x=1; x<sd_width; x++) {
				SDSETPIXEL(r, (q[x] >> 4) + NP2PAL_TEXT);
				r += sd_xalign;
			}
			SDSETPIXEL(r, NP2PAL_TEXT);
			r -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += SURFACE_WIDTH;
		r += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->src2 = q;
	sdraw->dst = r;
	sdraw->y = y;
}

//	grph:interleave ex
static void SCRNCALL SDSYM(n_gie)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		y;
	int		x;

	p = sdraw->src;
	q = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			sd_dirty[y+1] |= 0xff;
			SDSETPIXEL(q, NP2PAL_TEXT3);
			for (x=0; x<sd_width; x++) {
				q += sd_xalign;
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable) {
					SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(q, x, y);
				}
#else
				SDSETPIXEL(q, p[x] + NP2PAL_GRPH);
#endif
			}
			q -= sd_xbytes;
		}
		q += sd_yalign;

		if (sd_dirty[y+1]) {
			SDSETPIXEL(q, NP2PAL_TEXT3);
			for (x=0; x<sd_width; x++) {
				q += sd_xalign;
				SDSETPIXEL(q, p[x] + NP2PAL_SKIP);
			}
			q -= sd_xbytes;
		}
		p += (SURFACE_WIDTH * 2);
		q += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

//	text + grph:interleave ex
static void SCRNCALL SDSYM(n_2ie)(SDRAW sdraw, int maxy) {

const UINT8	*p;
const UINT8	*q;
	UINT8	*r;
	int		y;
	int		x;
	UINT8	c;

	p = sdraw->src;
	q = sdraw->src2;
	r = sdraw->dst;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			sd_dirty[y+1] |= 0xff;
			SDSETPIXEL(r, (q[0] >> 4) + NP2PAL_TEXT3);
			r += sd_xalign;
			for (x=1; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || q[x]) {
                    SDSETPIXEL(r, p[x-1] + q[x] + NP2PAL_GRPH);
				} else {
					VFPUTPIXEL(r, x - 1, y);
				}
#else
				SDSETPIXEL(r, p[x-1] + q[x] + NP2PAL_GRPH);
#endif
				r += sd_xalign;
			}
#if defined(SUPPORT_VIDEOFILTER)
			if(!bVFEnable) {
				SDSETPIXEL(r, p[x-1] + NP2PAL_GRPH);
			} else {
				VFPUTPIXEL(r, x - 1, y);
			}
#else
			SDSETPIXEL(r, p[x-1] + NP2PAL_GRPH);
#endif
			r -= sd_xbytes;
		}
		q += SURFACE_WIDTH;
		r += sd_yalign;

		if (sd_dirty[y+1]) {
			SDSETPIXEL(r, (q[0] >> 4) + NP2PAL_TEXT3);
			r += sd_xalign;
			for (x=1; x<sd_width; x++) {
				c = q[x] >> 4;
				if (!c) {
					c = p[x-1] + NP2PALS_TXT;
				}
				SDSETPIXEL(r, c + NP2PAL_TEXT);
				r += sd_xalign;
			}
			SDSETPIXEL(r, p[x-1] + NP2PAL_SKIP);
			r -= sd_xbytes;
		}
		p += (SURFACE_WIDTH * 2);
		q += SURFACE_WIDTH;
		r += sd_yalign;
		y += 2;
	} while(y < maxy);

	sdraw->src = p;
	sdraw->src2 = q;
	sdraw->dst = r;
	sdraw->y = y;
}

#if defined(SUPPORT_CRT15KHZ)
// text 1プレーン (15kHz)
static void SCRNCALL SDSYM(n_td)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		a;
	int		y;
	int		x;
	int		c;

	p = sdraw->src;
	q = sdraw->dst;
	a = sdraw->yalign;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			c = (p[0] >> 4) + NP2PAL_TEXT3;
			SDSETPIXEL(q, c);
			SDSETPIXEL((q + a), c);
			q += sd_xalign;
			for (x=1; x<sd_width; x++) {
				c = p[x] + NP2PAL_GRPH;
				SDSETPIXEL(q, c);
				SDSETPIXEL((q + a), c);
				q += sd_xalign;
			}
			SDSETPIXEL(q, NP2PAL_TEXT2);
			SDSETPIXEL((q + a), NP2PAL_TEXT2);
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += a * 2;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// grph 1プレーン (15kHz)
static void SCRNCALL SDSYM(n_gd)(SDRAW sdraw, int maxy) {

const UINT8	*p;
	UINT8	*q;
	int		a;
	int		y;
	int		x;
	int		c;

	p = sdraw->src;
	q = sdraw->dst;
	a = sdraw->yalign;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			SDSETPIXEL(q, NP2PAL_TEXT3);
			SDSETPIXEL((q + a), NP2PAL_TEXT3);
			for (x=0; x<sd_width; x++) {
				q += sd_xalign;
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable) {
					c = p[x] + NP2PAL_GRPH;
					SDSETPIXEL(q, c);
					SDSETPIXEL((q + a), c);
				} else {
					VFPUTPIXEL(q, x, y);
					VFPUTPIXEL(q, x, y + 1);
				}
#else
				c = p[x] + NP2PAL_GRPH;
				SDSETPIXEL(q, c);
				SDSETPIXEL((q + a), c);
#endif
			}
			q -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += a * 2;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->dst = q;
	sdraw->y = y;
}

// text + grph (15kHz)
static void SCRNCALL SDSYM(n_2d)(SDRAW sdraw, int maxy) {

const UINT8	*p;
const UINT8	*q;
	UINT8	*r;
	int		a;
	int		y;
	int		x;
	int		c;

	p = sdraw->src;
	q = sdraw->src2;
	r = sdraw->dst;
	a = sdraw->yalign;
	y = sdraw->y;
	const int sd_width = sdraw->width;
	const int sd_xalign = sdraw->xalign;
	const int sd_xbytes = sdraw->xbytes;
	const int sd_yalign = sdraw->yalign;
	UINT8 *sd_dirty = sdraw->dirty;
	(void)sd_width; (void)sd_xalign; (void)sd_xbytes;
	(void)sd_yalign; (void)sd_dirty;
	do {
		if (sd_dirty[y]) {
			c = (q[0] >> 4) + NP2PAL_TEXT3;
			SDSETPIXEL(r, c);
			SDSETPIXEL((r + a), c);
			r += sd_xalign;
			for (x=1; x<sd_width; x++) {
#if defined(SUPPORT_VIDEOFILTER)
				if(!bVFEnable || q[x]) {
                    c = p[x-1] + q[x] + NP2PAL_GRPH;
					SDSETPIXEL(r, c);
					SDSETPIXEL((r + a), c);
				} else {
					VFPUTPIXEL(r, x - 1, y);
					VFPUTPIXEL(r, x - 1, y + 1);
				}
#else
				c = p[x-1] + q[x] + NP2PAL_GRPH;
				SDSETPIXEL(r, c);
				SDSETPIXEL((r + a), c);
#endif
				r += sd_xalign;
			}
#if defined(SUPPORT_VIDEOFILTER)
			if(!bVFEnable) {
				c = p[x-1] + NP2PAL_GRPH;
				SDSETPIXEL(r, c);
				SDSETPIXEL((r + a), c);
			} else {
				VFPUTPIXEL(r, x - 1, y);
				VFPUTPIXEL(r, x - 1, y + 1);
			}
#else
			c = p[x-1] + NP2PAL_GRPH;
			SDSETPIXEL(r, c);
			SDSETPIXEL((r + a), c);
#endif
			r -= sd_xbytes;
		}
		p += SURFACE_WIDTH;
		q += SURFACE_WIDTH;
		r += a * 2;
	} while(++y < maxy);

	sdraw->src = p;
	sdraw->src2 = q;
	sdraw->dst = r;
	sdraw->y = y;
}
#endif

static const SDRAWFN SDSYM(n)[] = {
		SDSYM(n_0),		SDSYM(n_t),		SDSYM(n_g),		SDSYM(n_2),
		SDSYM(n_0),		SDSYM(n_ti),	SDSYM(n_gi),	SDSYM(n_2i),
		SDSYM(n_0),		SDSYM(n_ti),	SDSYM(n_gie),	SDSYM(n_2ie),
#if defined(SUPPORT_CRT15KHZ)
		SDSYM(n_0),		SDSYM(n_td),	SDSYM(n_gd),	SDSYM(n_2d),
#endif
	};
#endif

