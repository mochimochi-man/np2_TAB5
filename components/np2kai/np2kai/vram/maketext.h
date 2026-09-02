
enum {
#ifdef NP2_ESP32
	/* Must equal SURFACE_WIDTH/8. ESP32 shrank SURFACE_WIDTH to 640 (PSRAM
	   budget) => 640/8 = 80. Upstream 128 assumes SURFACE_WIDTH=1024; with 640
	   the renderer wrote 128*8=1024 bytes/scanline into a 640-stride buffer,
	   so every scanline drifted 384 bytes -> overlapping/garbled text. */
	TEXTXMAX		= 80,
#else
	TEXTXMAX		= 128,
#endif
	TEXTYMAX		= 400,

	TXTATR_ST		= 0x01,		// ~シークレット
	TXTATR_BL		= 0x02,		// ブリンク
	TXTATR_RV		= 0x04,		// リバース
	TXTATR_UL		= 0x08,		// アンダーライン
	TXTATR_VL		= 0x10,		// バーチカルライン
	TXTATR_BG		= 0x10,		// 簡易グラフ
	TEXTATR_RGB		= 0xe0		// ビット並びはGRBの順
};

typedef struct {
	UINT8	timing;
	UINT8	count;
	UINT8	renewal;
	UINT8	gaiji;
	UINT8	attr;
	UINT8	curdisp;
	UINT8	curdisplast;
	UINT8	blink;
	UINT8	blinkdisp;
	UINT16	curpos;
} TRAM_T;


#ifdef __cplusplus
extern "C" {
#endif

extern	TRAM_T	tramflag;

void maketext_initialize(void);
void maketext_reset(void);
UINT8 maketext_curblink(void);
void maketext(int text_renewal);
void maketext40(int text_renewal);

#ifdef __cplusplus
}
#endif

