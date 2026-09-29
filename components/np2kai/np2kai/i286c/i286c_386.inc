// i286c 386 extension - the parts of the 80386 instruction set that real-mode
// programs use, added on top of the 286 core.
//
// Some PC-98 software written for the 386 never enters protected mode; it only
// uses 32-bit arithmetic (the 66h operand-size prefix, as a C compiler emits for
// long), near conditional jumps (0F 8x) and a handful of other 0Fh opcodes. On
// the plain 286 core every one of those is an invalid opcode, INT 6, which on
// the PC-98 is also the STOP key vector - so a game shows its "do not press
// STOP" message and stops. The instruction list was taken by running such
// programs on the IA-32 core and recording every 386-only instruction they
// executed: Touhou Fuumaroku (TH02), Z-frame FINAL, STORM CALIBER GAIDEN,
// COUNTER ATTACK OF VERRIER THE TRIAL and Burning Star EX.
//
// What is here, real mode only:
//   66h  32-bit operand size for ALU, INC/DEC, PUSH/POP(AD), MOV, LEA, XCHG,
//        CWDE/CDQ, PUSHFD/POPFD, shifts and rotates, MUL/IMUL/DIV/IDIV, the
//        string instructions (with REP), MOVZX/MOVSX, SHLD/SHRD, BT*, BSF/BSR
//   0Fh  Jcc rel16, SETcc, MOVZX/MOVSX, IMUL r,rm, BT*, BSF/BSR, SHLD/SHRD,
//        PUSH/POP FS/GS, LSS/LFS/LGS
//   64h/65h  FS/GS segment override, and MOV to and from FS/GS
//   67h  32-bit addressing (base, scaled index, 32-bit displacement)
// Not here: protected mode, paging, control registers. Those stay INT 6, as
// before.
//
// The CPU still identifies as a 286: FLAGS bits 12-15 behave as they did, so a
// program that tests for a 386 is told no. That keeps DOS drivers such as
// EMM386 from trying to enter protected mode on a core that cannot do it; only
// software that uses 386 instructions without asking gets them.
//
// This file is compiled through i286c_386.c; the opcode tables in i286c_mn.c
// point at the handlers at the end.

// Off = the plain 286 again: every opcode here is INT 6, as it was before.
int		i386x_enabled = 1;
UINT16	i386x_hi[8];			// upper halves of EAX..EDI
UINT16	i386x_fs, i386x_gs;
UINT32	i386x_fs_base, i386x_gs_base;
static UINT16 i386x_ip0;		// IP of the instruction's first byte, for faults

void i386x_reset(void) {

	ZeroMemory(i386x_hi, sizeof(i386x_hi));
	i386x_fs = 0;
	i386x_gs = 0;
	i386x_fs_base = 0;
	i386x_gs_base = 0;
}

#define	X_R16(i)	(((UINT16 *)&I286_REG) + (i))

static UINT32 x_r32(UINT i) {

	return ((UINT32)i386x_hi[i] << 16) | *X_R16(i);
}

static void x_w32(UINT i, UINT32 v) {

	*X_R16(i) = (UINT16)v;
	i386x_hi[i] = (UINT16)(v >> 16);
}

static UINT32 x_rd32(UINT32 a) {

	return i286_memoryread_w(a) | ((UINT32)i286_memoryread_w(a + 2) << 16);
}

static void x_wr32(UINT32 a, UINT32 v) {

	i286_memorywrite_w(a, (REG16)v);
	i286_memorywrite_w(a + 2, (REG16)(v >> 16));
}

static UINT32 x_imm32(void) {

	UINT	lo;
	UINT	hi;

	GET_PCWORD(lo);
	GET_PCWORD(hi);
	return (UINT32)lo | ((UINT32)hi << 16);
}

static UINT32 x_imm8s(void) {

	UINT	b;

	GET_PCBYTE(b);
	return (UINT32)(SINT32)(SINT8)b;
}

static void x_push32(UINT32 v) {

	I286_SP -= 4;
	x_wr32(SS_BASE + I286_SP, v);
}

static UINT32 x_pop32(void) {

	UINT32	v;

	v = x_rd32(SS_BASE + I286_SP);
	I286_SP += 4;
	return v;
}

// ---- operands: a ModRM byte, then register or memory

typedef struct {
	UINT	m;			// the ModRM byte
	UINT	reg;		// bits 5-3
	UINT32	madr;		// linear address when m < 0xc0
} XMODRM;

static void x_modrm(XMODRM *x) {

	GET_PCBYTE(x->m);
	x->reg = (x->m >> 3) & 7;
	x->madr = (x->m < 0xc0) ? CALC_EA(x->m) : 0;
}

static UINT32 x_get32(const XMODRM *x) {

	if (x->m >= 0xc0) {
		I286_WORKCLOCK(2);
		return x_r32(x->m & 7);
	}
	I286_WORKCLOCK(5);
	return x_rd32(x->madr);
}

static void x_put32(const XMODRM *x, UINT32 v) {

	if (x->m >= 0xc0) {
		x_w32(x->m & 7, v);
	}
	else {
		x_wr32(x->madr, v);
	}
}

static UINT32 x_get16(const XMODRM *x) {

	if (x->m >= 0xc0) {
		return *X_R16(x->m & 7);
	}
	return i286_memoryread_w(x->madr);
}

static void x_put16(const XMODRM *x, UINT32 v) {

	if (x->m >= 0xc0) {
		*X_R16(x->m & 7) = (UINT16)v;
	}
	else {
		i286_memorywrite_w(x->madr, (REG16)v);
	}
}

static UINT32 x_get8(const XMODRM *x) {

	if (x->m >= 0xc0) {
		return *(REG8_B20(x->m));
	}
	return i286_memoryread(x->madr);
}

static void x_put8(const XMODRM *x, UINT32 v) {

	if (x->m >= 0xc0) {
		*(REG8_B20(x->m)) = (UINT8)v;
	}
	else {
		i286_memorywrite(x->madr, (REG8)v);
	}
}

// ---- flags
// I286_FLAGL carries C, P, A, Z and S; overflow lives apart in I286_OV, as it
// does everywhere else in this core.

static UINT8 x_szp(UINT32 r, UINT32 sign) {

	UINT8	f;

	f = (UINT8)(BYTESZPF(r & 0xff) & P_FLAG);
	if ((r & ((sign << 1) - 1)) == 0) {
		f |= Z_FLAG;
	}
	if (r & sign) {
		f |= S_FLAG;
	}
	return f;
}

#define	SZP32(r)	x_szp((r), 0x80000000)
#define	SZP16(r)	x_szp((r) & 0xffff, 0x8000)

// ADD OR ADC SBB AND SUB XOR CMP, as numbered by the ModRM reg field
static UINT32 x_alu32(UINT kind, UINT32 d, UINT32 s) {

	UINT32	r;
	UINT	c;

	switch (kind) {
		case 0:
			r = d + s;
			I286_OV = ((r ^ s) & (r ^ d)) >> 31;
			I286_FLAGL = (UINT8)(((r ^ d ^ s) & A_FLAG) | SZP32(r) | (r < d));
			break;

		case 2:
			c = I286_FLAGL & C_FLAG;
			r = d + s + c;
			I286_OV = ((r ^ s) & (r ^ d)) >> 31;
			I286_FLAGL = (UINT8)(((r ^ d ^ s) & A_FLAG) | SZP32(r) |
									(c ? (r <= d) : (r < d)));
			break;

		case 3:
			c = I286_FLAGL & C_FLAG;
			r = d - s - c;
			I286_OV = ((d ^ r) & (d ^ s)) >> 31;
			I286_FLAGL = (UINT8)(((r ^ d ^ s) & A_FLAG) | SZP32(r) |
									(c ? (d <= s) : (d < s)));
			break;

		case 5:
		case 7:
			r = d - s;
			I286_OV = ((d ^ r) & (d ^ s)) >> 31;
			I286_FLAGL = (UINT8)(((r ^ d ^ s) & A_FLAG) | SZP32(r) | (d < s));
			break;

		case 1:
			r = d | s;
			I286_OV = 0;
			I286_FLAGL = SZP32(r);
			break;

		case 4:
			r = d & s;
			I286_OV = 0;
			I286_FLAGL = SZP32(r);
			break;

		default:
			r = d ^ s;
			I286_OV = 0;
			I286_FLAGL = SZP32(r);
			break;
	}
	return r;
}

// ROL ROR RCL RCR SHL SHR SAL SAR. `bits` is 16 or 32; the count is masked
// to five bits as a 386 does.
static UINT32 x_shift(UINT kind, UINT32 d, UINT cnt, UINT bits) {

	UINT32	mask;
	UINT32	sign;
	UINT32	r;
	UINT32	c;
	UINT	i;

	cnt &= 31;
	if (cnt == 0) {
		return d;
	}
	mask = (bits == 32) ? 0xffffffff : 0xffff;
	sign = (bits == 32) ? 0x80000000 : 0x8000;
	d &= mask;
	I286_WORKCLOCK(3);
	switch (kind) {
		case 0:				// ROL
			i = cnt % bits;
			r = i ? (((d << i) | (d >> (bits - i))) & mask) : d;
			c = r & 1;
			I286_OV = ((r & sign) != 0) ^ c;
			I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | c);
			return r;

		case 1:				// ROR
			i = cnt % bits;
			r = i ? (((d >> i) | (d << (bits - i))) & mask) : d;
			c = (r & sign) != 0;
			I286_OV = c ^ ((r & (sign >> 1)) != 0);
			I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | c);
			return r;

		case 2:				// RCL
			c = I286_FLAGL & C_FLAG;
			for (i = 0; i < cnt; i++) {
				UINT32 out = (d & sign) != 0;
				d = ((d << 1) | c) & mask;
				c = out;
			}
			I286_OV = ((d & sign) != 0) ^ c;
			I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | c);
			return d;

		case 3:				// RCR
			c = I286_FLAGL & C_FLAG;
			I286_OV = ((d & sign) != 0) ^ c;
			for (i = 0; i < cnt; i++) {
				UINT32 out = d & 1;
				d = (d >> 1) | (c ? sign : 0);
				c = out;
			}
			I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | c);
			return d;

		case 4:				// SHL
		case 6:
			if (cnt > bits) {
				c = 0;
				r = 0;
			}
			else {
				c = (d >> (bits - cnt)) & 1;
				r = (cnt == bits) ? 0 : ((d << cnt) & mask);
			}
			I286_OV = ((r & sign) != 0) ^ c;
			I286_FLAGL = (UINT8)(x_szp(r, sign) | c);
			return r;

		case 5:				// SHR
			c = (cnt > bits) ? 0 : ((d >> (cnt - 1)) & 1);
			r = (cnt >= bits) ? 0 : (d >> cnt);
			I286_OV = (d & sign) != 0;
			I286_FLAGL = (UINT8)(x_szp(r, sign) | c);
			return r;

		default:			// SAR
			if (bits == 16) {
				SINT32 sd = (SINT16)d;
				c = (sd >> ((cnt > 16 ? 16 : cnt) - 1)) & 1;
				r = (UINT32)(sd >> (cnt > 15 ? 15 : cnt)) & mask;
			}
			else {
				SINT32 sd = (SINT32)d;
				c = (sd >> (cnt - 1)) & 1;
				r = (UINT32)(sd >> cnt);
			}
			I286_OV = 0;
			I286_FLAGL = (UINT8)(x_szp(r, sign) | c);
			return r;
	}
}

static int x_cond(UINT cc) {

	UINT	f;
	int		r;

	f = I286_FLAGL;
	switch (cc >> 1) {
		case 0:	r = (I286_OV != 0); break;
		case 1:	r = (f & C_FLAG) != 0; break;
		case 2:	r = (f & Z_FLAG) != 0; break;
		case 3:	r = (f & (C_FLAG | Z_FLAG)) != 0; break;
		case 4:	r = (f & S_FLAG) != 0; break;
		case 5:	r = (f & P_FLAG) != 0; break;
		case 6:	r = ((f & S_FLAG) != 0) != (I286_OV != 0); break;
		default:
			r = ((f & Z_FLAG) != 0) ||
				(((f & S_FLAG) != 0) != (I286_OV != 0));
			break;
	}
	return (cc & 1) ? !r : r;
}

static void x_invalid(void) {

	INT_NUM(6, i386x_ip0);
}

static void x_divide_error(void) {

	INT_NUM(0, i386x_ip0);
}

static void x_setseg(UINT which, UINT16 sel) {

	switch (which) {
		case 0:
			I286_ES = sel;
			ES_BASE = SEGSELECT(sel);
			break;

		case 2:
			I286_SS = sel;
			SS_BASE = SEGSELECT(sel);
			SS_FIX = SS_BASE;
			break;

		case 3:
			I286_DS = sel;
			DS_BASE = SEGSELECT(sel);
			DS_FIX = DS_BASE;
			break;

		case 4:
			i386x_fs = sel;
			i386x_fs_base = (UINT32)sel << 4;
			break;

		case 5:
			i386x_gs = sel;
			i386x_gs_base = (UINT32)sel << 4;
			break;
	}
}

// ---- 0Fh, for both operand sizes

static void x_bt_apply(UINT kind, const XMODRM *x, UINT32 bit, int o32) {

	UINT32	v;
	UINT32	b;

	v = o32 ? x_get32(x) : x_get16(x);
	b = (UINT32)1 << bit;
	I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | ((v & b) ? C_FLAG : 0));
	switch (kind) {
		case 1: v |= b; break;			// BTS
		case 2: v &= ~b; break;			// BTR
		case 3: v ^= b; break;			// BTC
		default: return;				// BT
	}
	if (o32) {
		x_put32(x, v);
	}
	else {
		x_put16(x, v);
	}
}

static void x_bt_reg(UINT kind, int o32) {

	XMODRM	x;
	SINT32	off;
	UINT32	bits;

	x_modrm(&x);
	bits = o32 ? 32 : 16;
	off = o32 ? (SINT32)x_r32(x.reg) : (SINT32)(SINT16)*X_R16(x.reg);
	if (x.m < 0xc0) {
		// A register bit offset reaches outside the operand in memory
		SINT32 unit = (off >= 0) ? (off / (SINT32)bits) : -((-off + (SINT32)bits - 1) / (SINT32)bits);
		x.madr += (UINT32)(unit * (SINT32)(bits / 8));
	}
	x_bt_apply(kind, &x, (UINT32)off & (bits - 1), o32);
	I286_WORKCLOCK(6);
}

static void x_shxd(int right, int bycl, int o32) {

	XMODRM	x;
	UINT32	d;
	UINT32	s;
	UINT32	r;
	UINT32	c;
	UINT	cnt;
	UINT	bits;
	UINT32	mask;
	UINT32	sign;

	x_modrm(&x);
	bits = o32 ? 32 : 16;
	mask = o32 ? 0xffffffff : 0xffff;
	sign = o32 ? 0x80000000 : 0x8000;
	d = o32 ? x_get32(&x) : x_get16(&x);
	s = (o32 ? x_r32(x.reg) : *X_R16(x.reg)) & mask;
	if (bycl) {
		cnt = I286_CL;
	}
	else {
		GET_PCBYTE(cnt);
	}
	cnt &= 31;
	if (cnt == 0 || cnt > bits) {
		return;
	}
	if (!right) {
		r = (UINT32)((((UINT64)d << bits) | s) << cnt >> bits) & mask;
		c = (d >> (bits - cnt)) & 1;
	}
	else {
		r = (UINT32)((((UINT64)s << bits) | d) >> cnt) & mask;
		c = (d >> (cnt - 1)) & 1;
	}
	I286_OV = ((r ^ d) & sign) != 0;
	I286_FLAGL = (UINT8)(x_szp(r, sign) | c);
	if (o32) {
		x_put32(&x, r);
	}
	else {
		x_put16(&x, r);
	}
	I286_WORKCLOCK(3);
}

void i386x_0f(UINT op2, UINT16 ip0, int o32) {

	if (!i386x_enabled) {
		INT_NUM(6, ip0);
		return;
	}

	XMODRM	x;
	UINT32	v;
	UINT32	s;

	i386x_ip0 = ip0;
	if ((op2 & 0xf0) == 0x80) {					// Jcc rel16 / rel32
		SINT32 disp;
		if (o32) {
			disp = (SINT32)x_imm32();
		}
		else {
			UINT w;
			GET_PCWORD(w);
			disp = (SINT16)w;
		}
		if (x_cond(op2 & 15)) {
			I286_WORKCLOCK(7);
			I286_IP = (UINT16)(I286_IP + disp);
		}
		else {
			I286_WORKCLOCK(3);
		}
		return;
	}
	if ((op2 & 0xf0) == 0x90) {					// SETcc rm8
		x_modrm(&x);
		x_put8(&x, x_cond(op2 & 15) ? 1 : 0);
		I286_WORKCLOCK(4);
		return;
	}
	switch (op2) {
		case 0xa0:								// push fs
			if (o32) x_push32(i386x_fs); else REGPUSH(i386x_fs, 3);
			return;

		case 0xa1:								// pop fs
			v = o32 ? (x_pop32() & 0xffff) : 0;
			if (!o32) {
				REGPOP(v, 5);
			}
			x_setseg(4, (UINT16)v);
			return;

		case 0xa8:								// push gs
			if (o32) x_push32(i386x_gs); else REGPUSH(i386x_gs, 3);
			return;

		case 0xa9:								// pop gs
			v = o32 ? (x_pop32() & 0xffff) : 0;
			if (!o32) {
				REGPOP(v, 5);
			}
			x_setseg(5, (UINT16)v);
			return;

		case 0xa3: x_bt_reg(0, o32); return;	// bt
		case 0xab: x_bt_reg(1, o32); return;	// bts
		case 0xb3: x_bt_reg(2, o32); return;	// btr
		case 0xbb: x_bt_reg(3, o32); return;	// btc

		case 0xba:								// bt* rm, imm8
			x_modrm(&x);
			GET_PCBYTE(s);
			if (x.reg < 4) {
				x_invalid();
				return;
			}
			x_bt_apply(x.reg - 4, &x, s & (o32 ? 31 : 15), o32);
			I286_WORKCLOCK(6);
			return;

		case 0xa4: x_shxd(0, 0, o32); return;	// shld imm8
		case 0xa5: x_shxd(0, 1, o32); return;	// shld cl
		case 0xac: x_shxd(1, 0, o32); return;	// shrd imm8
		case 0xad: x_shxd(1, 1, o32); return;	// shrd cl

		case 0xaf:								// imul r, rm
			x_modrm(&x);
			if (o32) {
				SINT64 p = (SINT64)(SINT32)x_get32(&x) * (SINT32)x_r32(x.reg);
				x_w32(x.reg, (UINT32)p);
				I286_OV = (p != (SINT32)p);
			}
			else {
				SINT32 p = (SINT32)(SINT16)x_get16(&x) * (SINT16)*X_R16(x.reg);
				*X_R16(x.reg) = (UINT16)p;
				I286_OV = (p != (SINT16)p);
			}
			I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | (I286_OV ? C_FLAG : 0));
			I286_WORKCLOCK(20);
			return;

		case 0xb2:								// lss
		case 0xb4:								// lfs
		case 0xb5:								// lgs
			x_modrm(&x);
			if (x.m >= 0xc0) {
				x_invalid();
				return;
			}
			if (o32) {
				v = x_rd32(x.madr);
				s = i286_memoryread_w(x.madr + 4);
				x_w32(x.reg, v);
			}
			else {
				v = i286_memoryread_w(x.madr);
				s = i286_memoryread_w(x.madr + 2);
				*X_R16(x.reg) = (UINT16)v;
			}
			x_setseg((op2 == 0xb2) ? 2 : (op2 == 0xb4) ? 4 : 5, (UINT16)s);
			I286_WORKCLOCK(7);
			return;

		case 0xb6:								// movzx r, rm8
		case 0xbe:								// movsx r, rm8
			x_modrm(&x);
			v = x_get8(&x);
			if (op2 == 0xbe) {
				v = (UINT32)(SINT32)(SINT8)v;
			}
			if (o32) x_w32(x.reg, v); else *X_R16(x.reg) = (UINT16)v;
			I286_WORKCLOCK(3);
			return;

		case 0xb7:								// movzx r, rm16
		case 0xbf:								// movsx r, rm16
			x_modrm(&x);
			v = x_get16(&x);
			if (op2 == 0xbf) {
				v = (UINT32)(SINT32)(SINT16)v;
			}
			if (o32) x_w32(x.reg, v); else *X_R16(x.reg) = (UINT16)v;
			I286_WORKCLOCK(3);
			return;

		case 0xbc:								// bsf
		case 0xbd:								// bsr
			x_modrm(&x);
			v = o32 ? x_get32(&x) : x_get16(&x);
			if (v == 0) {
				I286_FLAGL |= Z_FLAG;
			}
			else {
				UINT32 n;
				if (op2 == 0xbc) {
					for (n = 0; !(v & 1); n++) v >>= 1;
				}
				else {
					for (n = o32 ? 31 : 15; !(v & ((UINT32)1 << n)); n--) ;
				}
				I286_FLAGL &= ~Z_FLAG;
				if (o32) x_w32(x.reg, n); else *X_R16(x.reg) = (UINT16)n;
			}
			I286_WORKCLOCK(10);
			return;
	}
	x_invalid();
}

// ---- 66h: 32-bit operand size

static void x_popfd(void) {

	UINT	flag;

	flag = x_pop32() & 0xffff;
	I286_OV = flag & O_FLAG;
	I286_FLAG = (UINT16)(flag & (0xfff ^ O_FLAG));
#if defined(VAEG_FIX)
	I286_TRAP = ((flag & 0x100) == 0x100);
#else
	I286_TRAP = ((flag & 0x300) == 0x300);
#endif
	I286_WORKCLOCK(5);
	if ((I286_TRAP) || ((flag & I_FLAG) && (PICEXISTINTR))) {
		I286IRQCHECKTERM
	}
}

static void x_string32(UINT op, UINT rep) {

	int		dir;
	UINT32	d;
	UINT32	s;

	dir = (I286_FLAG & D_FLAG) ? -4 : 4;
	for (;;) {
		if (rep && I286_CX == 0) {
			break;
		}
		switch (op) {
			case 0xa5:							// movsd
				x_wr32(ES_BASE + I286_DI, x_rd32(DS_FIX + I286_SI));
				I286_SI += dir;
				I286_DI += dir;
				break;

			case 0xa7:							// cmpsd
				d = x_rd32(DS_FIX + I286_SI);
				s = x_rd32(ES_BASE + I286_DI);
				x_alu32(7, d, s);
				I286_SI += dir;
				I286_DI += dir;
				break;

			case 0xab:							// stosd
				x_wr32(ES_BASE + I286_DI, x_r32(0));
				I286_DI += dir;
				break;

			case 0xad:							// lodsd
				x_w32(0, x_rd32(DS_FIX + I286_SI));
				I286_SI += dir;
				break;

			default:							// scasd
				x_alu32(7, x_r32(0), x_rd32(ES_BASE + I286_DI));
				I286_DI += dir;
				break;
		}
		I286_WORKCLOCK(4);
		if (!rep) {
			break;
		}
		I286_CX--;
		if (op == 0xa7 || op == 0xaf) {
			if ((rep == 1) && !(I286_FLAGL & Z_FLAG)) break;
			if ((rep == 2) && (I286_FLAGL & Z_FLAG)) break;
		}
	}
}

// Opcodes whose meaning does not depend on the operand size: the prefix is
// simply ignored and the 16-bit handler runs.
static int x_size_neutral(UINT op) {

	switch (op) {
		case 0x00: case 0x02: case 0x04: case 0x08: case 0x0a: case 0x0c:
		case 0x10: case 0x12: case 0x14: case 0x18: case 0x1a: case 0x1c:
		case 0x20: case 0x22: case 0x24: case 0x27: case 0x28: case 0x2a:
		case 0x2c: case 0x2f: case 0x30: case 0x32: case 0x34: case 0x37:
		case 0x38: case 0x3a: case 0x3c: case 0x3f:
		case 0x80: case 0x82: case 0x84: case 0x86: case 0x88: case 0x8a:
		case 0x8c: case 0x8e:
		case 0x9b: case 0x9e: case 0x9f:
		case 0xa0: case 0xa2: case 0xa4: case 0xa6: case 0xa8: case 0xaa:
		case 0xac: case 0xae:
		case 0xc0: case 0xc6: case 0xd0: case 0xd2: case 0xd4: case 0xd5:
		case 0xd7: case 0xe4: case 0xe6: case 0xec: case 0xee:
		case 0xf5: case 0xf6: case 0xf8: case 0xf9: case 0xfa: case 0xfb:
		case 0xfc: case 0xfd: case 0xfe:
			return 1;
	}
	return ((op >= 0x70) && (op < 0x80)) || ((op >= 0xb0) && (op < 0xb8)) ||
			(op == 0xeb);
}

static void x_exec32(UINT op, UINT rep) {

	XMODRM	x;
	UINT32	d;
	UINT32	s;
	UINT32	r;

	if (x_size_neutral(op)) {
		if (rep == 1) i286op_repe[op]();
		else if (rep == 2) i286op_repne[op]();
		else i286op[op]();
		return;
	}
	if ((op < 0x40) && ((op & 7) < 6)) {
		UINT kind = op >> 3;
		switch (op & 7) {
			case 1:								// alu rm32, r32
				x_modrm(&x);
				d = x_get32(&x);
				r = x_alu32(kind, d, x_r32(x.reg));
				if (kind != 7) x_put32(&x, r);
				return;

			case 3:								// alu r32, rm32
				x_modrm(&x);
				s = x_get32(&x);
				r = x_alu32(kind, x_r32(x.reg), s);
				if (kind != 7) x_w32(x.reg, r);
				return;

			case 5:								// alu eax, imm32
				I286_WORKCLOCK(3);
				s = x_imm32();
				r = x_alu32(kind, x_r32(0), s);
				if (kind != 7) x_w32(0, r);
				return;
		}
	}
	if ((op >= 0x40) && (op < 0x50)) {			// inc / dec r32
		UINT i = op & 7;
		UINT32 c = I286_FLAGL & C_FLAG;
		d = x_r32(i);
		if (op < 0x48) {
			r = d + 1;
			I286_OV = (r == 0x80000000);
		}
		else {
			r = d - 1;
			I286_OV = (d == 0x80000000);
		}
		x_w32(i, r);
		I286_FLAGL = (UINT8)(((r ^ d ^ 1) & A_FLAG) | SZP32(r) | c);
		I286_WORKCLOCK(2);
		return;
	}
	if ((op >= 0x50) && (op < 0x58)) {			// push r32
		I286_WORKCLOCK(3);
		x_push32(x_r32(op & 7));
		return;
	}
	if ((op >= 0x58) && (op < 0x60)) {			// pop r32
		I286_WORKCLOCK(5);
		x_w32(op & 7, x_pop32());
		return;
	}
	if ((op >= 0x91) && (op < 0x98)) {			// xchg eax, r32
		I286_WORKCLOCK(3);
		d = x_r32(0);
		x_w32(0, x_r32(op & 7));
		x_w32(op & 7, d);
		return;
	}
	if ((op >= 0xb8) && (op < 0xc0)) {			// mov r32, imm32
		I286_WORKCLOCK(2);
		x_w32(op & 7, x_imm32());
		return;
	}
	switch (op) {
		case 0x0f:
			{
				UINT op2;
				GET_PCBYTE(op2);
				i386x_0f(op2, i386x_ip0, 1);
			}
			return;

		case 0x60:								// pushad
			d = x_r32(4);
			x_push32(x_r32(0));
			x_push32(x_r32(1));
			x_push32(x_r32(2));
			x_push32(x_r32(3));
			x_push32(d);
			x_push32(x_r32(5));
			x_push32(x_r32(6));
			x_push32(x_r32(7));
			I286_WORKCLOCK(18);
			return;

		case 0x61:								// popad
			x_w32(7, x_pop32());
			x_w32(6, x_pop32());
			x_w32(5, x_pop32());
			x_pop32();
			x_w32(3, x_pop32());
			x_w32(2, x_pop32());
			x_w32(1, x_pop32());
			x_w32(0, x_pop32());
			I286_WORKCLOCK(24);
			return;

		case 0x68:								// push imm32
			I286_WORKCLOCK(3);
			x_push32(x_imm32());
			return;

		case 0x6a:								// push imm8
			I286_WORKCLOCK(3);
			x_push32(x_imm8s());
			return;

		case 0x69:								// imul r32, rm32, imm32
		case 0x6b:								// imul r32, rm32, imm8
			{
				SINT64 p;
				x_modrm(&x);
				d = x_get32(&x);
				s = (op == 0x69) ? x_imm32() : x_imm8s();
				p = (SINT64)(SINT32)d * (SINT32)s;
				x_w32(x.reg, (UINT32)p);
				I286_OV = (p != (SINT32)p);
				I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | (I286_OV ? C_FLAG : 0));
				I286_WORKCLOCK(20);
			}
			return;

		case 0x81:								// alu rm32, imm32
		case 0x83:								// alu rm32, imm8
			x_modrm(&x);
			d = x_get32(&x);
			s = (op == 0x81) ? x_imm32() : x_imm8s();
			r = x_alu32(x.reg, d, s);
			if (x.reg != 7) x_put32(&x, r);
			return;

		case 0x85:								// test rm32, r32
			x_modrm(&x);
			x_alu32(4, x_get32(&x), x_r32(x.reg));
			return;

		case 0x87:								// xchg rm32, r32
			x_modrm(&x);
			d = x_get32(&x);
			x_put32(&x, x_r32(x.reg));
			x_w32(x.reg, d);
			return;

		case 0x89:								// mov rm32, r32
			x_modrm(&x);
			I286_WORKCLOCK(2);
			x_put32(&x, x_r32(x.reg));
			return;

		case 0x8b:								// mov r32, rm32
			x_modrm(&x);
			x_w32(x.reg, x_get32(&x));
			return;

		case 0x8d:								// lea r32, m
			{
				UINT m;
				GET_PCBYTE(m);
				if (m >= 0xc0) {
					x_invalid();
					return;
				}
				I286_WORKCLOCK(3);
				x_w32((m >> 3) & 7, CALC_LEA(m));
			}
			return;

		case 0x8f:								// pop rm32
			d = x_pop32();
			x_modrm(&x);
			x_put32(&x, d);
			I286_WORKCLOCK(5);
			return;

		case 0x90:								// nop
			I286_WORKCLOCK(3);
			return;

		case 0x98:								// cwde
			I286_WORKCLOCK(3);
			x_w32(0, (UINT32)(SINT32)(SINT16)I286_AX);
			return;

		case 0x99:								// cdq
			I286_WORKCLOCK(2);
			x_w32(2, (x_r32(0) & 0x80000000) ? 0xffffffff : 0);
			return;

		case 0x9c:								// pushfd
			I286_WORKCLOCK(3);
			x_push32(REAL_FLAGREG);
			return;

		case 0x9d:								// popfd
			x_popfd();
			return;

		case 0xa1:								// mov eax, [moffs]
			{
				UINT o;
				GET_PCWORD(o);
				I286_WORKCLOCK(5);
				x_w32(0, x_rd32(DS_FIX + o));
			}
			return;

		case 0xa3:								// mov [moffs], eax
			{
				UINT o;
				GET_PCWORD(o);
				I286_WORKCLOCK(3);
				x_wr32(DS_FIX + o, x_r32(0));
			}
			return;

		case 0xa9:								// test eax, imm32
			I286_WORKCLOCK(3);
			x_alu32(4, x_r32(0), x_imm32());
			return;

		case 0xa5:
		case 0xa7:
		case 0xab:
		case 0xad:
		case 0xaf:
			x_string32(op, rep);
			return;

		case 0xc1:								// shift rm32, imm8
		case 0xd1:								// shift rm32, 1
		case 0xd3:								// shift rm32, cl
			{
				UINT cnt;
				x_modrm(&x);
				d = x_get32(&x);
				if (op == 0xc1) {
					GET_PCBYTE(cnt);
				}
				else {
					cnt = (op == 0xd1) ? 1 : I286_CL;
				}
				r = x_shift(x.reg, d, cnt, 32);
				x_put32(&x, r);
			}
			return;

		case 0xc7:								// mov rm32, imm32
			x_modrm(&x);
			I286_WORKCLOCK(2);
			x_put32(&x, x_imm32());
			return;

		case 0xf7:
			x_modrm(&x);
			d = x_get32(&x);
			switch (x.reg) {
				case 0:
				case 1:							// test rm32, imm32
					x_alu32(4, d, x_imm32());
					return;

				case 2:							// not
					x_put32(&x, ~d);
					return;

				case 3:							// neg
					r = 0 - d;
					I286_OV = (d == 0x80000000);
					I286_FLAGL = (UINT8)(((r ^ d) & A_FLAG) | SZP32(r) | (d != 0));
					x_put32(&x, r);
					return;

				case 4:							// mul
					{
						UINT64 p = (UINT64)x_r32(0) * d;
						x_w32(0, (UINT32)p);
						x_w32(2, (UINT32)(p >> 32));
						I286_OV = ((p >> 32) != 0);
						I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | (I286_OV ? C_FLAG : 0));
						I286_WORKCLOCK(38);
					}
					return;

				case 5:							// imul
					{
						SINT64 p = (SINT64)(SINT32)x_r32(0) * (SINT32)d;
						x_w32(0, (UINT32)p);
						x_w32(2, (UINT32)((UINT64)p >> 32));
						I286_OV = (p != (SINT32)p);
						I286_FLAGL = (UINT8)((I286_FLAGL & ~C_FLAG) | (I286_OV ? C_FLAG : 0));
						I286_WORKCLOCK(38);
					}
					return;

				case 6:							// div
					{
						UINT64 n = ((UINT64)x_r32(2) << 32) | x_r32(0);
						UINT64 q;
						if (d == 0) {
							x_divide_error();
							return;
						}
						q = n / d;
						if (q > 0xffffffff) {
							x_divide_error();
							return;
						}
						x_w32(0, (UINT32)q);
						x_w32(2, (UINT32)(n % d));
						I286_WORKCLOCK(38);
					}
					return;

				default:						// idiv
					{
						SINT64 n = (SINT64)(((UINT64)x_r32(2) << 32) | x_r32(0));
						SINT64 q;
						if (d == 0 || (n == (SINT64)((UINT64)1 << 63) && (SINT32)d == -1)) {
							x_divide_error();
							return;
						}
						q = n / (SINT32)d;
						if (q != (SINT32)q) {
							x_divide_error();
							return;
						}
						x_w32(0, (UINT32)q);
						x_w32(2, (UINT32)(n % (SINT32)d));
						I286_WORKCLOCK(43);
					}
					return;
			}
			return;

		case 0xff:
			x_modrm(&x);
			switch (x.reg) {
				case 0:							// inc rm32
				case 1:							// dec rm32
					{
						UINT32 c = I286_FLAGL & C_FLAG;
						d = x_get32(&x);
						if (x.reg == 0) {
							r = d + 1;
							I286_OV = (r == 0x80000000);
						}
						else {
							r = d - 1;
							I286_OV = (d == 0x80000000);
						}
						I286_FLAGL = (UINT8)(((r ^ d ^ 1) & A_FLAG) | SZP32(r) | c);
						x_put32(&x, r);
					}
					return;

				case 6:							// push rm32
					x_push32(x_get32(&x));
					return;
			}
			break;
	}
	x_invalid();
}

// ---- 67h: 32-bit addressing
// Only the address arithmetic changes: a ModRM operand can use any 32-bit
// register as a base, a scaled index and a 32-bit displacement. Rather than
// write every instruction a second time, the address is worked out here, ahead
// of the instruction, and handed to the core's own handler through the next
// CALC_EA / CALC_LEA / GET_EA it makes. Those skip the SIB and displacement
// bytes the 16-bit decoder does not know about.

int		i386x_eahook;			// the next address calculation takes the one below
static UINT32 x_ea_seg;
static UINT32 x_ea_off;
static UINT x_ea_len;			// bytes after the ModRM byte: SIB and displacement

UINT32 i386x_ea_take(void) {

	i386x_eahook = 0;
	I286_IP += x_ea_len;
	return x_ea_seg + x_ea_off;
}

UINT16 i386x_lea_take(void) {

	i386x_eahook = 0;
	I286_IP += x_ea_len;
	return (UINT16)x_ea_off;
}

UINT i386x_getea_take(UINT32 *seg) {

	i386x_eahook = 0;
	I286_IP += x_ea_len;
	*seg = x_ea_seg;
	return x_ea_off;
}

static UINT32 x_code32(UINT16 ip) {

	UINT32	v;
	UINT	k;

	v = 0;
	for (k = 0; k < 4; k++) {
		v |= (UINT32)i286_memoryread(CS_BASE + (UINT16)(ip + k)) << (k * 8);
	}
	return v;
}

// Decode the ModRM byte at IP + `at` (not consumed) with 32-bit addressing.
static void x_prepare_ea32(UINT at) {

	UINT16	p;
	UINT	m;
	UINT	mod;
	UINT	rm;
	UINT	n;
	UINT32	off;
	int		usess;

	p = (UINT16)(I286_IP + at);
	m = i286_memoryread(CS_BASE + p);
	mod = m >> 6;
	rm = m & 7;
	if (mod == 3) {
		return;
	}
	n = 1;
	off = 0;
	usess = 0;
	if (rm == 4) {
		UINT sib = i286_memoryread(CS_BASE + (UINT16)(p + 1));
		UINT base = sib & 7;
		UINT idx = (sib >> 3) & 7;
		n = 2;
		if ((base == 5) && (mod == 0)) {
			off = x_code32((UINT16)(p + n));
			n += 4;
		}
		else {
			off = x_r32(base);
			usess = (base == 4) || (base == 5);
		}
		if (idx != 4) {
			off += x_r32(idx) << (sib >> 6);
		}
	}
	else if ((rm == 5) && (mod == 0)) {
		off = x_code32((UINT16)(p + n));
		n += 4;
	}
	else {
		off = x_r32(rm);
		usess = (rm == 5);
	}
	if (mod == 1) {
		off += (UINT32)(SINT32)(SINT8)i286_memoryread(CS_BASE + (UINT16)(p + n));
		n += 1;
	}
	else if (mod == 2) {
		off += x_code32((UINT16)(p + n));
		n += 4;
	}
	x_ea_seg = usess ? SS_FIX : DS_FIX;
	x_ea_off = off;
	x_ea_len = n - 1;
	i386x_eahook = 1;
}

// Which one-byte opcodes carry a ModRM byte.
static int x_has_modrm(UINT op) {

	if (op < 0x40) {
		return (op & 7) < 4;
	}
	switch (op) {
		case 0x62: case 0x63: case 0x69: case 0x6b:
		case 0xc0: case 0xc1: case 0xc4: case 0xc5: case 0xc6: case 0xc7:
		case 0xd0: case 0xd1: case 0xd2: case 0xd3:
		case 0xf6: case 0xf7: case 0xfe: case 0xff:
			return 1;
	}
	return ((op >= 0x80) && (op < 0x90)) || ((op >= 0xd8) && (op < 0xe0));
}

// ...and which 0Fh ones.
static int x_has_modrm_0f(UINT op2) {

	if (op2 < 4) {
		return 1;
	}
	if ((op2 >= 0x90) && (op2 < 0xa0)) {
		return 1;
	}
	switch (op2) {
		case 0xa3: case 0xa4: case 0xa5: case 0xab: case 0xac: case 0xad:
		case 0xaf: case 0xb2: case 0xb3: case 0xb4: case 0xb5: case 0xb6:
		case 0xb7: case 0xba: case 0xbb: case 0xbc: case 0xbd: case 0xbe:
		case 0xbf:
			return 1;
	}
	return 0;
}

// MOV between AL/AX/EAX and a 32-bit absolute address.
static void x_moffs32(UINT op, int o32) {

	UINT32	a;

	a = DS_FIX + x_imm32();
	I286_WORKCLOCK(5);
	switch (op) {
		case 0xa0: I286_AL = i286_memoryread(a); break;
		case 0xa1: if (o32) x_w32(0, x_rd32(a)); else I286_AX = i286_memoryread_w(a); break;
		case 0xa2: i286_memorywrite(a, I286_AL); break;
		default:   if (o32) x_wr32(a, x_r32(0)); else i286_memorywrite_w(a, I286_AX); break;
	}
}

// ---- 66h / 67h and whatever prefixes come with them
// A segment override, REP, 66h and 67h can come in any order; the first of
// them to reach the core decides which entry point runs, and the rest are
// taken here.

static void x_prefixes(int o32, int a32, UINT rep) {

	UINT	op;
	UINT	n;
	int		seg;
	UINT16	cs;

	seg = 0;
	for (n = 0; n < MAX_PREFIX; n++) {
		GET_PCBYTE(op);
		switch (op) {
			case 0x26: DS_FIX = ES_BASE; SS_FIX = ES_BASE; seg = 1; continue;
			case 0x2e: DS_FIX = CS_BASE; SS_FIX = CS_BASE; seg = 1; continue;
			case 0x36: DS_FIX = SS_BASE; SS_FIX = SS_BASE; seg = 1; continue;
			case 0x3e: DS_FIX = DS_BASE; SS_FIX = DS_BASE; seg = 1; continue;
			case 0x64: DS_FIX = i386x_fs_base; SS_FIX = i386x_fs_base; seg = 1; continue;
			case 0x65: DS_FIX = i386x_gs_base; SS_FIX = i386x_gs_base; seg = 1; continue;
			case 0x66: o32 = 1; continue;
			case 0x67: a32 = 1; continue;
			case 0xf3: rep = 1; continue;
			case 0xf2: rep = 2; continue;
		}
		cs = I286_CS;
		if (a32 && (op >= 0xa0) && (op < 0xa4)) {
			x_moffs32(op, o32);
		}
		else {
			if (a32) {
				if (op == 0x0f) {
					if (x_has_modrm_0f(i286_memoryread(CS_BASE + I286_IP))) {
						x_prepare_ea32(1);
					}
				}
				else if (x_has_modrm(op)) {
					x_prepare_ea32(0);
				}
			}
			if (o32) {
				x_exec32(op, rep);
			}
			else if (rep == 1) {
				i286op_repe[op]();
			}
			else if (rep == 2) {
				i286op_repne[op]();
			}
			else {
				i286op[op]();
			}
			// A handler that never asked for its address (an FPU escape, say)
			// still has to step over the bytes it did not read. Not after an
			// interrupt, though: then IP is somewhere else entirely.
			if (i386x_eahook) {
				i386x_eahook = 0;
				if (I286_CS == cs) {
					I286_IP += x_ea_len;
				}
			}
		}
		if (seg) {
			REMOVE_PREFIX
		}
		return;
	}
	x_invalid();
}

void _opsize32(void) {					// 66: operand size

	if (!i386x_enabled) {
		INT_NUM(6, I286_IP - 1);
		return;
	}
	i386x_ip0 = I286_IP - 1;
	x_prefixes(1, 0, 0);
}

void _repe_opsize32(void) {				// F3 66

	if (!i386x_enabled) {
		INT_NUM(6, I286_IP - 1);
		return;
	}
	i386x_ip0 = I286_IP - 2;
	x_prefixes(1, 0, 1);
}

void _repne_opsize32(void) {				// F2 66

	if (!i386x_enabled) {
		INT_NUM(6, I286_IP - 1);
		return;
	}
	i386x_ip0 = I286_IP - 2;
	x_prefixes(1, 0, 2);
}

void _addrsize32(void) {					// 67: address size

	if (!i386x_enabled) {
		INT_NUM(6, I286_IP - 1);
		return;
	}
	i386x_ip0 = I286_IP - 1;
	x_prefixes(0, 1, 0);
}

void _repe_addrsize32(void) {				// F3 67

	if (!i386x_enabled) {
		INT_NUM(6, I286_IP - 1);
		return;
	}
	i386x_ip0 = I286_IP - 2;
	x_prefixes(0, 1, 1);
}

void _repne_addrsize32(void) {			// F2 67

	if (!i386x_enabled) {
		INT_NUM(6, I286_IP - 1);
		return;
	}
	i386x_ip0 = I286_IP - 2;
	x_prefixes(0, 1, 2);
}

// ---- 64h / 65h: FS / GS segment override

static void x_segprefix(UINT32 base, const I286OP *table) {

	if (!i386x_enabled) {
		INT_NUM(6, I286_IP - 1);
		return;
	}

	SS_FIX = base;
	DS_FIX = base;
	I286_PREFIX++;
	if (I286_PREFIX < MAX_PREFIX) {
		UINT op;
		GET_PCBYTE(op);
		table[op]();
		REMOVE_PREFIX
		I286_PREFIX = 0;
	}
	else {
		INT_NUM(6, I286_IP);
	}
}

void _segprefix_fs(void) { x_segprefix(i386x_fs_base, i286op); }
void _segprefix_gs(void) { x_segprefix(i386x_gs_base, i286op); }
void _repe_segprefix_fs(void) { x_segprefix(i386x_fs_base, i286op_repe); }
void _repe_segprefix_gs(void) { x_segprefix(i386x_gs_base, i286op_repe); }
void _repne_segprefix_fs(void) { x_segprefix(i386x_fs_base, i286op_repne); }
void _repne_segprefix_gs(void) { x_segprefix(i386x_gs_base, i286op_repne); }
