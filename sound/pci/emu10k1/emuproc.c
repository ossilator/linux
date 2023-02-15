// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *                   Lee Revell <rlrevell@joe-job.com>
 *                   James Courtier-Dutton <James@superbug.co.uk>
 *                   Oswald Buddenhagen <oswald.buddenhagen@gmx.de>
 *                   Creative Labs, Inc.
 *
 *  Routines for control of EMU10K1 chips / proc interface routines
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <sound/core.h>
#include <sound/emu10k1.h>
#include "p16v.h"

static void snd_emu10k1_proc_spdif_status(struct snd_emu10k1 * emu,
					  struct snd_info_buffer *buffer,
					  char *title,
					  int status_reg,
					  int rate_reg)
{
	static const char * const clkaccy[4] = { "1000ppm", "50ppm", "variable", "unknown" };
	static const int samplerate[16] = { 44100, 1, 48000, 32000, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
	static const char * const channel[16] = { "unspec", "left", "right", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15" };
	static const char * const emphasis[8] = { "none", "50/15 usec 2 channel", "2", "3", "4", "5", "6", "7" };
	unsigned int status, rate = 0;
	
	status = snd_emu10k1_ptr_read(emu, status_reg, 0);

	snd_iprintf(buffer, "\n%s\n", title);

	if (status != 0xffffffff) {
		snd_iprintf(buffer, "Professional Mode     : %s\n", (status & SPCS_PROFESSIONAL) ? "yes" : "no");
		snd_iprintf(buffer, "Not Audio Data        : %s\n", (status & SPCS_NOTAUDIODATA) ? "yes" : "no");
		snd_iprintf(buffer, "Copyright             : %s\n", (status & SPCS_COPYRIGHT) ? "yes" : "no");
		snd_iprintf(buffer, "Emphasis              : %s\n", emphasis[(status & SPCS_EMPHASISMASK) >> 3]);
		snd_iprintf(buffer, "Mode                  : %i\n", (status & SPCS_MODEMASK) >> 6);
		snd_iprintf(buffer, "Category Code         : 0x%x\n", (status & SPCS_CATEGORYCODEMASK) >> 8);
		snd_iprintf(buffer, "Generation Status     : %s\n", status & SPCS_GENERATIONSTATUS ? "original" : "copy");
		snd_iprintf(buffer, "Source Mask           : %i\n", (status & SPCS_SOURCENUMMASK) >> 16);
		snd_iprintf(buffer, "Channel Number        : %s\n", channel[(status & SPCS_CHANNELNUMMASK) >> 20]);
		snd_iprintf(buffer, "Sample Rate           : %iHz\n", samplerate[(status & SPCS_SAMPLERATEMASK) >> 24]);
		snd_iprintf(buffer, "Clock Accuracy        : %s\n", clkaccy[(status & SPCS_CLKACCYMASK) >> 28]);

		if (rate_reg > 0) {
			rate = snd_emu10k1_ptr_read(emu, rate_reg, 0);
			snd_iprintf(buffer, "S/PDIF Valid          : %s\n", rate & SRCS_SPDIFVALID ? "on" : "off");
			snd_iprintf(buffer, "S/PDIF Locked         : %s\n", rate & SRCS_SPDIFLOCKED ? "on" : "off");
			snd_iprintf(buffer, "Rate Locked           : %s\n", rate & SRCS_RATELOCKED ? "on" : "off");
			/* From ((Rate * 48000 ) / 262144); */
			snd_iprintf(buffer, "Estimated Sample Rate : %d\n", ((rate & 0xFFFFF ) * 375) >> 11); 
		}
	} else {
		snd_iprintf(buffer, "No signal detected.\n");
	}

}

static void snd_emu10k1_proc_read(struct snd_info_entry *entry, 
				  struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;
	const char * const *inputs = emu->audigy ?
		snd_emu10k1_audigy_ins : snd_emu10k1_sblive_ins;
	const char * const *outputs = emu->audigy ?
		snd_emu10k1_audigy_outs : snd_emu10k1_sblive_outs;
	unsigned short extin_mask = emu->audigy ? ~0 : emu->fx8010.extin_mask;
	unsigned short extout_mask = emu->audigy ? ~0 : emu->fx8010.extout_mask;
	unsigned int val, val1, ptrx, psst, dsl, snda;
	int nefx = emu->audigy ? 32 : 16;
	int idx;
	
	snd_iprintf(buffer, "EMU10K1\n\n");
	snd_iprintf(buffer, "Card                  : %s\n",
		    emu->card_capabilities->emu_model ? "E-MU D.A.S." :
		    emu->card_capabilities->ecard ? "E-MU A.P.S." :
		    emu->audigy ? "SB Audigy" : "SB Live!");
	snd_iprintf(buffer, "Internal TRAM (words) : 0x%x\n", emu->fx8010.itram_size);
	snd_iprintf(buffer, "External TRAM (words) : 0x%x\n", (int)emu->fx8010.etram_pages.bytes / 2);

	// The following are internal details in D.A.S. mode,
	// so there is no use in displaying them to the user.
//	if (emu->das_mode)
//		return;

	snd_iprintf(buffer, "\nEffect Send Routing & Amounts:\n");
	for (idx = 0; idx < NUM_G; idx++) {
		ptrx = snd_emu10k1_ptr_read(emu, PTRX, idx);
		psst = snd_emu10k1_ptr_read(emu, PSST, idx);
		dsl = snd_emu10k1_ptr_read(emu, DSL, idx);
		if (emu->audigy) {
			val = snd_emu10k1_ptr_read(emu, A_FXRT1, idx);
			val1 = snd_emu10k1_ptr_read(emu, A_FXRT2, idx);
			snda = snd_emu10k1_ptr_read(emu, A_SENDAMOUNTS, idx);
			snd_iprintf(buffer, "Ch%-2i: A=%2i:%02x, B=%2i:%02x, C=%2i:%02x, D=%2i:%02x, ",
				idx,
				val & 0x3f, REG_VAL_GET(PTRX_FXSENDAMOUNT_A, ptrx),
				(val >> 8) & 0x3f, REG_VAL_GET(PTRX_FXSENDAMOUNT_B, ptrx),
				(val >> 16) & 0x3f, REG_VAL_GET(PSST_FXSENDAMOUNT_C, psst),
				(val >> 24) & 0x3f, REG_VAL_GET(DSL_FXSENDAMOUNT_D, dsl));
			snd_iprintf(buffer, "E=%2i:%02x, F=%2i:%02x, G=%2i:%02x, H=%2i:%02x\n",
				val1 & 0x3f, (snda >> 24) & 0xff,
				(val1 >> 8) & 0x3f, (snda >> 16) & 0xff,
				(val1 >> 16) & 0x3f, (snda >> 8) & 0xff,
				(val1 >> 24) & 0x3f, snda & 0xff);
		} else {
			val = snd_emu10k1_ptr_read(emu, FXRT, idx);
			snd_iprintf(buffer, "Ch%-2i: A=%2i:%02x, B=%2i:%02x, C=%2i:%02x, D=%2i:%02x\n",
				idx,
				(val >> 16) & 0x0f, REG_VAL_GET(PTRX_FXSENDAMOUNT_A, ptrx),
				(val >> 20) & 0x0f, REG_VAL_GET(PTRX_FXSENDAMOUNT_B, ptrx),
				(val >> 24) & 0x0f, REG_VAL_GET(PSST_FXSENDAMOUNT_C, psst),
				(val >> 28) & 0x0f, REG_VAL_GET(DSL_FXSENDAMOUNT_D, dsl));
		}
	}
	snd_iprintf(buffer, "\nEffect Send Targets:\n");
	// Audigy actually has 64, but we don't use them all.
	for (idx = 0; idx < 32; idx++) {
		const char *c = snd_emu10k1_fxbus[idx];
		if (c)
			snd_iprintf(buffer, "  Channel %02i [%s]\n", idx, c);
	}
	if (!emu->card_capabilities->emu_model) {
		snd_iprintf(buffer, "\nOutput Channels:\n");
		for (idx = 0; idx < 32; idx++)
			if (outputs[idx] && (extout_mask & (1 << idx)))
				snd_iprintf(buffer, "  Channel %02i [%s]\n", idx, outputs[idx]);
		snd_iprintf(buffer, "\nInput Channels:\n");
		for (idx = 0; idx < 16; idx++)
			if (inputs[idx] && (extin_mask & (1 << idx)))
				snd_iprintf(buffer, "  Channel %02i [%s]\n", idx, inputs[idx]);
		snd_iprintf(buffer, "\nMultichannel Capture Sources:\n");
		for (idx = 0; idx < nefx; idx++)
			if (emu->efx_voices_mask[0] & (1 << idx))
				snd_iprintf(buffer, "  Channel %02i [Output: %s]\n",
					    idx, outputs[idx] ? outputs[idx] : "???");
		if (emu->audigy) {
			for (idx = 0; idx < 32; idx++)
				if (emu->efx_voices_mask[1] & (1 << idx))
					snd_iprintf(buffer, "  Channel %02i [Input: %s]\n",
						    idx + 32, inputs[idx] ? inputs[idx] : "???");
		} else {
			for (idx = 0; idx < 16; idx++) {
				if (emu->efx_voices_mask[0] & ((1 << 16) << idx)) {
					if (emu->card_capabilities->sblive51) {
						s8 c = snd_emu10k1_sblive51_fxbus2_map[idx];
						if (c == -1)
							snd_iprintf(buffer, "  Channel %02i [Output: %s]\n",
								    idx + 16, outputs[idx + 16]);
						else
							snd_iprintf(buffer, "  Channel %02i [Input: %s]\n",
								    idx + 16, inputs[c]);
					} else {
						snd_iprintf(buffer, "  Channel %02i [Input: %s]\n",
							    idx + 16, inputs[idx] ? inputs[idx] : "???");
					}
				}
			}
		}
	}
}

static void snd_emu10k1_proc_spdif_read(struct snd_info_entry *entry, 
				  struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;
	u32 value;
	u32 value2;

	if (emu->card_capabilities->emu_model) {
		snd_emu1010_fpga_lock(emu);

		// This represents the S/PDIF lock status on 0404b, which is
		// kinda weird and unhelpful, because monitoring it via IRQ is
		// impractical (one gets an IRQ flood as long as it is desynced).
		snd_emu1010_fpga_read(emu, EMU_HANA_IRQ_STATUS, &value);
		snd_iprintf(buffer, "Lock status 1: %#x\n", value & 0x10);

		// Bit 0x1 in LO being 0 is supposedly for ADAT lock.
		// The registers are always all zero on 0404b.
		snd_emu1010_fpga_read(emu, EMU_HANA_LOCK_STS_LO, &value);
		snd_emu1010_fpga_read(emu, EMU_HANA_LOCK_STS_HI, &value2);
		snd_iprintf(buffer, "Lock status 2: %#x %#x\n", value, value2);

		snd_iprintf(buffer, "S/PDIF rate: %dHz\n",
			    snd_emu1010_get_raw_rate(emu, EMU_HANA_WCLOCK_HANA_SPDIF_IN));
		if (emu->card_capabilities->emu_model != EMU_MODEL_EMU0404) {
			snd_iprintf(buffer, "ADAT rate: %dHz\n",
				    snd_emu1010_get_raw_rate(emu, EMU_HANA_WCLOCK_HANA_ADAT_IN));
			snd_iprintf(buffer, "Dock rate: %dHz\n",
				    snd_emu1010_get_raw_rate(emu, EMU_HANA_WCLOCK_2ND_HANA));
		}
		if (emu->card_capabilities->emu_model == EMU_MODEL_EMU0404 ||
		    emu->card_capabilities->emu_model == EMU_MODEL_EMU1010)
			snd_iprintf(buffer, "BNC rate: %dHz\n",
				    snd_emu1010_get_raw_rate(emu, EMU_HANA_WCLOCK_SYNC_BNC));

		// FIXME: we could control these two settings for TX.
		// "IEC958 Playback Default", for iecset
		snd_emu1010_fpga_read(emu, EMU_HANA_SPDIF_MODE, &value);
		if (value & EMU_HANA_SPDIF_MODE_RX_INVALID)
			snd_iprintf(buffer, "\nS/PDIF input invalid\n");
		else
			snd_iprintf(buffer, "\nS/PDIF mode: %s%s\n",
				    value & EMU_HANA_SPDIF_MODE_RX_PRO ? "professional" : "consumer",
				    value & EMU_HANA_SPDIF_MODE_RX_NOCOPY ? ", no copy" : "");

		snd_emu1010_fpga_unlock(emu);
	} else {
		snd_emu10k1_proc_spdif_status(emu, buffer, "CD-ROM S/PDIF In", CDCS, CDSRCS);
		snd_emu10k1_proc_spdif_status(emu, buffer, "Optical or Coax S/PDIF In", GPSCS, GPSRCS);
	}
#if 0
	val = snd_emu10k1_ptr_read(emu, ZVSRCS, 0);
	snd_iprintf(buffer, "\nZoomed Video\n");
	snd_iprintf(buffer, "Rate Locked           : %s\n", val & SRCS_RATELOCKED ? "on" : "off");
	snd_iprintf(buffer, "Estimated Sample Rate : 0x%x\n", val & SRCS_ESTSAMPLERATE);
#endif
}

static void snd_emu10k1_proc_rates_read(struct snd_info_entry *entry, 
				  struct snd_info_buffer *buffer)
{
	static const int samplerate[8] = { 44100, 48000, 96000, 192000, 4, 5, 6, 7 };
	struct snd_emu10k1 *emu = entry->private_data;
	unsigned int val, tmp, n;
	val = snd_emu10k1_ptr20_read(emu, CAPTURE_RATE_STATUS, 0);
	for (n = 0; n < 4; n++) {
		tmp = val >> (16 + (n*4));
		if (tmp & 0x8) snd_iprintf(buffer, "Channel %d: Rate=%d\n", n, samplerate[tmp & 0x7]);
		else snd_iprintf(buffer, "Channel %d: No input\n", n);
	}
}

struct emu10k1_reg_entry {
	unsigned short base, size;
	const char *name;
};

static const struct emu10k1_reg_entry sblive_reg_entries[] = {
	{    0, 0x10, "FXBUS" },
	{ 0x10, 0x10, "EXTIN" },
	{ 0x20, 0x10, "EXTOUT" },
	{ 0x30, 0x10, "FXBUS2" },
	{ 0x40, 0x20, NULL },  // Constants
	{ 0x100, 0x100, "GPR" },
	{ 0x200, 0x80, "ITRAM_DATA" },
	{ 0x280, 0x20, "ETRAM_DATA" },
	{ 0x300, 0x80, "ITRAM_ADDR" },
	{ 0x380, 0x20, "ETRAM_ADDR" },
	{ 0x400, 0, NULL }
};

static const struct emu10k1_reg_entry audigy_reg_entries[] = {
	{    0, 0x40, "FXBUS" },
	{ 0x40, 0x10, "EXTIN" },
	{ 0x50, 0x10, "P16VIN" },
	{ 0x60, 0x20, "EXTOUT" },
	{ 0x80, 0x20, "FXBUS2" },
	{ 0xa0, 0x10, "EMU32OUTH" },
	{ 0xb0, 0x10, "EMU32OUTL" },
	{ 0xc0, 0x20, NULL },  // Constants
	// This can't be quite right - overlap.
	//{ 0x100, 0xc0, "ITRAM_CTL" },
	//{ 0x1c0, 0x40, "ETRAM_CTL" },
	{ 0x160, 0x20, "A3_EMU32IN" },
	{ 0x1e0, 0x20, "A3_EMU32OUT" },
	{ 0x200, 0xc0, "ITRAM_DATA" },
	{ 0x2c0, 0x40, "ETRAM_DATA" },
	{ 0x300, 0xc0, "ITRAM_ADDR" },
	{ 0x3c0, 0x40, "ETRAM_ADDR" },
	{ 0x400, 0x200, "GPR" },
	{ 0x600, 0, NULL }
};

static const char * const emu10k1_const_entries[] = {
	"C_00000000",
	"C_00000001",
	"C_00000002",
	"C_00000003",
	"C_00000004",
	"C_00000008",
	"C_00000010",
	"C_00000020",
	"C_00000100",
	"C_00010000",
	"C_00000800",
	"C_10000000",
	"C_20000000",
	"C_40000000",
	"C_80000000",
	"C_7fffffff",
	"C_ffffffff",
	"C_fffffffe",
	"C_c0000000",
	"C_4f1bbcdc",
	"C_5a7ef9db",
	"C_00100000",
	"GPR_ACCU",
	"GPR_COND",
	"GPR_NOISE0",
	"GPR_NOISE1",
	"GPR_IRQ",
	"GPR_DBAC",
	"GPR_DBACE",
	"???",
};

static int disasm_emu10k1_reg(char *buffer,
			      const struct emu10k1_reg_entry *entries,
			      unsigned reg, const char *pfx)
{
	for (int i = 0; ; i++) {
		unsigned base = entries[i].base;
		unsigned size = entries[i].size;
		if (!size)
			return sprintf(buffer, "%s0x%03x", pfx, reg);
		if (reg >= base && reg < base + size) {
			const char *name = entries[i].name;
			reg -= base;
			if (name)
				return sprintf(buffer, "%s%s(%u)", pfx, name, reg);
			return sprintf(buffer, "%s%s", pfx, emu10k1_const_entries[reg]);
		}
	}
}

static int disasm_sblive_reg(char *buffer, unsigned reg, const char *pfx)
{
	return disasm_emu10k1_reg(buffer, sblive_reg_entries, reg, pfx);
}

static int disasm_audigy_reg(char *buffer, unsigned reg, const char *pfx)
{
	return disasm_emu10k1_reg(buffer, audigy_reg_entries, reg, pfx);
}

static void snd_emu10k1_proc_acode_read(struct snd_info_entry *entry,
				        struct snd_info_buffer *buffer)
{
	u32 pc;
	struct snd_emu10k1 *emu = entry->private_data;
	static const char * const insns[16] = {
		"MAC0", "MAC1", "MAC2", "MAC3", "MACINT0", "MACINT1", "ACC3", "MACMV",
		"ANDXOR", "TSTNEG", "LIMITGE", "LIMITLT", "LOG", "EXP", "INTERP", "SKIP",
	};
	static const char spaces[] = "                              ";
	const int nspaces = sizeof(spaces) - 1;

	snd_iprintf(buffer, "FX8010 Instruction List '%s'\n", emu->fx8010.name);
	snd_iprintf(buffer, "  Code dump      :\n");
	for (pc = 0; pc < (emu->audigy ? 1024 : 512); pc++) {
		u32 low, high;
		int len;
		char buf[100];
		char *bufp = buf;
			
		low = snd_emu10k1_efx_read(emu, pc * 2);
		high = snd_emu10k1_efx_read(emu, pc * 2 + 1);
		if (emu->audigy) {
			bufp += sprintf(bufp, "    %-7s  ", insns[(high >> 24) & 0x0f]);
			bufp += disasm_audigy_reg(bufp, (high >> 12) & 0x7ff, "");
			bufp += disasm_audigy_reg(bufp, (high >> 0) & 0x7ff, ", ");
			bufp += disasm_audigy_reg(bufp, (low >> 12) & 0x7ff, ", ");
			bufp += disasm_audigy_reg(bufp, (low >> 0) & 0x7ff, ", ");
		} else {
			bufp += sprintf(bufp, "    %-7s  ", insns[(high >> 20) & 0x0f]);
			bufp += disasm_sblive_reg(bufp, (high >> 10) & 0x3ff, "");
			bufp += disasm_sblive_reg(bufp, (high >> 0) & 0x3ff, ", ");
			bufp += disasm_sblive_reg(bufp, (low >> 10) & 0x3ff, ", ");
			bufp += disasm_sblive_reg(bufp, (low >> 0) & 0x3ff, ", ");
		}
		len = (int)(ptrdiff_t)(bufp - buf);
		snd_iprintf(buffer, "%s %s /* 0x%04x: 0x%08x%08x */\n",
			    buf, &spaces[nspaces - clamp(65 - len, 0, nspaces)],
			    pc, high, low);
	}
}

#define TOTAL_SIZE_GPR		(0x100*4)
#define A_TOTAL_SIZE_GPR	(0x200*4)
#define TOTAL_SIZE_TANKMEM_DATA	(0xa0*4)
#define TOTAL_SIZE_TANKMEM_ADDR (0xa0*4)
#define A_TOTAL_SIZE_TANKMEM_DATA (0x100*4)
#define A_TOTAL_SIZE_TANKMEM_ADDR (0x100*4)
#define TOTAL_SIZE_CODE		(0x200*8)
#define A_TOTAL_SIZE_CODE	(0x400*8)

static ssize_t snd_emu10k1_fx8010_read(struct snd_info_entry *entry,
				       void *file_private_data,
				       struct file *file, char __user *buf,
				       size_t count, loff_t pos)
{
	struct snd_emu10k1 *emu = entry->private_data;
	unsigned int offset;
	int tram_addr = 0;
	unsigned int *tmp;
	long res;
	unsigned int idx;
	
	if (!strcmp(entry->name, "fx8010_tram_addr")) {
		offset = TANKMEMADDRREGBASE;
		tram_addr = 1;
	} else if (!strcmp(entry->name, "fx8010_tram_data")) {
		offset = TANKMEMDATAREGBASE;
	} else if (!strcmp(entry->name, "fx8010_code")) {
		offset = emu->audigy ? A_MICROCODEBASE : MICROCODEBASE;
	} else {
		offset = emu->audigy ? A_FXGPREGBASE : FXGPREGBASE;
	}

	tmp = kmalloc(count + 8, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;
	for (idx = 0; idx < ((pos & 3) + count + 3) >> 2; idx++) {
		unsigned int val;
		val = snd_emu10k1_ptr_read(emu, offset + idx + (pos >> 2), 0);
		if (tram_addr && emu->audigy) {
			val >>= 11;
			val |= snd_emu10k1_ptr_read(emu, 0x100 + idx + (pos >> 2), 0) << 20;
		}
		tmp[idx] = val;
	}
	if (copy_to_user(buf, ((char *)tmp) + (pos & 3), count))
		res = -EFAULT;
	else
		res = count;
	kfree(tmp);
	return res;
}

static void snd_emu10k1_proc_voices_read(struct snd_info_entry *entry, 
				  struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;
	struct snd_emu10k1_voice *voice;
	int idx;
	static const char * const types[] = {
		"Unused", "EFX", "EFX IRQ", "PCM", "PCM IRQ", "Synth"
	};
	static_assert(ARRAY_SIZE(types) == EMU10K1_NUM_TYPES);

	snd_iprintf(buffer, "ch\tdirty\tlast\tuse\n");
	for (idx = 0; idx < NUM_G; idx++) {
		voice = &emu->voices[idx];
		snd_iprintf(buffer, "%i\t%u\t%u\t%s\n",
			idx,
			voice->dirty,
			voice->last,
			types[voice->use]);
	}
}

#ifdef CONFIG_SND_DEBUG

static void snd_emu_proc_emu1010_link_read(struct snd_emu10k1 *emu,
					   struct snd_info_buffer *buffer,
					   u32 dst)
{
	u32 src = snd_emu1010_fpga_link_dst_src_read(emu, dst);
	snd_iprintf(buffer, "%04x: %04x\n", dst, src);
}

static void snd_emu_proc_emu1010_reg_read(struct snd_info_entry *entry,
				     struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;
	u32 value;
	int i;

	snd_emu1010_fpga_lock(emu);

	snd_iprintf(buffer, "EMU1010 Registers:\n\n");

	for(i = 0; i < 0x40; i+=1) {
		snd_emu1010_fpga_read(emu, i, &value);
		snd_iprintf(buffer, "%02x: %02x\n", i, value);
	}

	snd_iprintf(buffer, "\nEMU1010 Routes:\n\n");

	for (i = 0; i < 16; i++)  // To Alice2/Tina[2] via EMU32
		snd_emu_proc_emu1010_link_read(emu, buffer, i);
	if (emu->card_capabilities->emu_model != EMU_MODEL_EMU0404)
		for (i = 0; i < 32; i++)  // To Dock via EDI
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x100 + i);
	if (emu->card_capabilities->emu_model != EMU_MODEL_EMU1616)
		for (i = 0; i < 8; i++)  // To Hamoa/local
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x200 + i);
	for (i = 0; i < 8; i++)  // To Hamoa/Mana/local
		snd_emu_proc_emu1010_link_read(emu, buffer, 0x300 + i);
	if (emu->card_capabilities->emu_model == EMU_MODEL_EMU1616) {
		for (i = 0; i < 16; i++)  // To Tina2 via EMU32
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x400 + i);
	} else if (emu->card_capabilities->emu_model != EMU_MODEL_EMU0404) {
		for (i = 0; i < 8; i++)  // To Hana ADAT
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x400 + i);
		if (emu->card_capabilities->emu_model == EMU_MODEL_EMU1010B) {
			for (i = 0; i < 16; i++)  // To Tina via EMU32
				snd_emu_proc_emu1010_link_read(emu, buffer, 0x500 + i);
		} else {
			// To Alice2 via I2S
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x500);
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x501);
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x600);
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x601);
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x700);
			snd_emu_proc_emu1010_link_read(emu, buffer, 0x701);
		}
	}

	snd_emu1010_fpga_unlock(emu);
}

#if SNAP_INPUT
static void snd_emu_proc_fmt_input(struct snd_emu10k1 *emu, struct snd_info_buffer *buffer, int offset, int nbytes, int s)
{
	nbytes += offset;
	for (int n = offset; n < nbytes;) {
		snd_iprintf(buffer, "%08x", s);
		if (emu->snap_stereo) {
			if (emu->snap_width == 16) {
				for (int i = 0; i < 4; i++, n += 4, s += 1)
					snd_iprintf(buffer, "  %04x:%04x",
						    emu->snap_buffer[n] | (emu->snap_buffer[n + 1] << 8),
						    emu->snap_buffer[n + 2] | (emu->snap_buffer[n + 3] << 8));
			} else {
				for (int i = 0; i < 4; i++, n += 4, s += 2)
					snd_iprintf(buffer, "  %02x:%02x %02x:%02x",
						    emu->snap_buffer[n], emu->snap_buffer[n + 1],
						    emu->snap_buffer[n + 2], emu->snap_buffer[n + 3]);
			}
		} else {
			if (emu->snap_width == 32) {
				for (int i = 0; i < 4; i++, n += 4, s += 1)
					snd_iprintf(buffer, "  %08x",
						    emu->snap_buffer[n] | (emu->snap_buffer[n + 1] << 8) |
						    (emu->snap_buffer[n + 2] << 16) | (emu->snap_buffer[n + 3] << 24));
			} else if (emu->snap_width == 16) {
				for (int i = 0; i < 4; i++, n += 4, s += 2)
					snd_iprintf(buffer, "  %04x %04x",
						    emu->snap_buffer[n] | (emu->snap_buffer[n + 1] << 8),
						    emu->snap_buffer[n + 2] | (emu->snap_buffer[n + 3] << 8));
			} else {
				for (int i = 0; i < 4; i++, n += 4, s += 4)
					snd_iprintf(buffer, "  %02x %02x %02x %02x",
						    emu->snap_buffer[n], emu->snap_buffer[n + 1],
						    emu->snap_buffer[n + 2], emu->snap_buffer[n + 3]);
			}
		}
		snd_iprintf(buffer, "\n");
	}
}
#endif

static void snd_emu_proc_snapshot_read(struct snd_info_entry *entry,
				     struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;

	if (atomic_xchg(&emu->snapshot_busy, 1)) {
		snd_iprintf(buffer, "snapshot is locked\n");
		return;
	}

#if SNAP_SAMPLES_XADDR || SNAP_SAMPLES_XCACHE || SNAP_SAMPLES_XMISC
	if (emu->das_mode) {
		snd_iprintf(buffer, "cannot snapshot extra voice in DAS mode\n");
		return;
	}
#endif

#if SNAP_SETUP
	snd_iprintf(buffer, "param:  chans=%d  subchans=%d  width=%d\n\n",
		    emu->snap_raw_chans, emu->snap_sub_chans, emu->snap_width);

	snd_iprintf(buffer, "snapshot'd voices:");
	for (int c = 0; c < emu->snap_total; c++)
		snd_iprintf(buffer, " %d", emu->snap_voices[c]);
	snd_iprintf(buffer, "\n\n");
#endif

#if SNAP_INIT
#if SNAP_INIT_VOL
	snd_iprintf(buffer, "A     B     C     D     E     F     G     H     CVCF\n");
	for (int c = 0; c < emu->snap_total; c++)
		snd_iprintf(buffer, "%04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %08x\n",
			    emu->snap_chan[c].csba & 0xffff, emu->snap_chan[c].csba >> 16,
			    emu->snap_chan[c].cshg & 0xffff, emu->snap_chan[c].cshg >> 16,
			    emu->snap_chan[c].csfe & 0xffff, emu->snap_chan[c].csfe >> 16,
			    emu->snap_chan[c].csdc & 0xffff, emu->snap_chan[c].csdc >> 16,
			    emu->snap_chan[c].cvcf);
	snd_iprintf(buffer, "\n");
#endif
	snd_iprintf(buffer, "CPF       PTRX      PSST      DSL       CCCA      CCR\n");
	for (int c = 0; c < emu->snap_total; c++) {
		snd_iprintf(buffer, "%08x  %08x  %08x  %08x  %08x  %08x\n",
			    emu->snap_chan[c].cpf,
			    emu->snap_chan[c].ptrx,
			    emu->snap_chan[c].psst,
			    emu->snap_chan[c].dsl,
			    emu->snap_chan[c].ccca,
			    emu->snap_chan[c].ccr);
	}
	snd_iprintf(buffer, "\n");
#endif

#if SNAP_OTHER
	snd_iprintf(buffer,
#if SNAP_OTHER_1
		"28 29"
#endif
		"\n");
	for (unsigned i = 0; i < ARRAY_SIZE(emu->snap_other); i++) {
		struct emu_other_snapshot *ss = &emu->snap_other[i];
#if SNAP_OTHER_1
		snd_iprintf(buffer, "%02x %02x", ss->fpga_reg_28, ss->fpga_reg_29);
#endif
		snd_iprintf(buffer, "\n");
	}
	snd_iprintf(buffer, "\n");
#endif

#define RELATIVE_WC 0
#define RELATIVE_WC_ERR 1

#if SNAP_IRQ
	snd_iprintf(buffer,
#if RELATIVE_WC
#if RELATIVE_WC_ERR
		"edWC  "
#endif
		"dWC    "
#else
		"WC     "
#endif
		"CC  IPR       "
#if SNAP_IRQ_DICE
		"DICE      "
#endif
#if SNAP_IRQ_CLIP
		"CLIP               "
#endif
#if SNAP_IRQ_CLIE
		"CLIE               "
#endif
#if SNAP_IRQ_HLIP
		"HLIP               "
#endif
	);
	for (int c = 0; c < emu->snap_total; c++)
		snd_iprintf(buffer, "| RDA "
#if SNAP_IRQ_CACHE
				    "CIS pos "
#endif
				    " ");
	snd_iprintf(buffer, "\n");
#if RELATIVE_WC && RELATIVE_WC_ERR
	int edwc = 0;
#endif
	for (int c = 0; c < emu->num_irq_snaps; c++) {
		struct emu_irq_snapshot *is = &emu->snap_irq[c];
		int wc = REG_VAL_GET(WC_SAMPLECOUNTER, is->wc);
#if RELATIVE_WC
		int dwc = c ? wc - REG_VAL_GET(WC_SAMPLECOUNTER, is[-1].wc) : 0;
		if (dwc < 0)
			dwc += 1 << REG_SIZE(WC_SAMPLECOUNTER);
#if RELATIVE_WC_ERR
		if (c)
			edwc += dwc - emu->snap_period;
		snd_iprintf(buffer, "%4d  %05x  ", edwc, dwc);
#else
		snd_iprintf(buffer, "%05x  ", dwc);
#endif
#else
		snd_iprintf(buffer, "%05x  ", wc);
#endif
		snd_iprintf(buffer, "%02x  %08x  ",
			    REG_VAL_GET(WC_CURRENTCHANNEL, is->wc),
			    is->ipr);
#if SNAP_IRQ_DICE
		snd_iprintf(buffer, "%08x  ", is->dice);
#endif
#if SNAP_IRQ_CLIP
		snd_iprintf(buffer, "%08x:%08x  ", is->cliph, is->clipl);
#endif
#if SNAP_IRQ_CLIE
		snd_iprintf(buffer, "%08x:%08x  ", is->clieh, is->cliel);
#endif
#if SNAP_IRQ_HLIP
		snd_iprintf(buffer, "%08x:%08x  ", is->hliph, is->hlipl);
#endif
		for (int c = 0; c < emu->snap_total; c++)
			snd_iprintf(buffer, "| %04x ",
				    REG_VAL_GET(CCCA_CURRADDR, is->ccca[c]));
#if SNAP_IRQ_CACHE
			snd_iprintf(buffer, "%02x %04x ",
				    REG_VAL_GET(CCR_CACHEINVALIDSIZE, is->ccr[c]),
				    REG_VAL_GET(CCCA_CURRADDR, is->ccca[c]) -
					REG_VAL_GET(CCR_CACHEINVALIDSIZE, is->ccr[c]));
#endif
		snd_iprintf(buffer, "\n");
	}
	snd_iprintf(buffer, "\n");
#endif

#if SNAP_INTERPOLATOR
	snd_iprintf(buffer, "R  S  WC     CC  Z1 << 5    Z1 << 5    sample\n");
	for (int c = 0; c < emu->num_inter_snaps; c++) {
		int z1 = emu->snap_inter[c].z1 << 5;
		int z2 = emu->snap_inter[c].z2 << 5;
		int fx = emu->snap_inter[c].fx;
		char sz1 = ' ', sz2 = ' ', sfx = ' ';
		if (z1 < 0)
			z1 = -z1, sz1 = '-';
		if (z2 < 0)
			z2 = -z2, sz2 = '-';
		if (fx < 0)
			fx = -fx, sfx = '-';
		//snd_iprintf(buffer, "%d  %d  %05x  %02x  %c%08x  %c%08x  %c%08x\n",
		//	    emu->snap_inter[c].r,
		//	    emu->snap_inter[c].s,
		//	    REG_VAL_GET(WC_SAMPLECOUNTER, emu->snap_inter[c].wc),
		//	    REG_VAL_GET(WC_CURRENTCHANNEL, emu->snap_inter[c].wc),
		//	    sz1, z1,
		//	    sz2, z2,
		//	    sfx, fx);
		int wz1 = z1 >> 24;
		int wz2 = z2 >> 24;
		int wfx = fx >> 24;
		int dz1 = ((z1 & 0xffffff) * 1000LL) >> 24;
		int dz2 = ((z2 & 0xffffff) * 1000LL) >> 24;
		int dfx = ((fx & 0xffffff) * 1000LL) >> 24;
		snd_iprintf(buffer, "%d  %d  %05x  %02x  %c% 3d.%03d  %c% 3d.%03d  %c% 3d.%03d\n",
			    emu->snap_inter[c].r,
			    emu->snap_inter[c].s,
			    REG_VAL_GET(WC_SAMPLECOUNTER, emu->snap_inter[c].wc),
			    REG_VAL_GET(WC_CURRENTCHANNEL, emu->snap_inter[c].wc),
			    sz1, wz1, dz1,
			    sz2, wz2, dz2,
			    sfx, wfx, dfx);
	}
	snd_iprintf(buffer, "\n");
#endif

#if SNAP_REGS
#if 0
	snd_iprintf(buffer, "wallclk CC ");
	for (int c = 0; c < emu->snap_total; c++) {
		snd_iprintf(buffer, "| "
#if SNAP_REGS_AMOUNT
			"CS        "
#endif
#if SNAP_REGS_FILTER
			"Z1        Z2        "
#endif
			"PTRX      CP        CA        "
			//"FA        "
			"CRA CIS ");
	}
	snd_iprintf(buffer, "| ID\n");
	for (int n = 0; n < emu->num_reg_snaps; n++) {
		struct emu_reg_snapshot *rs = &emu->snap_regs[n];
		snd_iprintf(buffer, "%05x   %02x ",
			    REG_VAL_GET(WC_SAMPLECOUNTER, rs->wc),
			    REG_VAL_GET(WC_CURRENTCHANNEL, rs->wc));
		for (int c = 0; c < emu->snap_total; c++) {
			snd_iprintf(buffer, "| ");
			if (c == rs->voice || rs->voice == -1) {
#if SNAP_REGS_AMOUNT
				snd_iprintf(buffer, "%08x  ", rs->csba[c]);
#endif
#if SNAP_REGS_FILTER
				snd_iprintf(buffer, "%08x  %08x  ", rs->z1[c] << 5, rs->z2[c] << 5);
#endif
				snd_iprintf(buffer, "%08x  %08x  %08x  ",
					    rs->ptrx[c], rs->cpf[c],
					    REG_VAL_GET(CCCA_CURRADDR, rs->ccca[c]));
				//snd_iprintf(buffer, "%08x  ", rs->cpf[c] & CPF_FRACADDRESS_MASK);
				snd_iprintf(buffer, "%02x  %02x  ",
					    REG_VAL_GET(CCR_READADDRESS, rs->ccr[c]),
					    REG_VAL_GET(CCR_CACHEINVALIDSIZE, rs->ccr[c]));
			} else {
				snd_iprintf(buffer,
#if SNAP_REGS_AMOUNT
					"          "
#endif
#if SNAP_REGS_FILTER
					"                    "
#endif
					"                              "
					//"          "
					"        ");
			}
		}
		snd_iprintf(buffer, "| %s\n", rs->lbl);
	}
#else
	snd_iprintf(buffer, "wallclk CC V  "
#if SNAP_REGS_AMOUNT
		"CS        "
#endif
#if SNAP_REGS_FILTER
		"Z1        Z2        "
#endif
		"PTRX      CP        CA        "
		//"FA        "
		"CLA     CRA CIS LIS CLF LF ID\n");
	for (int n = 0; n < emu->num_reg_snaps; n++) {
		struct emu_reg_snapshot *rs = &emu->snap_regs[n];
		if (rs->lbl)
			snd_iprintf(buffer, "%05x   %02x ",
				    REG_VAL_GET(WC_SAMPLECOUNTER, rs->wc),
				    REG_VAL_GET(WC_CURRENTCHANNEL, rs->wc));
		else
			snd_iprintf(buffer, "           ");
		snd_iprintf(buffer, "%2d ", rs->voice);
#if SNAP_REGS_AMOUNT
		snd_iprintf(buffer, "%08x  ", rs->csba);
#endif
#if SNAP_REGS_FILTER
		snd_iprintf(buffer, "%08x  %08x  ", rs->z1 << 5, rs->z2 << 5);
#endif
		snd_iprintf(buffer, "%08x  %08x  %08x  ",
			    rs->ptrx, rs->cpf,
			    REG_VAL_GET(CCCA_CURRADDR, rs->ccca));
		//snd_iprintf(buffer, "%08x  ", rs->cpf & CPF_FRACADDRESS_MASK);
		snd_iprintf(buffer, "%06x  %02x  %02x  %02x  %c   %c  ",
			    (REG_VAL_GET(CCR_CACHELOOPADDRHI, rs->ccr) << 16) |
				    REG_VAL_GET(CLP_CACHELOOPADDR, rs->clp),
			    REG_VAL_GET(CCR_READADDRESS, rs->ccr),
			    REG_VAL_GET(CCR_CACHEINVALIDSIZE, rs->ccr),
			    REG_VAL_GET(CCR_LOOPINVALSIZE, rs->ccr),
			    rs->ccr & CCR_CACHELOOPFLAG ? '1' : '0',
			    rs->ccr & CCR_LOOPFLAG ? '1' : '0');
		snd_iprintf(buffer, "%s\n", rs->lbl ? rs->lbl : "");
	}
#endif
	snd_iprintf(buffer, "\n");
#endif

#if PROC_CTL_CACHE
	snd_iprintf(buffer, "init:  read_addr=%06x  cache_read_addr=%02x  cache_inval=%02x\n"
		            "       cache_loop_addr=%06x  cache_loop_inval=%02x  cache_loop_flag=%d  loop_flag=%d\n\n",
		    emu->init_read_addr, emu->init_cache_read_addr, emu->init_cache_inval,
		    emu->init_cache_loop_addr, emu->init_cache_loop_inval,
		    emu->init_cache_loop_flag, emu->init_loop_flag);
#endif
#if PROC_CTL_AMOUNTS
	snd_iprintf(buffer, "init:  send_amount=%02x\n\n", emu->init_send_amount);
#endif
#if PROC_CTL_LOOP
	snd_iprintf(buffer, "init:  loop_start=%#x  loop_end=%#x\n\n",
		    emu->init_loop_start, emu->init_loop_end);
#endif

#if SNAP_CACHE
	snd_iprintf(buffer, "VC  RDA     CLA     CRA CIS LIS CLF LF ILS WSS CD\n");
	for (int c = 0; c < emu->num_cache_snaps; c++) {
		struct emu_cache_snapshot *sc = &emu->snap_cache[c];
		int ils = sc->ccr & CCR_INTERLEAVEDSAMPLES;
		int wss = sc->ccr & CCR_WORDSIZEDSAMPLES;
		snd_iprintf(buffer, "%2d  %06x  %06x  %02x  %02x  %02x  %c   %c  %c   %c  ",
			    sc->voice,
			    REG_VAL_GET(CCCA_CURRADDR, sc->ccca),
			    (REG_VAL_GET(CCR_CACHELOOPADDRHI, sc->ccr) << 16) |
				    REG_VAL_GET(CLP_CACHELOOPADDR, sc->clp),
			    REG_VAL_GET(CCR_READADDRESS, sc->ccr),
			    REG_VAL_GET(CCR_CACHEINVALIDSIZE, sc->ccr),
			    REG_VAL_GET(CCR_LOOPINVALSIZE, sc->ccr),
			    sc->ccr & CCR_CACHELOOPFLAG ? '1' : '0',
			    sc->ccr & CCR_LOOPFLAG ? '1' : '0',
			    ils ? '1' : '0',
			    wss ? '1' : '0');
		for (int l = 0; l < 2; l++) {
			if (l)
				snd_iprintf(buffer, "                                              ");
			for (int ci = 0; ci < 16; ci++) {
				u32 w = sc->cd[l * 16 + ci];
				if (wss && ils)
					snd_iprintf(buffer, " %08x", w);
				else if (wss || ils)
					snd_iprintf(buffer, "  %04x %04x", w & 0xffff, w >> 16);
				else
					snd_iprintf(buffer, "  %02x %02x %02x %02x",
						    w & 0xff, (w >> 8) & 0xff,
						    (w >> 16) & 0xff, w >> 24);
			}
			snd_iprintf(buffer, "\n");
		}
	}
	snd_iprintf(buffer, "\n");
#endif

#if SNAP_SAMPLES
	snd_iprintf(buffer,
#if SNAP_SAMPLES_TIME
		"dWallclk "
#endif
#if SNAP_SAMPLES_WC
		"wordclk CC  "
#endif
#if SNAP_SAMPLES_VALUES
		"sample     "
#endif
#if SNAP_SAMPLES_ADDR || SNAP_SAMPLES_XADDR
		"RDA      "
#endif
#if SNAP_SAMPLES_CACHE || SNAP_SAMPLES_XCACHE
		"CRA  CIS  "
		"CLF LF  LIS  "
#if SNAP_SAMPLES_CACHE_CLP
		"CLA     "
#endif
#endif
#if SNAP_SAMPLES_FILTER
#  if SNAP_INIT && SNAP_INIT_VOL
		"Z1/i  Z2/i  "
#  endif
		"Z1 << 5   Z2 << 5  "
#endif
#if SNAP_SAMPLES_MISC || SNAP_SAMPLES_XMISC
		"0x0e      0x0f      0x1f      0x57      0x7f"
#endif
		"\n");
#if SNAP_SAMPLES_FILTER && SNAP_INIT && SNAP_INIT_VOL
	u32 cx = emu->snap_width == 16 ? 0 : 0x80;
#endif
	for (int n = 0; n < emu->num_sample_snaps; n++) {
		struct emu_sample_snapshot *ss = &emu->snapshots[n];
#if SNAP_SAMPLES_WC
		int wc = REG_VAL_GET(WC_SAMPLECOUNTER, ss->wc);
		char wid = ' ';
		if (n) {
			int prev_wc = REG_VAL_GET(WC_SAMPLECOUNTER, ss[-1].wc);
			if (wc == prev_wc + 1) {
				wid = '*';
				if (emu->snap_chans > 1)
					snd_iprintf(buffer, "\n");
			} else if (wc != prev_wc)
				snd_iprintf(buffer, "----\n");
			else if (emu->snap_chans > 1)
				snd_iprintf(buffer, "\n");
		}
#endif
#if SNAP_SAMPLES_TIME
		snd_iprintf(buffer, "%7ld  ", n ? ss->ts.tv_nsec - ss[-1].ts.tv_nsec : 0);
#endif
#if SNAP_SAMPLES_WC
		snd_iprintf(buffer, "%05x%c  %02x  ",
			    wc, wid, REG_VAL_GET(WC_CURRENTCHANNEL, ss->wc));
#endif
#if SNAP_SAMPLES_XADDR || SNAP_SAMPLES_XCACHE || SNAP_SAMPLES_XMISC
#if SNAP_SAMPLES_XADDR
		int xca = REG_VAL_GET(CCCA_CURRADDR, ss->xccca);
		char xcaid = ' ';
#endif
#if SNAP_SAMPLES_XCACHE
		int xcra = REG_VAL_GET(CCR_READADDRESS, ss->xccr);
		int xcis = REG_VAL_GET(CCR_CACHEINVALIDSIZE, ss->xccr);
		char xcraid = ' ', xcisid = ' ';
#endif
		if (n) {
#if SNAP_SAMPLES_XADDR
			int prev_xca = REG_VAL_GET(CCCA_CURRADDR, ss[-1].xccca);
			if (xca != prev_xca)
				xcaid = '*';
#endif
#if SNAP_SAMPLES_XCACHE
			int prev_xcra = REG_VAL_GET(CCR_READADDRESS, ss[-1].xccr);
			int prev_xcis = REG_VAL_GET(CCR_CACHEINVALIDSIZE, ss[-1].xccr);
			if (xcra != prev_xcra)
				xcraid = '*';
			if (xcis != prev_xcis)
				xcisid = '*';
#endif
		}
#if SNAP_SAMPLES_VALUES
		snd_iprintf(buffer, "           ");
#endif
#if SNAP_SAMPLES_XADDR
		snd_iprintf(buffer, "%06x%c  ", xca, xcaid);
#endif
#if SNAP_SAMPLES_XCACHE
		snd_iprintf(buffer, "%02x%c  %02x%c  ", xcra, xcraid, xcis, xcisid);
#endif
#if SNAP_SAMPLES_XMISC
		snd_iprintf(buffer, "%08x  %08x  %08x  %08x  %08x",
			    ss->xreg0e, ss->xreg0f, ss->xreg1f, ss->xreg57, ss->xreg7f);
#endif
		snd_iprintf(buffer, "\n");
#endif
#if SNAP_SAMPLES_VALUES || SNAP_SAMPLES_ADDR || SNAP_SAMPLES_CACHE || SNAP_SAMPLES_FILTER
		for (int c = 0; c < emu->snap_chans; c++) {
#if SNAP_SAMPLES_VALUES
			int sam = ss->fxbus[c];
			char samid = ' ';
#endif
#if SNAP_SAMPLES_ADDR
			int ca = REG_VAL_GET(CCCA_CURRADDR, ss->ccca[c]);
			char caid = ' ';
#endif
#if SNAP_SAMPLES_CACHE
			int cra = REG_VAL_GET(CCR_READADDRESS, ss->ccr[c]);
			int cis = REG_VAL_GET(CCR_CACHEINVALIDSIZE, ss->ccr[c]);
			char craid = ' ', cisid = ' ';
			int clf = !!(ss->ccr[c] & CCR_CACHELOOPFLAG);
			int lf = !!(ss->ccr[c] & CCR_LOOPFLAG);
			int lis = REG_VAL_GET(CCR_LOOPINVALSIZE, ss->ccr[c]);
			char clfid = ' ', lfid = ' ', lisid = ' ';
#if SNAP_SAMPLES_CACHE_CLP
			int cla = (REG_VAL_GET(CCR_CACHELOOPADDRHI, ss->ccr[c]) << 16) |
					REG_VAL_GET(CLP_CACHELOOPADDR, ss->clp[c]);
			char claid = ' ';
#endif
#endif
#if SNAP_SAMPLES_FILTER && SNAP_INIT && SNAP_INIT_VOL
			u32 cv = max(emu->snap_chan[c].cvcf >> 16, 1U);
			u32 cm = 0xffff;
			if (cx) cv *= 256, cm = 0xff;
#endif
			if (n) {
#if SNAP_SAMPLES_VALUES
				int prev_sam = ss[-1].fxbus[c];
				if ((unsigned)sam < (unsigned)prev_sam)
					samid = '^';
				else if (sam != prev_sam)
					samid = '*';
#endif
#if SNAP_SAMPLES_ADDR
				int prev_ca = REG_VAL_GET(CCCA_CURRADDR, ss[-1].ccca[c]);
				if (ca < prev_ca)
					caid = '^';
				else if (ca != prev_ca)
					caid = '*';
#endif
#if SNAP_SAMPLES_CACHE
				int prev_cra = REG_VAL_GET(CCR_READADDRESS, ss[-1].ccr[c]);
				int prev_cis = REG_VAL_GET(CCR_CACHEINVALIDSIZE, ss[-1].ccr[c]);
				if (cra != prev_cra)
					craid = '*';
				if (cis != prev_cis)
					cisid = '*';
				int prev_clf = !!(ss[-1].ccr[c] & CCR_CACHELOOPFLAG);
				int prev_lf = !!(ss[-1].ccr[c] & CCR_LOOPFLAG);
				int prev_lis = REG_VAL_GET(CCR_LOOPINVALSIZE, ss[-1].ccr[c]);
				if (clf != prev_clf)
					clfid = '*';
				if (lf != prev_lf)
					lfid = '*';
				if (lis != prev_lis)
					lisid = '*';
#if SNAP_SAMPLES_CACHE_CLP
				int prev_cla = (REG_VAL_GET(CCR_CACHELOOPADDRHI, ss[-1].ccr[c]) << 16) |
						REG_VAL_GET(CLP_CACHELOOPADDR, ss[-1].clp[c]);
				if (cla != prev_cla)
					claid = '*';
#endif
#endif
			}
			if (c) {
#if SNAP_SAMPLES_TIME
				snd_iprintf(buffer, "         ");
#endif
#if SNAP_SAMPLES_WC
				snd_iprintf(buffer, "          ");
#endif
			}
#if SNAP_SAMPLES_VALUES
			snd_iprintf(buffer, "%08x%c  ", sam, samid);
#endif
#if SNAP_SAMPLES_ADDR
			snd_iprintf(buffer, "%06x%c  ", ca, caid);
#elif SNAP_SAMPLES_XADDR
			snd_iprintf(buffer, "         ");
#endif
#if SNAP_SAMPLES_CACHE
			snd_iprintf(buffer, "%02x%c  %02x%c  ", cra, craid, cis, cisid);
			snd_iprintf(buffer, "%d%c  %d%c  %02x%c  ",
				    clf, clfid, lf, lfid, lis, lisid);
#if SNAP_SAMPLES_CACHE_CLP
			snd_iprintf(buffer, "%06x%c  ", cla, claid);
#endif
#elif SNAP_SAMPLES_XCACHE
			snd_iprintf(buffer, "          ");
#endif
#if SNAP_SAMPLES_FILTER
#  if SNAP_INIT && SNAP_INIT_VOL
			snd_iprintf(buffer, "%04x  %04x  ",
				    (ss->z1[c] * 16 / cv ^ cx) & cm, (ss->z2[c] * 16 / cv ^ cx) & cm);
#  endif
			snd_iprintf(buffer, "%08x  %08x  ",
				    ss->z1[c] << 5, ss->z2[c] << 5);
#endif
#if SNAP_SAMPLES_MISC
			snd_iprintf(buffer, "%08x  %08x  %08x  %08x  %08x\n",
				    ss->reg0e[c], ss->reg0f[c], ss->reg1f[c], ss->reg57[c], ss->reg7f[c]);
#endif
			snd_iprintf(buffer, "\n");
		}
#endif
	}
	snd_iprintf(buffer, "\n");
#endif

#if SNAP_INPUT
//	snd_iprintf(buffer, "avail_max = 0x%lx  dma_bytes = 0x%lx\n"
//			    "hw_ptr_base = 0x%lx  hw_ptr_interrup = 0x%lx  hw_ptr_wrap = 0x%llx\n\n",
//			    emu->snap_avail_max, emu->snap_dma_bytes,
//			    emu->snap_hw_ptr_base, emu->snap_hw_ptr_interrupt, emu->snap_hw_ptr_wrap);

	if (emu->snap_buffer) {
		int ssft = (emu->snap_width == 32) + (emu->snap_width >= 16) + emu->snap_stereo;
		if (emu->snap_il) {
			snd_emu_proc_fmt_input(emu, buffer, 0, emu->snap_dma_bytes, emu->snap_start >> ssft);
		} else {
			int o = 0;
			int s = emu->snap_start;
			int n = emu->snap_dma_bytes / (emu->snap_raw_chans * emu->snap_sub_chans);
			for (int c = 0; c < emu->snap_raw_chans; c++) {
				for (int sc = 0; sc < emu->snap_sub_chans; sc++) {
					snd_iprintf(buffer, "Channel %d:%d:\n", c, sc);
					snd_emu_proc_fmt_input(emu, buffer, o, n, s >> ssft);
					snd_iprintf(buffer, "\n");
					o += n;
					s += n;
				}
			}
		}
	}
#endif

	atomic_set(&emu->snapshot_busy, 0);
}

#if PROC_CTL_CACHE || PROC_CTL_AMOUNTS || PROC_CTL_LOOP || SNAP_OTHER
static void snd_emu_proc_snapshot_write(struct snd_info_entry *entry,
                                      struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;
	char line[64];

	while (!snd_info_get_line(buffer, line, sizeof(line))) {
#if SNAP_OTHER
		if (!strcmp(line, "snapshot")) {
			if (atomic_xchg(&emu->snapshot_busy, 1))
				continue;
			snd_emu10k1_snapshot_other(emu);
			atomic_set(&emu->snapshot_busy, 0);
			continue;
		}
#endif
#if PROC_CTL_CACHE || PROC_CTL_AMOUNTS || PROC_CTL_LOOP
		char reg[64];
		u32 val;
		if (sscanf(line, "%63s %i", reg, &val) != 2)
			continue;
		if (0)
			{}
#if PROC_CTL_CACHE
		else if (!strcmp(reg, "read_addr"))
			emu->init_read_addr = val;
		else if (!strcmp(reg, "cache_read_addr"))
			emu->init_cache_read_addr = val;
		else if (!strcmp(reg, "cache_inval"))
			emu->init_cache_inval = val;
		else if (!strcmp(reg, "cache_loop_addr"))
			emu->init_cache_loop_addr = val;
		else if (!strcmp(reg, "cache_loop_inval"))
			emu->init_cache_loop_inval = val;
		else if (!strcmp(reg, "cache_loop_flag"))
			emu->init_cache_loop_flag = val;
		else if (!strcmp(reg, "loop_flag"))
			emu->init_loop_flag = val;
#endif
#if PROC_CTL_AMOUNTS
		else if (!strcmp(reg, "send_amount"))
			emu->init_send_amount = val;
#endif
#if PROC_CTL_LOOP
		else if (!strcmp(reg, "loop_start"))
			emu->init_loop_start = val;
		else if (!strcmp(reg, "loop_end"))
			emu->init_loop_end = val;
#endif
#endif
	}
}
#endif

static void snd_emu_proc_io_reg_read(struct snd_info_entry *entry,
				     struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;
	unsigned long value;
	int i;
	snd_iprintf(buffer, "IO Registers:\n\n");
	for(i = 0; i < 0x40; i+=4) {
		value = inl(emu->port + i);
		snd_iprintf(buffer, "%02X: %08lX\n", i, value);
	}
}

static void snd_emu_proc_io_reg_write(struct snd_info_entry *entry,
                                      struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;
	char line[64];
	u32 reg, val;
	while (!snd_info_get_line(buffer, line, sizeof(line))) {
		if (sscanf(line, "%x %x", &reg, &val) != 2)
			continue;
		if (reg < 0x40 && val <= 0xffffffff) {
			outl(val, emu->port + (reg & 0xfffffffc));
		}
	}
}

static unsigned int snd_ptr_read(struct snd_emu10k1 * emu,
				 unsigned int iobase,
				 unsigned int reg,
				 unsigned int chn)
{
	unsigned int regptr, val;

	regptr = (reg << 16) | chn;

	spin_lock_irq(&emu->emu_lock);
	outl(regptr, emu->port + iobase + PTR);
	val = inl(emu->port + iobase + DATA);
	spin_unlock_irq(&emu->emu_lock);
	return val;
}

static void snd_ptr_write(struct snd_emu10k1 *emu,
			  unsigned int iobase,
			  unsigned int reg,
			  unsigned int chn,
			  unsigned int data)
{
	unsigned int regptr;

	regptr = (reg << 16) | chn;

	spin_lock_irq(&emu->emu_lock);
	outl(regptr, emu->port + iobase + PTR);
	outl(data, emu->port + iobase + DATA);
	spin_unlock_irq(&emu->emu_lock);
}


static void snd_emu_proc_ptr_reg_read(struct snd_info_entry *entry,
				      struct snd_info_buffer *buffer, int iobase, int offset, int length, int voices)
{
	struct snd_emu10k1 *emu = entry->private_data;
	unsigned long value;
	int i,j;
	if (offset+length > 0xa0) {
		snd_iprintf(buffer, "Input values out of range\n");
		return;
	}
	snd_iprintf(buffer, "Registers 0x%x\n", iobase);
	for(i = offset; i < offset+length; i++) {
		snd_iprintf(buffer, "%02X: ",i);
		for (j = 0; j < voices; j++) {
			value = snd_ptr_read(emu, iobase, i, j);
			snd_iprintf(buffer, "%08lX ", value);
		}
		snd_iprintf(buffer, "\n");
	}
}

static void snd_emu_proc_ptr_reg_write(struct snd_info_entry *entry,
				       struct snd_info_buffer *buffer,
				       int iobase, int length, int voices)
{
	struct snd_emu10k1 *emu = entry->private_data;
	char line[64];
	unsigned int reg, channel_id , val;
	while (!snd_info_get_line(buffer, line, sizeof(line))) {
		if (sscanf(line, "%x %x %x", &reg, &channel_id, &val) != 3)
			continue;
		if (reg < length && channel_id < voices)
			snd_ptr_write(emu, iobase, reg, channel_id, val);
	}
}

static void snd_emu_proc_ptr_reg_write00(struct snd_info_entry *entry,
					 struct snd_info_buffer *buffer)
{
	snd_emu_proc_ptr_reg_write(entry, buffer, 0, 0x80, 64);
}

static void snd_emu_proc_ptr_reg_write20(struct snd_info_entry *entry,
					 struct snd_info_buffer *buffer)
{
	struct snd_emu10k1 *emu = entry->private_data;
	snd_emu_proc_ptr_reg_write(entry, buffer, 0x20,
				   emu->card_capabilities->ca0108_chip ? 0xa0 : 0x80, 4);
}
	

static void snd_emu_proc_ptr_reg_read00a(struct snd_info_entry *entry,
					 struct snd_info_buffer *buffer)
{
	snd_emu_proc_ptr_reg_read(entry, buffer, 0, 0, 0x40, 64);
}

static void snd_emu_proc_ptr_reg_read00b(struct snd_info_entry *entry,
					 struct snd_info_buffer *buffer)
{
	snd_emu_proc_ptr_reg_read(entry, buffer, 0, 0x40, 0x40, 64);
}

static void snd_emu_proc_ptr_reg_read20a(struct snd_info_entry *entry,
					 struct snd_info_buffer *buffer)
{
	snd_emu_proc_ptr_reg_read(entry, buffer, 0x20, 0, 0x40, 4);
}

static void snd_emu_proc_ptr_reg_read20b(struct snd_info_entry *entry,
					 struct snd_info_buffer *buffer)
{
	snd_emu_proc_ptr_reg_read(entry, buffer, 0x20, 0x40, 0x40, 4);
}

static void snd_emu_proc_ptr_reg_read20c(struct snd_info_entry *entry,
					 struct snd_info_buffer * buffer)
{
	snd_emu_proc_ptr_reg_read(entry, buffer, 0x20, 0x80, 0x20, 4);
}
#endif

static const struct snd_info_entry_ops snd_emu10k1_proc_ops_fx8010 = {
	.read = snd_emu10k1_fx8010_read,
};

int snd_emu10k1_proc_init(struct snd_emu10k1 *emu)
{
	struct snd_info_entry *entry;
#ifdef CONFIG_SND_DEBUG
	if (emu->card_capabilities->emu_model) {
		snd_card_ro_proc_new(emu->card, "emu1010_regs",
				     emu, snd_emu_proc_emu1010_reg_read);
	}
	snd_card_rw_proc_new(emu->card, "io_regs", emu,
			     snd_emu_proc_io_reg_read,
			     snd_emu_proc_io_reg_write);
	snd_card_rw_proc_new(emu->card, "ptr_regs00a", emu,
			     snd_emu_proc_ptr_reg_read00a,
			     snd_emu_proc_ptr_reg_write00);
	snd_card_rw_proc_new(emu->card, "ptr_regs00b", emu,
			     snd_emu_proc_ptr_reg_read00b,
			     snd_emu_proc_ptr_reg_write00);
	if (!emu->card_capabilities->emu_model &&
	    (emu->card_capabilities->ca0151_chip || emu->card_capabilities->ca0108_chip)) {
		snd_card_rw_proc_new(emu->card, "ptr_regs20a", emu,
				     snd_emu_proc_ptr_reg_read20a,
				     snd_emu_proc_ptr_reg_write20);
		snd_card_rw_proc_new(emu->card, "ptr_regs20b", emu,
				     snd_emu_proc_ptr_reg_read20b,
				     snd_emu_proc_ptr_reg_write20);
		if (emu->card_capabilities->ca0108_chip)
			snd_card_rw_proc_new(emu->card, "ptr_regs20c", emu,
					     snd_emu_proc_ptr_reg_read20c,
					     snd_emu_proc_ptr_reg_write20);
	}

	snd_card_rw_proc_new(emu->card, "snapshot", emu,
			     snd_emu_proc_snapshot_read,
#if PROC_CTL_CACHE || PROC_CTL_AMOUNTS || PROC_CTL_LOOP || SNAP_OTHER
			     snd_emu_proc_snapshot_write);
#else
			     NULL);
#endif
#endif
	
	snd_card_ro_proc_new(emu->card, "emu10k1", emu, snd_emu10k1_proc_read);

	if (emu->card_capabilities->emu10k2_chip)
		snd_card_ro_proc_new(emu->card, "spdif-in", emu,
				     snd_emu10k1_proc_spdif_read);
	if (emu->card_capabilities->ca0151_chip)
		snd_card_ro_proc_new(emu->card, "capture-rates", emu,
				     snd_emu10k1_proc_rates_read);

	snd_card_ro_proc_new(emu->card, "voices", emu,
			     snd_emu10k1_proc_voices_read);

	if (! snd_card_proc_new(emu->card, "fx8010_gpr", &entry)) {
		entry->content = SNDRV_INFO_CONTENT_DATA;
		entry->private_data = emu;
		entry->mode = S_IFREG | 0444 /*| S_IWUSR*/;
		entry->size = emu->audigy ? A_TOTAL_SIZE_GPR : TOTAL_SIZE_GPR;
		entry->c.ops = &snd_emu10k1_proc_ops_fx8010;
	}
	if (! snd_card_proc_new(emu->card, "fx8010_tram_data", &entry)) {
		entry->content = SNDRV_INFO_CONTENT_DATA;
		entry->private_data = emu;
		entry->mode = S_IFREG | 0444 /*| S_IWUSR*/;
		entry->size = emu->audigy ? A_TOTAL_SIZE_TANKMEM_DATA : TOTAL_SIZE_TANKMEM_DATA ;
		entry->c.ops = &snd_emu10k1_proc_ops_fx8010;
	}
	if (! snd_card_proc_new(emu->card, "fx8010_tram_addr", &entry)) {
		entry->content = SNDRV_INFO_CONTENT_DATA;
		entry->private_data = emu;
		entry->mode = S_IFREG | 0444 /*| S_IWUSR*/;
		entry->size = emu->audigy ? A_TOTAL_SIZE_TANKMEM_ADDR : TOTAL_SIZE_TANKMEM_ADDR ;
		entry->c.ops = &snd_emu10k1_proc_ops_fx8010;
	}
	if (! snd_card_proc_new(emu->card, "fx8010_code", &entry)) {
		entry->content = SNDRV_INFO_CONTENT_DATA;
		entry->private_data = emu;
		entry->mode = S_IFREG | 0444 /*| S_IWUSR*/;
		entry->size = emu->audigy ? A_TOTAL_SIZE_CODE : TOTAL_SIZE_CODE;
		entry->c.ops = &snd_emu10k1_proc_ops_fx8010;
	}
	snd_card_ro_proc_new(emu->card, "fx8010_acode", emu,
			     snd_emu10k1_proc_acode_read);
	return 0;
}
