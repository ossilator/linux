// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *                   Lee Revell <rlrevell@joe-job.com>
 *                   James Courtier-Dutton <James@superbug.co.uk>
 *                   Oswald Buddenhagen <oswald.buddenhagen@gmx.de>
 *                   Creative Labs, Inc.
 *
 *  Routines for control of EMU10K1 chips / PCM routines
 */

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <sound/core.h>
#include <sound/emu10k1.h>

#if SNAP_SETUP
static u32 snap_read(struct snd_emu10k1 *emu, unsigned int reg)
{
	return inl(emu->port + reg);
}

static u32 snap_ptr_read(struct snd_emu10k1 *emu, unsigned int reg, unsigned int chn)
{
	outl((reg << 16) | chn, emu->port + PTR);
	return inl(emu->port + DATA);
}
#endif

#if LOG_CAP_POS || LOG_PB_POS || LOG_PB_FILL || LOG_IOCTLS
static void log_event(struct snd_emu10k1 *emu, int log, int val)
{
	snd_emu10k1_ptr_write(emu, A_FXGPREGBASE + emu->fngprs + log, 0,
				0x7fffffffLL * val / 100);
}

static void log_strm_event(struct snd_pcm_substream *substream, int log, int val)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream)
		log_event(emu, log, val);
}
#endif

#if LOG_IOCTLS
static void log_strm_ctl_event(struct snd_pcm_substream *substream, int val)
{
	log_strm_event(substream, IoctlLogReg, val);
}
#else
#define log_strm_ctl_event(strm, val) do {} while (0)
#endif

#if 1
#define snap_info(...) dev_info(emu->card->dev, __VA_ARGS__)
#else
#define snap_info(...) do {} while (0)
#endif

#if SNAP_THREAD
static int snapshot_thread(void *aux)
{
	struct snd_emu10k1 *emu = aux;
	struct snd_pcm_substream *strm;
	struct snd_pcm_runtime *runtime;
	struct snd_emu10k1_pcm *epcm;
	unsigned long flags;
	int num = 0;

	snap_info("snapshotter entered\n");
	smp_store_release(&emu->snapshot_rsp, SNAP_SLEEPING);
loop1:
	switch (smp_load_acquire(&emu->snapshot_cmd)) {
	case SNAP_STOP:
		snap_info("stopping snapshotter before preparation\n");
		goto huuh;
	case SNAP_PREPARE: break;
	default: goto loop1;
	}
	if (!(strm = emu->snapshot_stream) || !(runtime = strm->runtime)) {
		dev_err(emu->card->dev, "huh, no open playback streams\n");
		goto huuh;
	}
	epcm = runtime->private_data;
	if (!epcm) {
		dev_err(emu->card->dev, "huh, no pcm\n");
		goto huuh;
	}
	snap_info("snapshotting %s\n", epcm->substream->name);
#if RECORD_FX
	snap_info("fx gprs %x\n", emu->fxgprs);
#endif

#if SNAP_SAMPLES
	if (!emu->snapshots) {
		dev_err(emu->card->dev, "huh, no snapshot buffer\n");
		goto huuh;
	}
#endif

#if SNAP_INPUT
	//emu->snap_avail_max = runtime->avail_max;
	//emu->snap_hw_ptr_base = runtime->hw_ptr_base;	/* Position at buffer restart */
	//emu->snap_hw_ptr_interrupt = runtime->hw_ptr_interrupt; /* Position at interrupt time */
	//emu->snap_hw_ptr_wrap = runtime->hw_ptr_wrap;
	emu->snap_dma_bytes = runtime->dma_bytes;
	memcpy(emu->snap_buffer, runtime->dma_area, runtime->dma_bytes);
#endif

	smp_store_release(&emu->snapshot_rsp, SNAP_PREPARED);
loop2:
	switch (smp_load_acquire(&emu->snapshot_cmd)) {
	case SNAP_STOP:
		snap_info("stopping snapshotter before start\n");
		goto huuh;
	case SNAP_START: break;
	default: goto loop2;
	}
	snap_info("start requested\n");
	smp_store_release(&emu->snapshot_rsp, SNAP_STARTED);

#if SNAP_SAMPLES
	for (; num < NUM_EMU_SNAPSHOTS; num++) {
		struct emu_sample_snapshot *ss = &emu->snapshots[num];
#if 1
		spin_lock_irqsave(&emu->emu_lock, flags);
#if SNAP_SAMPLES_WC
		ss->wc = snap_read(emu, WC);
#endif
#if SNAP_SAMPLES_XADDR || SNAP_SAMPLES_XCACHE || SNAP_SAMPLES_XMISC
		int xv = emu->snap_voices[emu->snap_chans];
#if SNAP_SAMPLES_XADDR
		ss->xccca = snap_ptr_read(emu, CCCA, xv);
#endif
#if SNAP_SAMPLES_XCACHE
		ss->xccr = snap_ptr_read(emu, CCR, xv);
#endif
#if SNAP_SAMPLES_XMISC
		ss->xreg0e = snap_ptr_read(emu, 0x0e, xv);
		ss->xreg0f = snap_ptr_read(emu, 0x0f, xv);
		ss->xreg1f = snap_ptr_read(emu, 0x1f, xv);
		ss->xreg57 = snap_ptr_read(emu, 0x57, xv);
		ss->xreg7f = snap_ptr_read(emu, 0x7f, xv);
#endif
#endif
#if SNAP_SAMPLES_VALUES || SNAP_SAMPLES_ADDR || SNAP_SAMPLES_CACHE || SNAP_SAMPLES_FILTER
		for (int c = 0; c < emu->snap_chans; c++) {
#if SNAP_SAMPLES_ADDR || SNAP_SAMPLES_CACHE || SNAP_SAMPLES_FILTER
			int cv = emu->snap_voices[c];
#endif
#if SNAP_SAMPLES_ADDR
			ss->ccca[c] = snap_ptr_read(emu, CCCA, cv);
#endif
#if SNAP_SAMPLES_CACHE
			ss->ccr[c] = snap_ptr_read(emu, CCR, cv);
#if SNAP_SAMPLES_CACHE_CLP
			ss->clp[c] = snap_ptr_read(emu, CLP, cv);
#endif
#endif
#if SNAP_SAMPLES_VALUES
			if (emu->snap_width == 32)
				ss->fxbus[c] = (c & 1) ? 0xdefea7ed : snap_ptr_read(emu, A_FXGPREGBASE + emu->fxgprs + (c >> 1), 0);
			else
				ss->fxbus[c] = snap_ptr_read(emu, A_FXGPREGBASE + emu->fxgprs + c, 0);
#endif
#if SNAP_SAMPLES_FILTER
			ss->z1[c] = snap_ptr_read(emu, Z1, cv);
			ss->z2[c] = snap_ptr_read(emu, Z2, cv);
#endif
#if SNAP_SAMPLES_MISC
			ss->reg0e[c] = snap_ptr_read(emu, 0x0e, cv);
			ss->reg0f[c] = snap_ptr_read(emu, 0x0f, cv);
			ss->reg1f[c] = snap_ptr_read(emu, 0x1f, cv);
			ss->reg57[c] = snap_ptr_read(emu, 0x57, cv);
			ss->reg7f[c] = snap_ptr_read(emu, 0x7f, cv);
#endif
		}
#endif
#if SNAP_SAMPLES_TIME
#if 1
		ktime_get_raw_ts64(&ss->ts);
#else
		ss->ts.tv_nsec = num * 1000;
#endif
#endif
		spin_unlock_irqrestore(&emu->emu_lock, flags);
		if (smp_load_acquire(&emu->snapshot_cmd) == SNAP_STOP)
			break;
#else
		for (;;) {
			if (smp_load_acquire(&emu->snapshot_cmd) == SNAP_STOP)
				goto huuh;
			newwc = snap_read(emu, WC);
			// alternatively, until WC_CURRENTCHANNEL changes - really high rate
			// - makes sense for IPR tracking
			//if (newwc != wc) {
			if ((newwc & WC_SAMPLECOUNTER_MASK) != (wc & WC_SAMPLECOUNTER_MASK)) {
				wc = newwc;
				break;
			}
		}
		ss->wc = wc;
		spin_lock_irqsave(&emu->emu_lock, flags);
		for (int c = 0; c < emu->snap_chans; c++) {
			int cv = voices[c];
			ss->ccca[c] = snap_ptr_read(emu, CCCA, cv);
			ss->ccr[c] = snap_ptr_read(emu, CCR, cv);
			//for (int ci = 0; ci < 16; ci++)
			//	ss->cd[c * 16 + ci] = snap_ptr_read(emu, CD0 + ci, cv);
			ss->fxbus[c] = snap_ptr_read(emu, A_FXGPREGBASE + /*emu->fxgprs +*/ c, 0);
			ss->cs[c] = snap_ptr_read(emu, A_CSBA, cv);
		}
		spin_unlock_irqrestore(&emu->emu_lock, flags);
		ktime_get_raw_ts64(&ss->ts);
		//ss->ts.tv_nsec = num * 1000;
#endif
	}
#else  // SNAP_IRQ_THREAD
#if SNAP_IRQ_CLIP_DRV
	u32 clipl = ~0, cliph = ~0;
#else
	u32 ipr = ~0;
#endif
#if SNAP_IRQ_CLIE
	u32 cliel = ~0, clieh = ~0;
#endif
#if SNAP_IRQ_ADDR_DRV
	int xv = emu->snap_voices[emu->snap_chans];
	u32 ccca = 0;
#endif
	for (; num < ARRAY_SIZE(emu->snap_irq); ) {
		struct emu_irq_snapshot *is = &emu->snap_irq[num];
		for (;;) {
			if (smp_load_acquire(&emu->snapshot_cmd) == SNAP_STOP)
				goto huuh;
			spin_lock_irqsave(&emu->emu_lock, flags);
#if SNAP_IRQ_CLIE
			cliel = snap_ptr_read(emu, CLIEL, 0);
			clieh = snap_ptr_read(emu, CLIEH, 0);
#endif
#if SNAP_IRQ_ADDR_DRV

			u32 pccca = ccca;
			ccca = snap_ptr_read(emu, CCCA, xv);
			// Ignore the non-address bits, as they are all constants.
			if (ccca < pccca)
				break;
#elif SNAP_IRQ_CLIP_DRV
			u32 nclipl = snap_ptr_read(emu, CLIPL, 0);
			u32 ncliph = snap_ptr_read(emu, CLIPH, 0);
			if (nclipl != clipl || ncliph != cliph) {
				clipl = nclipl;
				cliph = ncliph;
				break;
			}
#else
			u32 nipr = snap_read(emu, IPR);
			if (nipr != ipr) {
				ipr = nipr;
				break;
			}
#endif
#if SNAP_IRQ_CLIE
			if (!cliel && !clieh)
				break;
#endif
			spin_unlock_irqrestore(&emu->emu_lock, flags);
		}
#if SNAP_IRQ_CLIE
		int goout = 0;
		if (!cliel && !clieh) {
			goout = 1;
			goto goin;
		}
#endif
#if SNAP_IRQ_ADDR_DRV
#if SNAP_IRQ_CACHE
		u32 ccca2, ccr;
		do {
			ccca2 = ccca;
			ccr = snap_ptr_read(emu, CCR, xv);
			ccca = snap_ptr_read(emu, CCCA, xv);
		} while (ccca2 != ccca);
		is->ccr[emu->snap_chans] = ccr;
#endif
		is->ccca[emu->snap_chans] = ccca;
#elif SNAP_IRQ_CLIP_DRV
		if (clipl || cliph)
#else
		if (ipr)
#endif
		{
goin:
			is->wc = snap_read(emu, WC);
#if SNAP_IRQ_DICE
			is->dice = snap_ptr_read(emu, A_DICE, 0);
#endif
#if SNAP_IRQ_ADDR_DRV
			for (int c = 0; c < emu->snap_chans; c++) {
#else
			for (int c = 0; c < emu->snap_total; c++) {
#endif
				int cv = emu->snap_voices[c];
				u32 ccca = snap_ptr_read(emu, CCCA, cv);
#if SNAP_IRQ_CACHE
				u32 ccca2, ccr;
				do {
					ccca2 = ccca;
					ccr = snap_ptr_read(emu, CCR, cv);
					ccca = snap_ptr_read(emu, CCCA, cv);
				} while (ccca2 != ccca);
				is->ccr[c] = ccr;
#endif
				is->ccca[c] = ccca;
			}
#if SNAP_IRQ_CLIP_DRV || SNAP_IRQ_ADDR_DRV
			is->ipr = snap_read(emu, IPR);
#else
			is->ipr = ipr;
#endif
#if SNAP_IRQ_CLIP
#if SNAP_IRQ_CLIP_DRV
			is->clipl = clipl;
			is->cliph = cliph;
#else
			is->clipl = snap_ptr_read(emu, CLIPL, 0);
			is->cliph = snap_ptr_read(emu, CLIPH, 0);
#endif
#endif
#if SNAP_IRQ_HLIP
			is->hlipl = snap_ptr_read(emu, HLIPL, 0);
			is->hliph = snap_ptr_read(emu, HLIPH, 0);
#endif
#if SNAP_IRQ_CLIE
			is->cliel = cliel;
			is->clieh = clieh;
#endif
			num++;
		}
		spin_unlock_irqrestore(&emu->emu_lock, flags);
#if SNAP_IRQ_CLIE
		if (goout)
			break;
#endif
	}
#endif

huuh:
#if SNAP_SAMPLES
	emu->num_sample_snaps = num;
#else
	emu->num_irq_snaps = num;
#endif
	snap_info("snapshotter exiting\n");
	smp_store_release(&emu->snapshot_rsp, SNAP_STOPPED);
	return 0;
}
#endif

static void launch_snapshotter(struct snd_pcm_substream *substream)
{
#if SNAP_SETUP
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	snap_info("launching snapshotter\n");
#if SNAP_SAMPLES_XADDR || SNAP_SAMPLES_XCACHE || SNAP_SAMPLES_XMISC
	if (emu->das_mode) {
		dev_notice(emu->card->dev, "cannot snapshot extra voice in DAS mode\n");
		return;
	}
#endif
	if (!atomic_xchg(&emu->snapshot_busy, 1)) {
#if SNAP_THREAD
		struct task_struct *st = kthread_create_on_cpu(snapshot_thread, emu, 6, "emu-snapshotter");
		if (!st) {
			atomic_set(&emu->snapshot_busy, 0);
			dev_err(emu->card->dev, "cannot create snapshotter thread\n");
		} else {
			emu->snapshot_rsp = SNAP_NEW;
			emu->snapshot_cmd = SNAP_SLEEP;
			emu->snapshot_stream = substream;
			sched_set_fifo(st);
			wake_up_process(st);
		}
#else
		emu->snapshot_stream = substream;
#endif
	}
	else
		dev_notice(emu->card->dev, "snapshotter already exists\n");
#endif
}

#if SNAP_THREAD
static void snapshotter_state(struct snd_emu10k1 *emu, int *rsp)
{
	for (int niter = 0;; niter++) {
		int new_rsp = smp_load_acquire(&emu->snapshot_rsp);
		if (new_rsp != *rsp) {
			*rsp = new_rsp;
			return;
		}
		if (niter == 10000000)
			dev_notice(emu->card->dev, "iteration is taking too long\n");
	}
}
#endif

static void start_snapshotter(struct snd_pcm_substream *substream)
{
#if SNAP_THREAD
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	snap_info("starting snapshotter wanted\n");
	if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream) {
		int rsp = SNAP_NEW;
		for (;;) {
			snapshotter_state(emu, &rsp);
			switch (rsp) {
			case SNAP_SLEEPING:
				snap_info("requesting prepare\n");
				smp_store_release(&emu->snapshot_cmd, SNAP_PREPARE);
				break;
			case SNAP_PREPARED:
				snap_info("requesting start\n");
				smp_store_release(&emu->snapshot_cmd, SNAP_START);
				break;
			case SNAP_STARTED:
				snap_info("snapshotter reports start\n");
				return;
			case SNAP_STOPPED:
				// whoops
				snap_info("snapshotter unexpectedly reports done\n");
				emu->snapshot_stream = NULL;
				smp_mb__before_atomic();
				atomic_set(&emu->snapshot_busy, 0);
				return;
			}
		}
	}
	else
		dev_notice(emu->card->dev, "we don't own it\n");
#endif
}

static void stop_snapshotter(struct snd_pcm_substream *substream)
{
#if SNAP_SETUP
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	snap_info("stopping snapshotter wanted\n");
	if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream) {
#if SNAP_REGS
		dev_info(emu->card->dev, "tried %d register snapshots, %ssucessfully\n",
			 emu->tried_reg_snaps, emu->reg_snaps_full ? "NOT " : "");
#endif
#if SNAP_THREAD
		int rsp = SNAP_NEW;
		for (;;) {
			snapshotter_state(emu, &rsp);
			switch (rsp) {
			case SNAP_SLEEPING:
			case SNAP_PREPARED:
			case SNAP_STARTED:
				snap_info("requesting stop\n");
				smp_store_release(&emu->snapshot_cmd, SNAP_STOP);
				break;
			case SNAP_STOPPED:
				snap_info("snapshotter reports done\n");
				emu->snapshot_stream = NULL;
				smp_mb__before_atomic();
				atomic_set(&emu->snapshot_busy, 0);
				return;
			}
		}
#else
		emu->snapshot_stream = NULL;
		atomic_set(&emu->snapshot_busy, 0);
#endif
	}
	else
		dev_notice(emu->card->dev, "we don't own it\n");
#endif
}

static void setup_snapshotter(struct snd_pcm_substream *substream)
{
#if SNAP_SETUP
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	snap_info("snapshotter setup wanted\n");
	if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream) {
		struct snd_pcm_runtime *runtime = substream->runtime;
		struct snd_emu10k1_pcm *epcm = runtime->private_data;
		emu->snap_width = runtime->sample_bits;
		emu->snap_raw_chans = runtime->channels;
		emu->snap_sub_chans = 1 << emu->emu1010.clock_shift;
		emu->snap_il = (epcm->type == PLAYBACK_EMUVOICE);
		emu->snap_stereo = (epcm->type == PLAYBACK_EMUVOICE) ? (runtime->channels == 2) : 0;
		int channels = (epcm->type == PLAYBACK_EMUVOICE) ? 1 : runtime->channels;
		int subchans = (epcm->type == PLAYBACK_EMUVOICE) ?
			runtime->channels : (1 + emu->das_mode) << emu->emu1010.clock_shift;
		int snap_chan = 0;
#if SNAPSHOT_OFFSET
		int snap_offset = 0;
#endif
		for (int c = 0; c < channels; c++) {
			if (!epcm->voices[c]) {
				dev_err(emu->card->dev, "huh, no voice #%d:0\n", c);
				stop_snapshotter(substream);
				return;
			}
			int voice = epcm->voices[c]->number;
			for (int sc = 0; sc < subchans; sc++) {
#if SNAPSHOT_OFFSET
				if (snap_offset++ < SNAPSHOT_OFFSET)
					continue;
#endif
				if (snap_chan >= SNAPSHOT_CHANS)
					break;
				emu->snap_voices[snap_chan++] = voice + sc;
				snap_info("voice #%d:%d: %d\n", c, sc, voice + sc);
			}
		}
		emu->snap_chans = snap_chan;
#if SNAPSHOT_EXTRA
		if (!emu->das_mode && snap_chan < SNAPSHOT_CHANS_TOTAL) {
			int voice = epcm->extra->number;
			emu->snap_voices[snap_chan++] = voice;
			snap_info("extra voice: %d\n", voice);
		}
#endif
		emu->snap_total = snap_chan;
#if SNAP_IRQ_HANDLER
		emu->num_irq_snaps = 0;
#endif
#if SNAP_CACHE
		emu->num_cache_snaps = 0;
#endif
#if SNAP_REGS
		emu->tried_reg_snaps = 0;
		emu->num_reg_snaps = 0;
		emu->reg_snaps_full = 0;
#endif
	}
	else
		dev_notice(emu->card->dev, "we don't own it\n");
#endif
}

static void setup_snapshotter_post(struct snd_pcm_substream *substream, int period_size)
{
#if SNAP_SETUP
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream) {
		struct snd_pcm_runtime *runtime = substream->runtime;
		struct snd_emu10k1_pcm *epcm = runtime->private_data;
		emu->snap_start = epcm->start_addr;
		emu->snap_period = period_size;
		snap_info("setting up stream, interleaved %d (access %#x), stereo %d, bits %d, raw start %#x\n",
			  emu->snap_il, runtime->access, emu->snap_stereo, emu->snap_width, emu->snap_start);
	}
#endif
}

#if SNAP_REGS
static void snap_regs(struct snd_emu10k1 *emu, int cv, struct emu_reg_snapshot *rs)
{
	rs->voice = cv;
#if SNAP_REGS_FILTER
	rs->z1 = snap_ptr_read(emu, Z1, cv);
	rs->z2 = snap_ptr_read(emu, Z2, cv);
#endif
	rs->ccr = snap_ptr_read(emu, CCR, cv);
	rs->clp = snap_ptr_read(emu, CLP, cv);
	rs->ccca = snap_ptr_read(emu, CCCA, cv);
	rs->ptrx = snap_ptr_read(emu, PTRX, cv);
	rs->cpf = snap_ptr_read(emu, CPF, cv);
#if SNAP_REGS_AMOUNT
	rs->csba = snap_ptr_read(emu, A_CSBA, cv);
#endif
}

static void snapshot_regs_real(struct snd_pcm_substream *substream, const char *lbl, int v)
{
	unsigned long flags;
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	int si = emu->num_reg_snaps;
	struct emu_reg_snapshot *rs = &emu->snap_regs[si];
	int num = v < 0 ? emu->snap_total : 1;

	if (v < 0) {
		num = emu->snap_total;
	} else {
		for (int cv = 0; cv < emu->snap_total; cv++)
			if (emu->snap_voices[cv] == v)
				goto have;
		return;
	have:
		num = 1;
	}
	emu->tried_reg_snaps += num;
	if (emu->reg_snaps_full)
		return;
	si += num;
	if (si > ARRAY_SIZE(emu->snap_regs)) {
		dev_err(emu->card->dev, "too many register snapshorts at '%s'\n", lbl);
		emu->reg_snaps_full = 1;
		return;
	}
	if (!atomic_read(&emu->snapshot_busy) || emu->snapshot_stream != substream)
		return;
	emu->num_reg_snaps = si;
	rs->lbl = lbl;
	spin_lock_irqsave(&emu->emu_lock, flags);
	rs->wc = snap_read(emu, WC);
	if (v < 0) {
		for (v = 0; v < emu->snap_total; v++)
			snap_regs(emu, emu->snap_voices[v], rs++);
	} else {
		snap_regs(emu, v, rs);
	}
	spin_unlock_irqrestore(&emu->emu_lock, flags);
}

#define snapshot_regs_raw(ss, lbl, v) \
	do { \
		snapshot_regs_real(ss, lbl, v); \
		udelay(50); \
		snapshot_regs_real(ss, lbl " +50us", v); \
	} while (0)

#define snapshot_regs_one(ss, lbl, v) \
	snapshot_regs_raw(ss, lbl, v)

#define snapshot_regs_all(ss, lbl) \
	snapshot_regs_raw(ss, lbl, -1)
#else
#define snapshot_regs_one(ss, lbl, v) do {} while (0)
#define snapshot_regs_all(ss, lbl) do {} while (0)
#endif

static void snd_emu10k1_pcm_interrupt(struct snd_emu10k1 *emu,
				      struct snd_emu10k1_voice *voice)
{
	struct snd_emu10k1_pcm *epcm;

	epcm = voice->epcm;
	if (!epcm)
		return;
	if (epcm->substream == NULL)
		return;
#if 0
	dev_dbg(emu->card->dev,
		"IRQ: position = 0x%x, period = 0x%x, size = 0x%x\n",
			epcm->substream->runtime->hw->pointer(emu, epcm->substream),
			snd_pcm_lib_period_bytes(epcm->substream),
			snd_pcm_lib_buffer_bytes(epcm->substream));
#endif
	snd_pcm_period_elapsed(epcm->substream);
}

static void snd_emu10k1_pcm_ac97adc_interrupt(struct snd_emu10k1 *emu,
					      unsigned int status)
{
#if 0
	if (status & IPR_ADCBUFHALFFULL) {
		if (emu->pcm_capture_substream->runtime->mode == SNDRV_PCM_MODE_FRAME)
			return;
	}
#endif
	snd_pcm_period_elapsed(emu->pcm_capture_substream);
}

static void snd_emu10k1_pcm_ac97mic_interrupt(struct snd_emu10k1 *emu,
					      unsigned int status)
{
#if 0
	if (status & IPR_MICBUFHALFFULL) {
		if (emu->pcm_capture_mic_substream->runtime->mode == SNDRV_PCM_MODE_FRAME)
			return;
	}
#endif
	snd_pcm_period_elapsed(emu->pcm_capture_mic_substream);
}

static void snd_emu10k1_pcm_efx_interrupt(struct snd_emu10k1 *emu,
					  unsigned int status)
{
#if 0
	if (status & IPR_EFXBUFHALFFULL) {
		if (emu->pcm_capture_efx_substream->runtime->mode == SNDRV_PCM_MODE_FRAME)
			return;
	}
#endif
	snd_pcm_period_elapsed(emu->pcm_capture_efx_substream);
}	 

static void snd_emu10k1_pcm_free_voices(struct snd_emu10k1_pcm *epcm)
{
	struct snd_emu10k1 *emu = epcm->emu;

	for (unsigned i = 0; i < ARRAY_SIZE(epcm->voices); i++) {
		if (epcm->voices[i]) {
			snd_emu10k1_voice_free(epcm->emu, epcm->voices[i]);
			snap_info("freed voice %d\n", epcm->voices[i]->number);
			epcm->voices[i] = NULL;
		}
	}
}

static int snd_emu10k1_pcm_channel_alloc(struct snd_emu10k1_pcm *epcm,
					 int type, int count, int channels)
{
	struct snd_emu10k1 *emu = epcm->emu;
	int err;

	snd_emu10k1_pcm_free_voices(epcm);

	err = snd_emu10k1_voice_alloc(epcm->emu,
				      type, count, channels,
				      epcm, &epcm->voices[0]);
	if (err < 0)
		return err;
	snap_info("allocated %d voices, #0 = %d\n", channels * count, epcm->voices[0]->number);

	if (epcm->emu->das_mode) {
		epcm->voices[0]->interrupt = snd_emu10k1_pcm_interrupt;
		snap_info("allocated NO extra voice\n");
	} else if (epcm->extra == NULL) {
		// The hardware supports only (half-)loop interrupts, so to support an
		// arbitrary number of periods per buffer, we use an extra voice with a
		// period-sized loop as the interrupt source. Additionally, the interrupt
		// timing of the hardware is "suboptimal" and needs some compensation.
		err = snd_emu10k1_voice_alloc(epcm->emu,
					      type + 1, 1, 1,
					      epcm, &epcm->extra);
		if (err < 0) {
			/*
			dev_dbg(emu->card->dev, "pcm_channel_alloc: "
			       "failed extra: voices=%d, frame=%d\n",
			       voices, frame);
			*/
			snd_emu10k1_pcm_free_voices(epcm);
			return err;
		}
		epcm->extra->interrupt = snd_emu10k1_pcm_interrupt;
		snap_info("allocated extra voice %d\n", epcm->extra->number);
	}

	return 0;
}

// Primes 2-7 and 2^n multiples thereof, up to 16.
static const unsigned int efx_capture_channels[] = {
	1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16
};

static const struct snd_pcm_hw_constraint_list hw_constraints_efx_capture_channels = {
	.count = ARRAY_SIZE(efx_capture_channels),
	.list = efx_capture_channels,
	.mask = 0
};

static const unsigned int capture_buffer_sizes[31] = {
	384,	448,	512,	640,
	384*2,	448*2,	512*2,	640*2,
	384*4,	448*4,	512*4,	640*4,
	384*8,	448*8,	512*8,	640*8,
	384*16,	448*16,	512*16,	640*16,
	384*32,	448*32,	512*32,	640*32,
	384*64,	448*64,	512*64,	640*64,
	384*128,448*128,512*128
};

static const struct snd_pcm_hw_constraint_list hw_constraints_capture_buffer_sizes = {
	.count = 31,
	.list = capture_buffer_sizes,
	.mask = 0
};

static const unsigned int capture_rates[8] = {
	8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000
};

static const struct snd_pcm_hw_constraint_list hw_constraints_capture_rates = {
	.count = 8,
	.list = capture_rates,
	.mask = 0
};

static unsigned int snd_emu10k1_capture_rate_reg(unsigned int rate)
{
	switch (rate) {
	case 8000:	return ADCCR_SAMPLERATE_8;
	case 11025:	return ADCCR_SAMPLERATE_11;
	case 16000:	return ADCCR_SAMPLERATE_16;
	case 22050:	return ADCCR_SAMPLERATE_22;
	case 24000:	return ADCCR_SAMPLERATE_24;
	case 32000:	return ADCCR_SAMPLERATE_32;
	case 44100:	return ADCCR_SAMPLERATE_44;
	case 48000:	return ADCCR_SAMPLERATE_48;
	default:
			snd_BUG();
			return ADCCR_SAMPLERATE_8;
	}
}

static const unsigned int audigy_capture_rates[9] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};

static const struct snd_pcm_hw_constraint_list hw_constraints_audigy_capture_rates = {
	.count = 9,
	.list = audigy_capture_rates,
	.mask = 0
};

static unsigned int snd_emu10k1_audigy_capture_rate_reg(unsigned int rate)
{
	switch (rate) {
	case 8000:	return A_ADCCR_SAMPLERATE_8;
	case 11025:	return A_ADCCR_SAMPLERATE_11;
	case 12000:	return A_ADCCR_SAMPLERATE_12;
	case 16000:	return ADCCR_SAMPLERATE_16;
	case 22050:	return ADCCR_SAMPLERATE_22;
	case 24000:	return ADCCR_SAMPLERATE_24;
	case 32000:	return ADCCR_SAMPLERATE_32;
	case 44100:	return ADCCR_SAMPLERATE_44;
	case 48000:	return ADCCR_SAMPLERATE_48;
	default:
			snd_BUG();
			return A_ADCCR_SAMPLERATE_8;
	}
}

static void snd_emu10k1_constrain_capture_rates(struct snd_emu10k1 *emu,
						struct snd_pcm_runtime *runtime)
{
	if (emu->card_capabilities->emu_model &&
	    emu->emu1010.word_clock == 44100) {
		// This also sets the rate constraint by deleting SNDRV_PCM_RATE_KNOT
		runtime->hw.rates = SNDRV_PCM_RATE_11025 | \
				    SNDRV_PCM_RATE_22050 | \
				    SNDRV_PCM_RATE_44100;
		runtime->hw.rate_min = 11025;
		runtime->hw.rate_max = 44100;
		return;
	}
	snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				   emu->audigy ? &hw_constraints_audigy_capture_rates :
						 &hw_constraints_capture_rates);
}

static void snd_emu1010_constrain_efx_rate(struct snd_emu10k1 *emu,
					   struct snd_pcm_runtime *runtime)
{
	int rate;

	rate = emu->emu1010.word_clock << emu->emu1010.clock_shift;
	runtime->hw.rate_min = runtime->hw.rate_max = rate;
	runtime->hw.rates = snd_pcm_rate_to_rate_bit(rate);
}

static unsigned int emu10k1_calc_pitch_target(unsigned int rate)
{
	unsigned int pitch_target;

	pitch_target = (rate << 8) / 375;
	pitch_target = (pitch_target >> 1) + (pitch_target & 1);
	return pitch_target;
}

#define PITCH_48000 0x00004000
#define PITCH_96000 0x00008000
#define PITCH_85000 0x00007155
#define PITCH_80726 0x00006ba2
#define PITCH_67882 0x00005a82
#define PITCH_57081 0x00004c1c

static unsigned int emu10k1_select_interprom(unsigned int pitch_target)
{
	if (pitch_target == PITCH_48000)
		return CCCA_INTERPROM_0;
	else if (pitch_target < PITCH_48000)
		return CCCA_INTERPROM_1;
	else if (pitch_target >= PITCH_96000)
		return CCCA_INTERPROM_0;
	else if (pitch_target >= PITCH_85000)
		return CCCA_INTERPROM_6;
	else if (pitch_target >= PITCH_80726)
		return CCCA_INTERPROM_5;
	else if (pitch_target >= PITCH_67882)
		return CCCA_INTERPROM_4;
	else if (pitch_target >= PITCH_57081)
		return CCCA_INTERPROM_3;
	else  
		return CCCA_INTERPROM_2;
}

static u16 emu10k1_send_target_from_amount(u8 amount)
{
	static const u8 shifts[8] = { 4, 4, 5, 6, 7, 8, 9, 10 };
	static const u16 offsets[8] = { 0, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000 };
	u8 exp;

	if (amount == 0xff)
		return 0xffff;
	exp = amount >> 5;
	return ((amount & 0x1f) << shifts[exp]) + offsets[exp];
}

#if SNAP_INTERPOLATOR
static void /*u32*/ set_cache(struct snd_emu10k1 *emu, int voice, int ss, int sample, int value)
{
	int a = sample * ss;
	int o = a / 4;
	int rm = 0xffffffffU >> (32 - 8 * ss);
	int m = rm << ((a & 3) * 8);
	u32 cv = snd_emu10k1_ptr_read(emu, CD0 + o, voice);
	//u32 ov = (cv & m) >> ((a & 3) * 8);
	cv &= ~m;
	cv |= value << ((a & 3) * 8);
	snd_emu10k1_ptr_write(emu, CD0 + o, voice, cv);
	//return ov;
}

static void snd_emu10k1_test_interprom(struct snd_emu10k1 *emu,
				       struct snd_emu10k1_voice *evoice)
{
	struct snd_pcm_substream *substream = evoice->epcm->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct emu_inter_snapshot *is;
	int voice, stereo, w_16;
	int ss;
	int nums = 0;

	voice = evoice->number;
	stereo = runtime->channels == 2;
	w_16 = snd_pcm_format_width(runtime->format) == 16;

	if (stereo) {
		snd_emu10k1_ptr_write(emu, CPF, voice, CPF_STEREO_MASK | 0x2000);
	} else {
		snd_emu10k1_ptr_write(emu, CPF, voice, 0x2000);
	}

	snd_emu10k1_ptr_write_multiple(emu, voice,
		A_FXRT1, 0x03020100,
		A_FXRT2, 0x07060504,
		A_SENDAMOUNTS, 0,

		PTRX, 0xff00,
		A_CSBA, 0xffff,

		// Disable filter (in conjunction with CCCA_RESONANCE == 0)
		VTFT, 0x80000000 | VTFT_FILTERTARGET_MASK,
		CVCF, 0x80000000 | CVCF_CURRENTFILTER_MASK,

		CCR, 0,

		REGLIST_END);

	for (int i = 0; i < 32; i++)
		snd_emu10k1_ptr_write(emu, CD0 + i, voice, w_16 ? 0 : 0x80808080);

	ss = (w_16 + 1) * (stereo + 1);
	for (int r = 0; r < 8; r++) {
		snd_emu10k1_ptr_write(emu, CCCA, voice,
			      CCCA_INTERPROM_1 * r |
			      (w_16 ? 0 : CCCA_8BITSELECT));
		for (int s = 0; s < 8; s++) {
			int eq = 0;
			set_cache(emu, voice, ss, s, w_16 ? 100 * 256 : 100 + 0x80);
			for (;;) {
				unsigned long flags;
				if (nums == ARRAY_SIZE(emu->snap_inter))
					break;
				is = &emu->snap_inter[nums];
				is->r = r;
				is->s = s;
				spin_lock_irqsave(&emu->emu_lock, flags);
				is->wc = snap_read(emu, WC);
				is->z1 = snap_ptr_read(emu, Z1, voice);
				is->z2 = snap_ptr_read(emu, Z2, voice);
				is->fx = snap_ptr_read(emu, A_FXGPREGBASE + emu->fxgprs, 0);
				spin_unlock_irqrestore(&emu->emu_lock, flags);
				if (nums && is[-1].s == is->s &&
				    is[-1].z1 == is->z1 && is[-1].z2 == is->z2 && is[-1].fx == is->fx &&
					      (is[-1].wc & WC_SAMPLECOUNTER_MASK) != (is->wc & WC_SAMPLECOUNTER_MASK)) {
					eq++;
					if (eq == 3)
						break;
					continue;
				}
				eq = 0;
				nums++;
			}
			set_cache(emu, voice, ss, s, w_16 ? 0 : 0x80);
		}
	}
	emu->num_inter_snaps = nums;
}
#endif

static void snd_emu10k1_pcm_init_voice(struct snd_emu10k1 *emu,
				       struct snd_emu10k1_voice *evoice,
				       bool w_16, bool stereo,
				       unsigned int start_addr,
				       unsigned int end_addr,
				       const unsigned char *send_routing,
				       const unsigned char *send_amount)
{
#if SNAP_REGS
	struct snd_pcm_substream *substream = evoice->epcm->substream;
#endif
	unsigned int silent_page;
	int voice;

	voice = evoice->number;

	snap_info("initializing voice %d, stereo %d, 16-bit %d, start %#x, end %#x\n",
		  voice, stereo, w_16, start_addr, end_addr);


#if PROC_CTL_AMOUNTS
	// FIXME: this is obviously broken after "pass raw FX send config to snd_emu10k1_pcm_init_voice()"
	if (emu->init_send_amount != -1) {
		send_amount[0] = emu->init_send_amount;
		memset(send_amount + 1, 0, 7);
	}
#endif

	silent_page = ((unsigned int)emu->silent_page.addr << emu->address_mode) |
		      (emu->address_mode ? MAP_PTI_MASK1 : MAP_PTI_MASK0);
	snapshot_regs_one(substream, "before voice init", voice);
	snd_emu10k1_ptr_write_multiple(emu, voice,
		// Not really necessary for the slave, but it doesn't hurt
		CPF, stereo ? CPF_STEREO_MASK : 0,
#if SNAP_REGS
		REGLIST_END);
	snapshot_regs_one(substream, "after cpf init", voice);
	snd_emu10k1_ptr_write_multiple(emu, voice,
#endif
		// Assumption that PT is already 0 so no harm overwriting
		PTRX, (send_amount[0] << 8) | send_amount[1],
		// Stereo slaves don't need to have the addresses set, but it doesn't hurt
		DSL, end_addr | (send_amount[3] << 24),
		PSST, start_addr | (send_amount[2] << 24),
#if SNAP_REGS
		REGLIST_END);
	snapshot_regs_one(substream, "after ptr init", voice);
	snd_emu10k1_ptr_write_multiple(emu, voice,
#endif
		CCCA, emu10k1_select_interprom(evoice->epcm->pitch_target) |
		      (w_16 ? 0 : CCCA_8BITSELECT),
#if SNAP_REGS
		REGLIST_END);
	snapshot_regs_one(substream, "after ccca init", voice);
	snd_emu10k1_ptr_write_multiple(emu, voice,
#endif
		// Clear filter delay memory
		// pointless? a new sample is fetched from the cache and progresses through
		// the filter at each word clock cycle even if the current pitch is zero.
		// maybe if the voice is started really quickly after this?
		Z1, 0,
		Z2, 0,
#if SNAP_REGS_FILTER
		REGLIST_END);
	snapshot_regs_one(substream, "after z init", voice);
	snd_emu10k1_ptr_write_multiple(emu, voice,
#endif
		// Invalidate maps
		MAPA, silent_page,
		MAPB, silent_page,
#if 0  // No obvious effect of anything snapshotted
		REGLIST_END);
	snapshot_regs_one(substream, "after map init", voice);
	snd_emu10k1_ptr_write_multiple(emu, voice,
#endif
		// Disable filter (in conjunction with CCCA_RESONANCE == 0)
		VTFT, VTFT_FILTERTARGET_MASK,
		CVCF, CVCF_CURRENTFILTER_MASK,
		REGLIST_END);
#if 0  // We don't actually capture these
	snapshot_regs_one(substream, "after VTFT & CVCF init", voice);
#endif
	// Setup routing
	if (emu->audigy) {
		snd_emu10k1_ptr_write_multiple(emu, voice,
			A_FXRT1, snd_emu10k1_compose_audigy_fxrt1(send_routing),
			A_FXRT2, snd_emu10k1_compose_audigy_fxrt2(send_routing),
			A_SENDAMOUNTS, snd_emu10k1_compose_audigy_sendamounts(send_amount),
			REGLIST_END);
#if 0  // We don't actually capture these
		snapshot_regs_one(substream, "after routing init", voice);
#endif
		for (int i = 0; i < 4; i++) {
			u32 aml = emu10k1_send_target_from_amount(send_amount[2 * i]);
			u32 amh = emu10k1_send_target_from_amount(send_amount[2 * i + 1]);
			snd_emu10k1_ptr_write(emu, A_CSBA + i, voice, (amh << 16) | aml);
		}
#if SNAP_REGS_AMOUNT
		snapshot_regs_one(substream, "after cs init", voice);
#endif
	} else {
		snd_emu10k1_ptr_write(emu, FXRT, voice,
				      snd_emu10k1_compose_send_routing(send_routing));
#if 0  // We don't actually capture these
		snapshot_regs_one(substream, "after routing init", voice);
#endif
	}

	emu->voices[voice].dirty = 1;
}

static void snd_emu10k1_pcm_init_voices(struct snd_emu10k1 *emu,
					struct snd_emu10k1_voice *evoice,
					bool w_16, bool stereo,
					unsigned int start_addr,
					unsigned int end_addr,
					struct snd_emu10k1_pcm_mixer *mix)
{
	spin_lock_irq(&emu->reg_lock);
	snd_emu10k1_pcm_init_voice(emu, evoice, w_16, stereo,
				   start_addr, end_addr,
				   &mix->send_routing[stereo][0],
				   &mix->send_volume[stereo][0]);
	if (stereo)
		snd_emu10k1_pcm_init_voice(emu, evoice + 1, w_16, true,
					   start_addr, end_addr,
					   &mix->send_routing[2][0],
					   &mix->send_volume[2][0]);
	spin_unlock_irq(&emu->reg_lock);
}

static void snd_emu10k1_pcm_init_das_voices(struct snd_emu10k1 *emu,
					    struct snd_emu10k1_voice *evoice,
					    unsigned int start_addr,
					    unsigned int end_addr,
					    unsigned char channel)
{
	static const unsigned char send_amount[8] = { 255, 0, 0, 0, 0, 0, 0, 0 };
	unsigned char send_routing[9];

	for (int i = 0; i < ARRAY_SIZE(send_routing); i++)
		send_routing[i] = (channel + i) % NUM_G;
	snd_emu10k1_pcm_init_voice(emu, evoice, true, true,
				   start_addr, end_addr,
				   send_routing, send_amount);
	snd_emu10k1_pcm_init_voice(emu, evoice + 1, true, true,
				   start_addr, end_addr,
				   send_routing + 1, send_amount);
}

static void snd_emu10k1_pcm_init_extra_voice(struct snd_emu10k1 *emu,
					     struct snd_emu10k1_voice *evoice,
					     bool w_16,
					     unsigned int start_addr,
					     unsigned int end_addr)
{
	static const unsigned char send_routing[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	static const unsigned char send_amount[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	snd_emu10k1_pcm_init_voice(emu, evoice, w_16, false,
				   start_addr, end_addr,
				   send_routing, send_amount);
}

static void emu_murder_cache(struct snd_emu10k1 *emu, int stereo, struct snd_pcm_substream *substream)
{
#if SNAP_CACHE
	if (!atomic_read(&emu->snapshot_busy) || emu->snapshot_stream != substream)
		return;

	for (int c = 0; c < emu->snap_total; c++) {
		int voice = emu->snap_voices[c];
		//int j = i * 4;
		//if (stereo && !(c & 1)) j += 0x80;
		int sample = (stereo && (c & 1)) ? 0x0defaced : 0xdeadbeef;
		for (int i = 0; i < 32; i++) {
			//snd_emu10k1_ptr_write(emu, CD0 + i, voice, j + ((j + 1) << 8) + ((j + 2) << 16) + ((j + 3) << 24));
			snd_emu10k1_ptr_write(emu, CD0 + i, voice, sample);
		}
	}
#endif
}

static void emu_snap_cache(struct snd_emu10k1 *emu, struct snd_pcm_substream *substream)
{
#if SNAP_CACHE
	if (!atomic_read(&emu->snapshot_busy) || emu->snapshot_stream != substream)
		return;

	udelay(1000);
	unsigned long flags;
	spin_lock_irqsave(&emu->emu_lock, flags);
	for (int c = 0; c < emu->snap_total; c++) {
		int si = emu->num_cache_snaps;
		if (si == ARRAY_SIZE(emu->snap_cache)) {
			dev_err(emu->card->dev, "too many cache snapshots\n");
			break;
		}
		emu->num_cache_snaps++;
		struct emu_cache_snapshot *cs = &emu->snap_cache[si];
		int voice = emu->snap_voices[c];
		cs->voice = voice;
		cs->ccca = snap_ptr_read(emu, CCCA, voice);
		cs->ccr = snap_ptr_read(emu, CCR, voice);
		cs->clp = snap_ptr_read(emu, CLP, voice);
		for (int i = 0; i < 32; i++)
			cs->cd[i] = snap_ptr_read(emu, CD0 + i, voice);
		//ss->csba = snap_ptr_read(emu, A_CSBA, voice);
		//ss->reg0e = snap_ptr_read(emu, 0x0e, voice);
		//ss->reg0f = snap_ptr_read(emu, 0x0f, voice);
		//ss->reg1e = snap_ptr_read(emu, 0x1e, voice);
		//ss->reg1f = snap_ptr_read(emu, 0x1f, voice);
		//ss->reg57 = snap_ptr_read(emu, 0x57, voice);
		//ss->reg7f = snap_ptr_read(emu, 0x7f, voice);
	}
	spin_unlock_irqrestore(&emu->emu_lock, flags);
#endif
}

static int snd_emu10k1_playback_hw_params(struct snd_pcm_substream *substream,
					  struct snd_pcm_hw_params *hw_params)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	size_t alloc_size;
	int type, channels, count;
	int err;

	log_strm_ctl_event(substream, 7);

	if (epcm->type == PLAYBACK_EMUVOICE) {
		type = EMU10K1_PCM;
		channels = 1;
		count = params_channels(hw_params);
	} else {
		type = EMU10K1_EFX;
		channels = params_channels(hw_params);
		count = (1 + emu->das_mode) << emu->emu1010.clock_shift;
	}
	err = snd_emu10k1_pcm_channel_alloc(epcm, type, count, channels);
	if (err < 0)
		return err;

	alloc_size = params_buffer_bytes(hw_params);
	if (emu->iommu_workaround)
		alloc_size += EMUPAGESIZE;
	err = snd_pcm_lib_malloc_pages(substream, alloc_size);
	if (err < 0)
		return err;
	if (emu->iommu_workaround && runtime->dma_bytes >= EMUPAGESIZE)
		runtime->dma_bytes -= EMUPAGESIZE;
	if (err > 0) {	/* change */
		int mapped;
		if (epcm->memblk != NULL)
			snd_emu10k1_free_pages(emu, epcm->memblk);
		epcm->memblk = snd_emu10k1_alloc_pages(emu, substream);
		epcm->start_addr = 0;
		if (! epcm->memblk)
			return -ENOMEM;
		mapped = ((struct snd_emu10k1_memblk *)epcm->memblk)->mapped_page;
		if (mapped < 0)
			return -ENOMEM;
		epcm->start_addr = mapped << PAGE_SHIFT;
#if SNAP_INPUT
		if (emu->snapshot_stream == substream) {
			if (emu->snap_buffer)
				kfree(emu->snap_buffer);
			emu->snap_buffer = kmalloc(runtime->dma_bytes, GFP_KERNEL);
		}
#endif
	}

	log_strm_ctl_event(substream, 10);

	return 0;
}

static int snd_emu10k1_playback_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm;

	log_strm_ctl_event(substream, -7);

	if (runtime->private_data == NULL)
		return 0;
	epcm = runtime->private_data;
	if (epcm->extra) {
		snd_emu10k1_voice_free(epcm->emu, epcm->extra);
		snap_info("freed extra voice %d\n", epcm->extra->number);
		epcm->extra = NULL;
	}
	snd_emu10k1_pcm_free_voices(epcm);
	if (epcm->memblk) {
		snd_emu10k1_free_pages(emu, epcm->memblk);
		epcm->memblk = NULL;
		epcm->start_addr = 0;
	}
	snd_pcm_lib_free_pages(substream);

	log_strm_ctl_event(substream, -10);

	return 0;
}

static int snd_emu10k1_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	bool w_16 = snd_pcm_format_width(runtime->format) == 16;
	bool stereo = runtime->channels == 2;
	unsigned int start_addr, end_addr;
	unsigned int rate;

#if SNAP_INTERPOLATOR
	snd_emu10k1_test_interprom(emu, epcm->voices[0]);
#endif
	setup_snapshotter(substream);

	rate = runtime->rate;
	if (emu->card_capabilities->emu_model &&
	    emu->emu1010.word_clock == 44100)
		rate = rate * 480 / 441;
	epcm->pitch_target = emu10k1_calc_pitch_target(rate);

	start_addr = epcm->start_addr >> w_16;
	end_addr = start_addr + runtime->period_size;
	snd_emu10k1_pcm_init_extra_voice(emu, epcm->extra, w_16,
					 start_addr, end_addr);
	start_addr >>= stereo;
	epcm->ccca_start_addr = start_addr;
	end_addr = start_addr + runtime->buffer_size;

#if PROC_CTL_LOOP
	if (emu->init_loop_end != -1)
		end_addr = start_addr + emu->init_loop_end;
	if (emu->init_loop_start != -1)
		start_addr += emu->init_loop_start;
#endif


	snd_emu10k1_pcm_init_voices(emu, epcm->voices[0], w_16, stereo,
				    start_addr, end_addr,
				    &emu->pcm_mixer[substream->number]);

	setup_snapshotter_post(substream, runtime->period_size);

	return 0;
}

static int snd_emu10k1_efx_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	bool das_mode = emu->das_mode;
	unsigned int shift = emu->emu1010.clock_shift;
	unsigned int start_addr;
	unsigned int extra_size, channel_size;
	unsigned int i, j;

	setup_snapshotter(substream);

	log_strm_ctl_event(substream, 17);

	epcm->pitch_target = PITCH_48000;

	start_addr = epcm->start_addr >> 1;  // 16-bit voices
	channel_size = runtime->buffer_size >> shift;

	if (das_mode) {
		unsigned count = 1 << shift;
		start_addr >>= 1;
		epcm->ccca_start_addr = start_addr;
#if LOG_PB_POS || LOG_PB_FILL
		if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream) {
			emu->snap_pb_base = start_addr;
			emu->snap_pb_size = channel_size;
		}
#endif
		for (i = 0; i < runtime->channels; i++) {
			for (j = 0; j < count; j++) {
				snd_emu10k1_pcm_init_das_voices(emu, epcm->voices[i] + j * 2,
								start_addr, start_addr + channel_size,
								(i * count + j) * 2);
				start_addr += channel_size;
			}
		}
	} else {
		extra_size = runtime->period_size >> shift;
		snd_emu10k1_pcm_init_extra_voice(emu, epcm->extra, true,
						 start_addr, start_addr + extra_size);

		epcm->ccca_start_addr = start_addr;
#if LOG_PB_POS || LOG_PB_FILL
		if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream) {
			emu->snap_pb_base = start_addr;
			emu->snap_pb_size = channel_size;
		}
#endif
		for (i = 0; i < runtime->channels; i++) {
			unsigned int xstart_addr = start_addr;
			unsigned int xend_addr = start_addr + channel_size;
#if PROC_CTL_LOOP
			if (emu->init_loop_end != -1)
				xend_addr = xstart_addr + emu->init_loop_end;
			if (emu->init_loop_start != -1)
				xstart_addr += emu->init_loop_start;
#endif
			snd_emu10k1_pcm_init_voices(emu, epcm->voices[i], true, false,
						    xstart_addr, xend_addr,
						    &emu->efx_pcm_mixer[i]);
			start_addr += channel_size;
		}
	}

	//runtime->silence_size = runtime->silence_threshold = runtime->period_size + (64 << shift);
	//runtime->silence_size = ULONG_MAX;

	log_strm_ctl_event(substream, 20);

	setup_snapshotter_post(substream, runtime->period_size >> shift);

	snap_info("efx pb prep, %d periods a %#lx, buffer %#lx, bits %d, frame bits %d\n",
		  runtime->periods, runtime->period_size, runtime->buffer_size,
		  runtime->sample_bits, runtime->frame_bits);

	return 0;
}

static const struct snd_pcm_hardware snd_emu10k1_efx_playback =
{
	.info =			(SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_NONINTERLEAVED |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_PAUSE),
	.formats =		SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_48000,
	.rate_min =		48000,
	.rate_max =		48000,
	.channels_min =		1,
	.channels_max =		NUM_EFX_PLAYBACK,
	.buffer_bytes_max =	(128*1024),
	.period_bytes_max =	(128*1024),
	.periods_min =		2,
	.periods_max =		2,
	.fifo_size =		0,
};

static int snd_emu10k1_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	int idx;

	/* zeroing the buffer size will stop capture */
	snd_emu10k1_ptr_write(emu, epcm->capture_bs_reg, 0, 0);
	switch (epcm->type) {
	case CAPTURE_AC97ADC:
		snd_emu10k1_ptr_write(emu, ADCCR, 0, 0);
		break;
	case CAPTURE_EFX:
		if (emu->card_capabilities->emu_model) {
			unsigned mask = 0xffffffff >> (32 - runtime->channels * 2);
			if (emu->das_mode) {
				unsigned shift = emu->emu1010.clock_shift;
				if (shift) {
#if MANIPULATE_FX
					if (1) {
#else
					if (emu->card_capabilities->emu_in_32) {
#endif
						if (shift == 2)
							mask |= mask << 16;
						epcm->capture_cr_val2 = mask;
					} else {
						if (shift == 2)
							mask |= mask << 8;
						mask |= mask << 16;
						epcm->capture_cr_val2 = 0;
					}
				} else {
					epcm->capture_cr_val2 = 0;
				}
				epcm->capture_cr_val = mask;
			} else {
				// The upper 32 16-bit capture voices, two for each of the 16 32-bit channels.
				// The lower voices are occupied by A_EXTOUT_*_CAP*.
				epcm->capture_cr_val = 0;
				epcm->capture_cr_val2 = mask;
			}
		}
		if (emu->audigy) {
			snd_emu10k1_ptr_write_multiple(emu, 0,
				A_FXWC1, 0,
				A_FXWC2, 0,
				REGLIST_END);
		} else
			snd_emu10k1_ptr_write(emu, FXWC, 0, 0);
		break;
	default:
		break;
	}	
	snd_emu10k1_ptr_write(emu, epcm->capture_ba_reg, 0, runtime->dma_addr);
	epcm->capture_bufsize = snd_pcm_lib_buffer_bytes(substream);
#if LOG_CAP_POS
	emu->snap_cap_size = epcm->capture_bufsize;
#endif
	epcm->capture_bs_val = 0;
	for (idx = 0; idx < 31; idx++) {
		if (capture_buffer_sizes[idx] == epcm->capture_bufsize) {
			epcm->capture_bs_val = idx + 1;
			break;
		}
	}
	if (epcm->capture_bs_val == 0) {
		snd_BUG();
		epcm->capture_bs_val++;
	}
	snap_info("capturing %u channels, buffer size %u, bs reg 0x%x\n",
		  runtime->channels, epcm->capture_bufsize, epcm->capture_bs_val);
	if (epcm->type == CAPTURE_AC97ADC) {
		unsigned rate = runtime->rate;
		if (!(runtime->hw.rates & SNDRV_PCM_RATE_48000))
			rate = rate * 480 / 441;

		epcm->capture_cr_val = emu->audigy ? A_ADCCR_LCHANENABLE : ADCCR_LCHANENABLE;
		if (runtime->channels > 1)
			epcm->capture_cr_val |= emu->audigy ? A_ADCCR_RCHANENABLE : ADCCR_RCHANENABLE;
		epcm->capture_cr_val |= emu->audigy ?
			snd_emu10k1_audigy_capture_rate_reg(rate) :
			snd_emu10k1_capture_rate_reg(rate);
	}
	return 0;
}

static void snd_emu10k1_playback_fill_cache(struct snd_emu10k1 *emu,
					    unsigned voice,
					    u32 sample, bool stereo,
					    unsigned loop_start)
{
	u32 ccr;

	// We assume that the cache is resting at this point (i.e.,
	// CCR_CACHEINVALIDSIZE is very small).

#if PROC_CTL_CACHE
	if (emu->init_cache_inval == -1 && emu->init_cache_loop_inval != -1)
#endif
	// Clear leading frames. For simplicitly, this does too much,
	// except for 16-bit stereo. And the interpolator will actually
	// access them at all only when we're pitch-shifting.
	for (int i = 0; i < 3; i++)
		snd_emu10k1_ptr_write(emu, CD0 + i, voice, sample);

#if PROC_CTL_CACHE
	u32 cis = emu->init_cache_inval != -1 ? emu->init_cache_inval : (64 - 3);
	u32 lis = emu->init_cache_loop_inval != -1 ? emu->init_cache_loop_inval : 0;
	u32 cra = emu->init_cache_read_addr != -1 ? emu->init_cache_read_addr : 0;
	u32 cla = emu->init_cache_loop_addr != -1 ? emu->init_cache_loop_addr + loop_start : 0;
	u32 clp = REG_VAL_PUT(CLP_CACHELOOPADDR, cla & 0xffff);
	if (stereo)
		snd_emu10k1_ptr_write(emu, CLP, voice + 1, clp);
	snd_emu10k1_ptr_write(emu, CLP, voice, clp);
	ccr = REG_VAL_PUT(CCR_CACHEINVALIDSIZE, cis) |
		REG_VAL_PUT(CCR_LOOPINVALSIZE, lis) |
		REG_VAL_PUT(CCR_CACHELOOPADDRHI, cla >> 16) |
		REG_VAL_PUT(CCR_READADDRESS, cra) |
		(emu->init_cache_loop_flag > 0 ? CCR_CACHELOOPFLAG : 0) |
		(emu->init_loop_flag > 0 ? CCR_LOOPFLAG : 0);
#else
	// Fill cache
	ccr = (64 - 3) << REG_SHIFT(CCR_CACHEINVALIDSIZE);
#endif
	if (stereo) {
		// The engine goes haywire if CCR_READADDRESS is out of sync
		snd_emu10k1_ptr_write(emu, CCR, voice + 1, ccr);
	}
	snd_emu10k1_ptr_write(emu, CCR, voice, ccr);
}

static void snd_emu10k1_playback_prepare_voices(struct snd_emu10k1 *emu,
						struct snd_emu10k1_pcm *epcm,
						bool w_16, bool stereo,
						int shift, int channels)
{
	struct snd_pcm_substream *substream = epcm->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned eloop_start = epcm->start_addr >> w_16;
	unsigned loop_start = eloop_start >> stereo;
	unsigned eloop_size = runtime->period_size >> shift;
	unsigned loop_size = runtime->buffer_size >> shift;
	u32 sample = w_16 ? 0 : 0x80808080;
	int count = 1 << shift;

	emu_murder_cache(emu, stereo, substream);
	snapshot_regs_all(substream, "before set address");

	// To make the playback actually start at the 1st frame,
	// we need to compensate for two circumstances:
	// - The actual position is delayed by the cache size (64 frames)
	// - The interpolator is centered around the 4th frame
	unsigned int xloop_size = loop_size;
#if PROC_CTL_LOOP
	if (emu->init_loop_end != -1 && emu->init_loop_end < 64) {
		if (emu->init_loop_start != -1)
			xloop_size = emu->init_loop_end - emu->init_loop_start;
		else
			xloop_size = emu->init_loop_end;
	}
#endif
	unsigned int oloop_start = loop_start;
#if PROC_CTL_CACHE
	if (emu->init_read_addr != -1)
		loop_start += emu->init_read_addr;
	else
#endif
		loop_start += (epcm->resume_pos + 64 - 3) % xloop_size;
	for (int i = 0; i < channels; i++) {
		unsigned voice = epcm->voices[i]->number;
		for (int j = 0; j < count; j++, voice += 2) {
			snd_emu10k1_ptr_write(emu, CCCA_CURRADDR, voice, loop_start);
			//if (stereo)
			//	snd_emu10k1_ptr_write(emu, CCCA_CURRADDR, voice + 1, loop_start);
			snapshot_regs_one(epcm->substream, "before fill", voice);
			snd_emu10k1_playback_fill_cache(emu, voice, sample, stereo, oloop_start);
			snapshot_regs_one(epcm->substream, "after fill", voice);
			loop_start += loop_size;
		}
	}

	// The interrupt is triggered when CCCA_CURRADDR (CA) wraps around,
	// which is ahead of the actual playback position, so the interrupt
	// source needs to be delayed.
	//
	// In principle, this wouldn't need to be the cache's entire size - in
	// practice, CCR_CACHEINVALIDSIZE (CIS) > `fetch threshold` has never
	// been observed, and assuming 40 _bytes_ should be safe.
	//
	// The cache fills are somewhat random, which makes it impossible to
	// align them with the interrupts. This makes a non-delayed interrupt
	// source not practical, as the interrupt handler would have to wait
	// for (CA - CIS) >= period_boundary for every channel in the stream.
	//
	// This is why all other (open) drivers for these chips use timer-based
	// interrupts.
	//
	// But then, Audigy introduced delayed interrupt functionality, so we
	// use it in D.A.S. mode, where we may need all voices. We can't use
	// it in regular mode due to supporting multiple substreams, which the
	// delayed IRQs cannot really handle; also, we prefer supporting an
	// arbitrary number of periods there, for which we need the extra voice.
	//
	if (!emu->das_mode) {
		eloop_start += (epcm->resume_pos + eloop_size - 3) % eloop_size;
		snd_emu10k1_ptr_write(emu, CCCA_CURRADDR, epcm->extra->number, eloop_start);
	}

	snapshot_regs_all(substream, "after set address");
	emu_snap_cache(emu, substream);

	// It takes a moment until the cache fills complete,
	// but the unmuting takes long enough for that.
}

static void snd_emu10k1_playback_commit_volume(struct snd_emu10k1 *emu,
					       struct snd_emu10k1_voice *evoice,
					       unsigned int vattn)
{
	snd_emu10k1_ptr_write_multiple(emu, evoice->number,
		VTFT, vattn | VTFT_FILTERTARGET_MASK,
		CVCF, vattn | CVCF_CURRENTFILTER_MASK,
		REGLIST_END);
}

static void snd_emu10k1_playback_unmute_voice(struct snd_emu10k1 *emu,
					      struct snd_emu10k1_voice *evoice,
					      bool stereo, bool master,
					      struct snd_emu10k1_pcm_mixer *mix)
{
	unsigned int vattn;
	unsigned int tmp;

	tmp = stereo ? (master ? 1 : 2) : 0;
	vattn = mix->attn[tmp] << 16;
	snd_emu10k1_playback_commit_volume(emu, evoice, vattn);
}	

static void snd_emu10k1_playback_unmute_voices(struct snd_emu10k1 *emu,
					       struct snd_emu10k1_voice *evoice,
					       bool stereo,
					       struct snd_emu10k1_pcm_mixer *mix)
{
	snd_emu10k1_playback_unmute_voice(emu, evoice, stereo, true, mix);
	if (stereo)
		snd_emu10k1_playback_unmute_voice(emu, evoice + 1, true, false, mix);
}

static void snd_emu10k1_playback_unmute_das_voices(struct snd_emu10k1 *emu,
						   struct snd_emu10k1_voice *evoice)
{
	snd_emu10k1_playback_commit_volume(emu, evoice, 0x8000 << 16);
	snd_emu10k1_playback_commit_volume(emu, evoice + 1, 0x8000 << 16);
}

static void snd_emu10k1_playback_mute_voice(struct snd_emu10k1 *emu,
					    struct snd_emu10k1_voice *evoice)
{
	snd_emu10k1_playback_commit_volume(emu, evoice, 0);
}

static void snd_emu10k1_playback_mute_voices(struct snd_emu10k1 *emu,
					     struct snd_emu10k1_voice *evoice,
					     bool stereo)
{
	snd_emu10k1_playback_mute_voice(emu, evoice);
	if (stereo)
		snd_emu10k1_playback_mute_voice(emu, evoice + 1);
}

static void snd_emu10k1_playback_commit_pitch(struct snd_emu10k1 *emu,
					      u32 voice, u32 pitch_target)
{
	u32 ptrx = snd_emu10k1_ptr_read(emu, PTRX, voice);
	u32 cpf = snd_emu10k1_ptr_read(emu, CPF, voice);
	snd_emu10k1_ptr_write_multiple(emu, voice,
		PTRX, (ptrx & ~PTRX_PITCHTARGET_MASK) | pitch_target,
		CPF, (cpf & ~(CPF_CURRENTPITCH_MASK | CPF_FRACADDRESS_MASK)) | pitch_target,
		REGLIST_END);
}

static void snd_emu10k1_playback_trigger_voice(struct snd_emu10k1 *emu,
					       struct snd_emu10k1_voice *evoice)
{
	unsigned int voice;

	voice = evoice->number;
	snd_emu10k1_playback_commit_pitch(emu, voice, evoice->epcm->pitch_target << 16);
}

static void snd_emu10k1_playback_stop_voice(struct snd_emu10k1 *emu,
					    struct snd_emu10k1_voice *evoice)
{
	unsigned int voice;

	voice = evoice->number;
	snd_emu10k1_playback_commit_pitch(emu, voice, 0);
}

static void snap_init_pre(struct snd_pcm_substream *substream, struct snd_emu10k1 *emu)
{
#if SNAP_INIT
	if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream) {
		unsigned long flags;
		spin_lock_irqsave(&emu->emu_lock, flags);
		for (int c = 0; c < emu->snap_total; c++) {
			struct emu_init_snapshot *sc = &emu->snap_chan[c];
			int cv = emu->snap_voices[c];
			sc->cpf = snap_ptr_read(emu, CPF, cv);
			sc->ptrx = snap_ptr_read(emu, PTRX, cv);
			sc->psst = snap_ptr_read(emu, PSST, cv);
			sc->dsl = snap_ptr_read(emu, DSL, cv);
			sc->ccca = snap_ptr_read(emu, CCCA, cv);
			sc->ccr = snap_ptr_read(emu, CCR, cv);
		}
		spin_unlock_irqrestore(&emu->emu_lock, flags);
	}
#endif
}

static void snap_init_post(struct snd_pcm_substream *substream, struct snd_emu10k1 *emu)
{
#if SNAP_INIT && SNAP_INIT_VOL
	if (atomic_read(&emu->snapshot_busy) && emu->snapshot_stream == substream) {
		unsigned long flags;
		spin_lock_irqsave(&emu->emu_lock, flags);
		for (int c = 0; c < emu->snap_total; c++) {
			struct emu_init_snapshot *sc = &emu->snap_chan[c];
			int cv = emu->snap_voices[c];
			sc->cvcf = snap_ptr_read(emu, CVCF, cv);
			sc->csba = snap_ptr_read(emu, A_CSBA, cv);
			sc->cshg = snap_ptr_read(emu, A_CSDC, cv);
			sc->csfe = snap_ptr_read(emu, A_CSFE, cv);
			sc->csdc = snap_ptr_read(emu, A_CSHG, cv);
		}
		spin_unlock_irqrestore(&emu->emu_lock, flags);
	}
#endif
}

static void snd_emu10k1_playback_set_running(struct snd_emu10k1 *emu,
					     struct snd_emu10k1_pcm *epcm)
{
	epcm->running = 1;
	if (emu->das_mode) {
		unsigned int voice = epcm->voices[0]->number;
		snd_emu10k1_voice_half_loop_intr_enable(emu, voice);
		snd_emu10k1_voice_intr_enable(emu, voice);
	} else {
		snd_emu10k1_voice_intr_enable(emu, epcm->extra->number);
	}
}

static void snd_emu10k1_playback_set_stopped(struct snd_emu10k1 *emu,
					      struct snd_emu10k1_pcm *epcm)
{
	if (emu->das_mode) {
		unsigned int voice = epcm->voices[0]->number;
		snd_emu10k1_voice_half_loop_intr_disable(emu, voice);
		snd_emu10k1_voice_intr_disable(emu, voice);
	} else {
		snd_emu10k1_voice_intr_disable(emu, epcm->extra->number);
	}
	epcm->running = 0;
}

static int snd_emu10k1_playback_trigger(struct snd_pcm_substream *substream,
				        int cmd)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	struct snd_emu10k1_pcm_mixer *mix;
	bool w_16 = snd_pcm_format_width(runtime->format) == 16;
	bool stereo = runtime->channels == 2;
	int result = 0;

	/*
	dev_dbg(emu->card->dev,
		"trigger - emu10k1 = 0x%x, cmd = %i, pointer = %i\n",
	       (int)emu, cmd, substream->ops->pointer(substream))
	*/
	spin_lock(&emu->reg_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		snd_emu10k1_playback_prepare_voices(emu, epcm, w_16, stereo, 0, 1);
		fallthrough;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		snapshot_regs_all(substream, "before unmute");
		mix = &emu->pcm_mixer[substream->number];
		snd_emu10k1_playback_unmute_voices(emu, epcm->voices[0], stereo, mix);

		start_snapshotter(substream);

		snapshot_regs_all(substream, "before start");
		snd_emu10k1_playback_set_running(emu, epcm);
		snd_emu10k1_playback_trigger_voice(emu, epcm->voices[0]);
		snd_emu10k1_playback_trigger_voice(emu, epcm->extra);
		snapshot_regs_all(substream, "after start");

		snap_init_pre(substream, emu);

		dev_info(substream->pcm->card->dev, "playback started\n");
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		dev_info(substream->pcm->card->dev, "playback stopping\n");

		snap_init_post(substream, emu);

		snapshot_regs_all(substream, "before stop");
		snd_emu10k1_playback_stop_voice(emu, epcm->voices[0]);
		snd_emu10k1_playback_stop_voice(emu, epcm->extra);
		snd_emu10k1_playback_set_stopped(emu, epcm);
		snapshot_regs_all(substream, "after stop");

		snd_emu10k1_playback_mute_voices(emu, epcm->voices[0], stereo);
		snapshot_regs_all(substream, "after mute");

		break;
	default:
		result = -EINVAL;
		break;
	}
	spin_unlock(&emu->reg_lock);
	return result;
}

static int snd_emu10k1_capture_trigger(struct snd_pcm_substream *substream,
				       int cmd)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	int result = 0;

	spin_lock(&emu->reg_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		/* hmm this should cause full and half full interrupt to be raised? */
		outl(epcm->capture_ipr, emu->port + IPR);
		snd_emu10k1_intr_enable(emu, epcm->capture_inte);
		/*
		dev_dbg(emu->card->dev, "adccr = 0x%x, adcbs = 0x%x\n",
		       epcm->adccr, epcm->adcbs);
		*/
		switch (epcm->type) {
		case CAPTURE_AC97ADC:
			snd_emu10k1_ptr_write(emu, ADCCR, 0, epcm->capture_cr_val);
			break;
		case CAPTURE_EFX:
			if (emu->audigy) {
				snd_emu10k1_ptr_write_multiple(emu, 0,
					A_FXWC1, epcm->capture_cr_val,
					A_FXWC2, epcm->capture_cr_val2,
					REGLIST_END);
				dev_dbg(emu->card->dev,
					"cr_val=0x%x, cr_val2=0x%x\n",
					epcm->capture_cr_val,
					epcm->capture_cr_val2);
			} else
				snd_emu10k1_ptr_write(emu, FXWC, 0, epcm->capture_cr_val);
			break;
		default:	
			break;
		}
		snd_emu10k1_ptr_write(emu, epcm->capture_bs_reg, 0, epcm->capture_bs_val);
		epcm->running = 1;
		epcm->first_ptr = 1;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		epcm->running = 0;
		snd_emu10k1_intr_disable(emu, epcm->capture_inte);
		outl(epcm->capture_ipr, emu->port + IPR);
		snd_emu10k1_ptr_write(emu, epcm->capture_bs_reg, 0, 0);
		switch (epcm->type) {
		case CAPTURE_AC97ADC:
			snd_emu10k1_ptr_write(emu, ADCCR, 0, 0);
			break;
		case CAPTURE_EFX:
			if (emu->audigy) {
				snd_emu10k1_ptr_write_multiple(emu, 0,
					A_FXWC1, 0,
					A_FXWC2, 0,
					REGLIST_END);
			} else
				snd_emu10k1_ptr_write(emu, FXWC, 0, 0);
			break;
		default:
			break;
		}
		break;
	default:
		result = -EINVAL;
	}
	spin_unlock(&emu->reg_lock);
	return result;
}

static snd_pcm_uframes_t snd_emu10k1_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	int shift = emu->emu1010.clock_shift;
	int ptr;

#if PROC_CTL_LOOP
	// force a timeout after first expected period
	if (emu->init_loop_start != -1 || emu->init_loop_end != -1)
		return 0;
#endif

	if (!epcm->running)
		return 0;

	ptr = snd_emu10k1_ptr_read(emu, CCCA, epcm->voices[0]->number) & 0x00ffffff;
	ptr -= epcm->ccca_start_addr;

	// This is the size of the whole cache minus the interpolator read-ahead,
	// which leads us to the actual playback position.
	//
	// The cache is constantly kept mostly filled, so in principle we could
	// return a more advanced position representing how far the hardware has
	// already read the buffer, and set runtime->delay accordingly. However,
	// this would be slightly different for every channel (and remarkably slow
	// to obtain), so only a fixed worst-case value would be practical.
	//
	ptr -= 64 - 3;
	if (ptr < 0)
		ptr += runtime->buffer_size >> shift;

	/*
	dev_dbg(emu->card->dev,
	       "ptr = 0x%lx, buffer_size = 0x%lx, period_size = 0x%lx\n",
	       (long)ptr, (long)runtime->buffer_size,
	       (long)runtime->period_size);
	*/

	return ptr << shift;
}

static u64 snd_emu10k1_efx_playback_voice_mask(struct snd_emu10k1_pcm *epcm,
					       bool stereo, int count, int channels)
{
	u64 mask = 0;
	u64 mask0 = (1 << (count << stereo)) - 1;

	for (int i = 0; i < channels; i++) {
		int voice = epcm->voices[i]->number;
		mask |= mask0 << voice;
	}
	return mask;
}

static void snd_emu10k1_efx_playback_freeze_voices(struct snd_emu10k1 *emu,
						   struct snd_emu10k1_pcm *epcm,
						   bool stereo, int count, int channels)
{
	for (int i = 0; i < channels; i++) {
		int voice = epcm->voices[i]->number;
		for (int j = 0; j < count; j++, voice += 2) {
			snd_emu10k1_ptr_write(emu, CPF_STOP, voice, 1);
			if (stereo) {
				// Weirdly enough, the stereo slave needs to be stopped separately
				snd_emu10k1_ptr_write(emu, CPF_STOP, voice + 1, 1);
			}
			snd_emu10k1_playback_commit_pitch(emu, voice, PITCH_48000 << 16);
		}
	}
}

static void snd_emu10k1_efx_playback_unmute_voices(struct snd_emu10k1 *emu,
						   struct snd_emu10k1_pcm *epcm,
						   int channels)
{
	for (int i = 0; i < channels; i++)
		snd_emu10k1_playback_unmute_voice(emu, epcm->voices[i], false, true,
						  &emu->efx_pcm_mixer[i]);
}

static void snd_emu10k1_efx_playback_unmute_das_voices(struct snd_emu10k1 *emu,
						       struct snd_emu10k1_pcm *epcm,
						       int count, int channels)
{
	for (int i = 0; i < channels; i++)
		for (int j = 0; j < count; j++)
			snd_emu10k1_playback_unmute_das_voices(emu, epcm->voices[i] + j * 2);
}

static void snd_emu10k1_efx_playback_stop_voices(struct snd_emu10k1 *emu,
						 struct snd_emu10k1_pcm *epcm,
						 bool stereo, int count, int channels)
{
	for (int i = 0; i < channels; i++)
		for (int j = 0; j < count; j++)
			snd_emu10k1_playback_stop_voice(emu, epcm->voices[i] + j * 2);
	snd_emu10k1_playback_set_stopped(emu, epcm);

	for (int i = 0; i < channels; i++)
		for (int j = 0; j < count; j++)
			snd_emu10k1_playback_mute_voices(
						emu, epcm->voices[i] + j * 2, stereo);
}

static int snd_emu10k1_efx_playback_trigger(struct snd_pcm_substream *substream,
				        int cmd)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	unsigned shift = emu->emu1010.clock_shift;
	unsigned count = 1U << shift;
	bool das_mode = emu->das_mode;
	u64 mask;
	int result = 0;

	spin_lock(&emu->reg_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		start_snapshotter(substream);

		log_strm_ctl_event(substream, 30);

		mask = snd_emu10k1_efx_playback_voice_mask(
				epcm, das_mode, count, runtime->channels);
		for (int i = 0; i < 10; i++) {
			// Note that the freeze is not interruptible, so we make no
			// effort to reset the bits outside the error handling here.
			snd_emu10k1_voice_set_loop_stop_multiple(emu, mask);
			snapshot_regs_all(epcm->substream, "before freezing");
			snd_emu10k1_efx_playback_freeze_voices(
					emu, epcm, das_mode, count, runtime->channels);
			log_strm_ctl_event(substream, 40);
			snapshot_regs_all(epcm->substream, "after freezing");

			snd_emu10k1_playback_prepare_voices(
					emu, epcm, true, das_mode, shift, runtime->channels);
			log_strm_ctl_event(substream, 50);

			// It might seem to make more sense to unmute the voices only after
			// they have been started, to potentially avoid torturing the speakers
			// if something goes wrong. However, we cannot unmute atomically,
			// which means that we'd get some mild artifacts in the regular case.
			snapshot_regs_all(substream, "before unmute");
			if (das_mode)
				snd_emu10k1_efx_playback_unmute_das_voices(
						emu, epcm, count, runtime->channels);
			else
				snd_emu10k1_efx_playback_unmute_voices(
						emu, epcm, runtime->channels);
			log_strm_ctl_event(substream, 60);

			snapshot_regs_all(substream, "before start");
			snd_emu10k1_playback_set_running(emu, epcm);
			result = snd_emu10k1_voice_clear_loop_stop_multiple_atomic(emu, mask);
			log_strm_ctl_event(substream, 70);

			if (result == 0) {
				// The extra voice is allowed to lag a bit
				if (!emu->das_mode)
					snd_emu10k1_playback_trigger_voice(emu, epcm->extra);

				log_strm_ctl_event(substream, 80);

				snapshot_regs_all(substream, "after start");

				snap_init_pre(substream, emu);

				dev_info(substream->pcm->card->dev, "playback started\n");
				goto leave;
			}
			log_strm_ctl_event(substream, -40);
			snapshot_regs_all(substream, "after failed start");

			snd_emu10k1_efx_playback_stop_voices(
					emu, epcm, das_mode, count, runtime->channels);

			if (result != -EAGAIN)
				break;
			// The sync start can legitimately fail due to NMIs, etc.
			udelay(100);
		}
		log_strm_ctl_event(substream, -50);
		snd_emu10k1_voice_clear_loop_stop_multiple(emu, mask);
		log_strm_ctl_event(substream, -60);

		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev_info(substream->pcm->card->dev, "playback stopping\n");
		snap_init_post(substream, emu);

		log_strm_ctl_event(substream, -10);

		snapshot_regs_all(substream, "before stop");
		if (!emu->das_mode)
			snd_emu10k1_playback_stop_voice(emu, epcm->extra);
		snd_emu10k1_efx_playback_stop_voices(
				emu, epcm, das_mode, count, runtime->channels);
		snapshot_regs_all(substream, "after stop");

		log_strm_ctl_event(substream, -20);

		epcm->resume_pos = snd_emu10k1_playback_pointer(substream);
		break;
	default:
		result = -EINVAL;
		break;
	}
leave:
	spin_unlock(&emu->reg_lock);
	return result;
}

static void *get_dma_ptr(struct snd_pcm_runtime *runtime,
			 int channel, unsigned long hwoff)
{
	return runtime->dma_area + hwoff +
		channel * (runtime->dma_bytes / runtime->channels);
}

static void *get_dma_ptr_x(struct snd_pcm_runtime *runtime,
			   int shift, int channel, int subch, unsigned long hwoff)
{
	return runtime->dma_area + hwoff +
		((channel << shift) + subch) *
			(runtime->dma_bytes / (runtime->channels << shift));
}

static int snd_emu10k1_efx_playback_silence(struct snd_pcm_substream *substream,
					    int channel, unsigned long hwoff,
					    unsigned long bytes)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned shift = emu->emu1010.clock_shift;
	unsigned i, j, channels, subchans, voices;

#if LOG_PB_FILL
	//log_strm_event(substream, PbFillLogReg, (hwoff + bytes) * 100 / runtime->dma_bytes);
#endif
	if (!shift) {
		// Non-interleaved buffer is assumed
//		dev_info(emu->card->dev, "fill silence non-IL, "
//			 "channel %d, hwoff %#lx, bytes %lu",
//			 channel, hwoff, bytes);
		memset(get_dma_ptr(runtime, channel, hwoff), 0, bytes);
	} else {
		// Interleaved buffer is assumed, which isn't actually the case
		channels = runtime->channels;
		subchans = 1 << shift;
		voices = channels << shift;
//		dev_info(emu->card->dev, "fill silence, "
//			 "hwoff %#lx, bytes %lu; channels %u, subchans %u, voices %u, bytes/voice %lu",
//			 hwoff, bytes, channels, subchans, voices, bytes / voices);
		hwoff /= voices;
		if (bytes % (voices << 2))  // See *_copy_user() below.
			return -EIO;
		bytes /= voices;
		for (i = 0; i < channels; i++)
			for (j = 0; j < subchans; j++)
				memset(get_dma_ptr_x(runtime, shift, i, j, hwoff), 0, bytes);
	}
	return 0;
}

static int snd_emu10k1_efx_playback_copy(struct snd_pcm_substream *substream,
					 int channel, unsigned long hwoff,
					 struct iov_iter *iter, unsigned long bytes)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned shift = emu->emu1010.clock_shift;
	unsigned i, j, k, channels, subchans, voices, frame_size, frames;

#if LOG_PB_FILL
	log_strm_event(substream, PbFillLogReg, (hwoff + bytes) * 100 / runtime->dma_bytes);
#endif
	if (!shift) {
		// Non-interleaved source
//		dev_info(emu->card->dev, "copy from user non-IL, "
//			 "channel %d, hwoff %#lx, bytes %lu",
//			 channel, hwoff, bytes);
		if (copy_from_iter(get_dma_ptr(runtime, channel, hwoff), bytes, iter) != bytes)
			return -EFAULT;
	} else {
		// Interleaved source
		if (snd_BUG_ON(iter->nr_segs != 1))  // We don't support generalized iovec's.
			return -EIO;
		channels = runtime->channels;
		subchans = 1 << shift;
		voices = channels << shift;
		frame_size = voices << 2;
		// It is recommended that writes are period-sized, and it appears
		// unlikely that someone would actually use a period size which
		// is not divisible by four, so don't bother making it work.
		// This check should also prevent that hwoff becomes unaligned.
		// Ideally, snd_pcm_sw_params.xfer_align would handle this ...
		if (snd_BUG_ON(bytes % frame_size))
			return -EIO;
		frames = bytes / frame_size;
//		dev_info(emu->card->dev, "copy from user, "
//			 "hwoff %#lx, bytes %lu; channels %u, subchans %u, voices %u, frames %u",
//			 hwoff, bytes, channels, subchans, voices, frames);
		hwoff /= voices;
		if (iter_is_ubuf(iter)) {
			void __user *buf = iter->ubuf;
			if (!user_access_begin(buf, bytes))
				return -EFAULT;
			for (i = 0; i < channels; i++) {
				for (j = 0; j < subchans; j++) {
					u32 *dst = get_dma_ptr_x(runtime, shift, i, j, hwoff);
					u32 __user *src = (u32 __user *)buf + j * channels + i;
					for (k = 0; k < frames; k++, dst++, src += voices)
						unsafe_get_user(*dst, src, faulted);
				}
			}
			user_access_end();
		} else if (iov_iter_is_kvec(iter)) {
			void *buf = iter->kvec->iov_base;
			for (i = 0; i < channels; i++) {
				for (j = 0; j < subchans; j++) {
					u32 *dst = get_dma_ptr_x(runtime, shift, i, j, hwoff);
					u32 *src = (u32 *)buf + j * channels + i;
					for (k = 0; k < frames; k++, dst++, src += voices)
						*dst = *src;
				}
			}
		} else {
			snd_BUG();
			return -EIO;
		}
	}
	return 0;

faulted:
	user_access_end();
	return -EFAULT;
}

static snd_pcm_uframes_t snd_emu10k1_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm = runtime->private_data;
	unsigned int ptr;

	if (!epcm->running)
		return 0;
	if (epcm->first_ptr) {
		udelay(50);	/* hack, it takes awhile until capture is started */
		epcm->first_ptr = 0;
	}
	ptr = snd_emu10k1_ptr_read(emu, epcm->capture_idx_reg, 0) & 0x0000ffff;
	//dev_info(emu->card->dev, "cap ptr %u\n", ptr);
	return bytes_to_frames(runtime, ptr);
}

/*
 *  Playback support device description
 */

static const struct snd_pcm_hardware snd_emu10k1_playback =
{
	.info =			(SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_PAUSE),
	.formats =		SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_96000,
	.rate_min =		4000,
	.rate_max =		96000,
	.channels_min =		1,
	.channels_max =		2,
	.buffer_bytes_max =	(128*1024),
	.period_bytes_max =	(128*1024),
	.periods_min =		2,
	.periods_max =		1024,
	.fifo_size =		0,
};

/*
 *  Capture support device description
 */

static const struct snd_pcm_hardware snd_emu10k1_capture =
{
	.info =			(SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT,
	.rate_min =		8000,
	.rate_max =		48000,
	.channels_min =		1,
	.channels_max =		2,
	.buffer_bytes_max =	(64*1024),
	.period_bytes_min =	384,
	.period_bytes_max =	(64*1024),
	.periods_min =		2,
	.periods_max =		2,
	.fifo_size =		0,
};

static const struct snd_pcm_hardware snd_emu10k1_capture_efx =
{
	.info =			(SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_48000,
	.rate_min =		48000,
	.rate_max =		48000,
	.channels_min =		1,
	.channels_max =		16,
	.buffer_bytes_max =	(64*1024),
	.period_bytes_min =	384,
	.period_bytes_max =	(64*1024),
	.periods_min =		2,
	.periods_max =		2,
	.fifo_size =		0,
};

/*
 *
 */

static void snd_emu10k1_pcm_mixer_notify1(struct snd_emu10k1 *emu, struct snd_kcontrol *kctl, int idx, int activate)
{
	struct snd_ctl_elem_id id;

	if (! kctl)
		return;
	if (activate)
		kctl->vd[idx].access &= ~SNDRV_CTL_ELEM_ACCESS_INACTIVE;
	else
		kctl->vd[idx].access |= SNDRV_CTL_ELEM_ACCESS_INACTIVE;
	snd_ctl_notify(emu->card, SNDRV_CTL_EVENT_MASK_VALUE |
		       SNDRV_CTL_EVENT_MASK_INFO,
		       snd_ctl_build_ioff(&id, kctl, idx));
}

static void snd_emu10k1_pcm_mixer_notify(struct snd_emu10k1 *emu, int idx, int activate)
{
	snd_emu10k1_pcm_mixer_notify1(emu, emu->ctl_send_routing, idx, activate);
	snd_emu10k1_pcm_mixer_notify1(emu, emu->ctl_send_volume, idx, activate);
	snd_emu10k1_pcm_mixer_notify1(emu, emu->ctl_attn, idx, activate);
}

static void snd_emu10k1_pcm_efx_mixer_notify(struct snd_emu10k1 *emu, int idx, int activate)
{
	snd_emu10k1_pcm_mixer_notify1(emu, emu->ctl_efx_send_routing, idx, activate);
	snd_emu10k1_pcm_mixer_notify1(emu, emu->ctl_efx_send_volume, idx, activate);
	snd_emu10k1_pcm_mixer_notify1(emu, emu->ctl_efx_attn, idx, activate);
}

static void snd_emu10k1_pcm_clock_mutiplier_notify(struct snd_emu10k1 *emu)
{
	struct snd_kcontrol *kctl = emu->ctl_clock_shift;
	struct snd_ctl_elem_id id;

	// Modifying the clock multiplier during playback/capture
	// would make a mess, so we lock it.
	if (emu->emu1010.clock_users) {
		if (!(kctl->vd[0].access & SNDRV_CTL_ELEM_ACCESS_WRITE))
			return;
		kctl->vd[0].access &= ~SNDRV_CTL_ELEM_ACCESS_WRITE;
	} else {
		if (kctl->vd[0].access & SNDRV_CTL_ELEM_ACCESS_WRITE)
			return;
		kctl->vd[0].access |= SNDRV_CTL_ELEM_ACCESS_WRITE;
	}
	snd_ctl_build_ioff(&id, kctl, 0);
	snd_ctl_notify(emu->card, SNDRV_CTL_EVENT_MASK_INFO, &id);
}

static void snd_emu10k1_pcm_free_substream(struct snd_pcm_runtime *runtime)
{
	kfree(runtime->private_data);
}

static int snd_emu10k1_efx_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_pcm_mixer *mix;
	int i;

	log_strm_ctl_event(substream, -100);

	stop_snapshotter(substream);

#if LOG_PB_POS
	log_strm_event(substream, PbLogReg, -100);
#endif
#if LOG_PB_FILL
	log_strm_event(substream, PbFillLogReg, -100);
#endif
	if (emu->das_mode) {
		emu->emu1010.clock_users--;
		snd_emu10k1_pcm_clock_mutiplier_notify(emu);
		return 0;
	}
	for (i = 0; i < NUM_EFX_PLAYBACK; i++) {
		mix = &emu->efx_pcm_mixer[i];
		mix->epcm = NULL;
		snd_emu10k1_pcm_efx_mixer_notify(emu, i, 0);
	}
	return 0;
}

static int snd_emu10k1_playback_set_constraints(struct snd_emu10k1 *emu,
						struct snd_pcm_runtime *runtime)
{
	int err;

	// The buffer size must be a multiple of the period size, to avoid a
	// mismatch between the extra voice and the regular voices.
	err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		return err;
	// The hardware is typically the cache's size of 64 frames ahead.
	// Leave enough time for actually filling up the buffer.
	err = snd_pcm_hw_constraint_minmax(
			runtime, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
			128 << emu->emu1010.clock_shift, UINT_MAX);
	return err;
}

static int snd_emu10k1_efx_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_pcm *epcm;
	struct snd_emu10k1_pcm_mixer *mix;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int i, j, err;

	epcm = kzalloc(sizeof(*epcm), GFP_KERNEL);
	if (epcm == NULL)
		return -ENOMEM;
	epcm->emu = emu;
	epcm->type = PLAYBACK_EFX;
	epcm->substream = substream;
	
	runtime->private_data = epcm;
	runtime->private_free = snd_emu10k1_pcm_free_substream;
	runtime->hw = snd_emu10k1_efx_playback;
	if (emu->card_capabilities->emu_model) {
		snd_emu1010_constrain_efx_rate(emu, runtime);
		if (emu->das_mode) {
			unsigned shift = emu->emu1010.clock_shift;
			if (shift) {
				runtime->hw.info =
					// No SNDRV_PCM_INFO_MMAP; doable without SNDRV_PCM_INFO_MMAP_VALID
					SNDRV_PCM_INFO_INTERLEAVED |  // Unlike in 1x mode
					SNDRV_PCM_INFO_BLOCK_TRANSFER |
					SNDRV_PCM_INFO_RESUME |
					SNDRV_PCM_INFO_PAUSE;
				if (shift == 2)
					runtime->hw.channels_max = 8;
				err = snd_pcm_hw_constraint_step(
						runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 1 << shift);
				if (err < 0) {
					kfree(epcm);
					return err;
				}
			}
			runtime->hw.formats = SNDRV_PCM_FMTBIT_S32_LE;
			runtime->hw.periods_max = 2;  // Not using an extra voice
		}
	}
	err = snd_emu10k1_playback_set_constraints(emu, runtime);
	if (err < 0) {
		kfree(epcm);
		return err;
	}

	launch_snapshotter(substream);
	log_strm_ctl_event(substream, 0);
#if LOG_PB_POS
	log_strm_event(substream, PbLogReg, -50);
#endif
#if LOG_PB_FILL
	log_strm_event(substream, PbFillLogReg, -50);
#endif

	if (emu->das_mode) {
		emu->emu1010.clock_users++;
		snd_emu10k1_pcm_clock_mutiplier_notify(emu);
		return 0;
	}
	for (i = 0; i < NUM_EFX_PLAYBACK; i++) {
		mix = &emu->efx_pcm_mixer[i];
		for (j = 0; j < 8; j++)
			mix->send_routing[0][j] = i + j;
		memset(&mix->send_volume, 0, sizeof(mix->send_volume));
		mix->send_volume[0][0] = 255;
		mix->attn[0] = 0x8000;
		mix->epcm = epcm;
		snd_emu10k1_pcm_efx_mixer_notify(emu, i, 1);
	}
	return 0;
}

static int snd_emu10k1_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_pcm *epcm;
	struct snd_emu10k1_pcm_mixer *mix;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int i, err, sample_rate;

	epcm = kzalloc(sizeof(*epcm), GFP_KERNEL);
	if (epcm == NULL)
		return -ENOMEM;
	epcm->emu = emu;
	epcm->type = PLAYBACK_EMUVOICE;
	epcm->substream = substream;
	runtime->private_data = epcm;
	runtime->private_free = snd_emu10k1_pcm_free_substream;
	runtime->hw = snd_emu10k1_playback;
	err = snd_emu10k1_playback_set_constraints(emu, runtime);
	if (err < 0) {
		kfree(epcm);
		return err;
	}
	if (emu->card_capabilities->emu_model)
		sample_rate = emu->emu1010.word_clock;
	else
		sample_rate = 48000;
	err = snd_pcm_hw_rule_noresample(runtime, sample_rate);
	if (err < 0) {
		kfree(epcm);
		return err;
	}

	launch_snapshotter(substream);
#if LOG_PB_POS
	log_strm_event(substream, PbLogReg, -100);
#endif
#if LOG_PB_FILL
	log_strm_event(substream, PbFillLogReg, -100);
#endif

	mix = &emu->pcm_mixer[substream->number];
	for (i = 0; i < 8; i++)
		mix->send_routing[0][i] = mix->send_routing[1][i] = mix->send_routing[2][i] = i;
	memset(&mix->send_volume, 0, sizeof(mix->send_volume));
	mix->send_volume[0][0] = mix->send_volume[0][1] =
	mix->send_volume[1][0] = mix->send_volume[2][1] = 255;
	mix->attn[0] = mix->attn[1] = mix->attn[2] = 0x8000;
	mix->epcm = epcm;
	snd_emu10k1_pcm_mixer_notify(emu, substream->number, 1);
	return 0;
}

static int snd_emu10k1_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_pcm_mixer *mix = &emu->pcm_mixer[substream->number];

#if LOG_PB_POS
	log_strm_event(substream, PbLogReg, -50);
#endif
#if LOG_PB_FILL
	log_strm_event(substream, PbFillLogReg, -50);
#endif
	stop_snapshotter(substream);
	mix->epcm = NULL;
	snd_emu10k1_pcm_mixer_notify(emu, substream->number, 0);
	return 0;
}

static int snd_emu10k1_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_pcm *epcm;

	epcm = kzalloc(sizeof(*epcm), GFP_KERNEL);
	if (epcm == NULL)
		return -ENOMEM;
	epcm->emu = emu;
	epcm->type = CAPTURE_AC97ADC;
	epcm->substream = substream;
	epcm->capture_ipr = IPR_ADCBUFFULL|IPR_ADCBUFHALFFULL;
	epcm->capture_inte = INTE_ADCBUFENABLE;
	epcm->capture_ba_reg = ADCBA;
	epcm->capture_bs_reg = ADCBS;
	epcm->capture_idx_reg = emu->audigy ? A_ADCIDX : ADCIDX;
	runtime->private_data = epcm;
	runtime->private_free = snd_emu10k1_pcm_free_substream;
	runtime->hw = snd_emu10k1_capture;
	snd_emu10k1_constrain_capture_rates(emu, runtime);
	snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				   &hw_constraints_capture_buffer_sizes);
	emu->capture_interrupt = snd_emu10k1_pcm_ac97adc_interrupt;
	emu->pcm_capture_substream = substream;
#if LOG_CAP_POS
	log_strm_event(substream, CapLogReg, -100);
#endif
	return 0;
}

static int snd_emu10k1_capture_close(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);

#if LOG_CAP_POS
	log_strm_event(substream, CapLogReg, -50);
#endif
	emu->capture_interrupt = NULL;
	emu->pcm_capture_substream = NULL;
	return 0;
}

static int snd_emu10k1_capture_mic_open(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_pcm *epcm;
	struct snd_pcm_runtime *runtime = substream->runtime;

	epcm = kzalloc(sizeof(*epcm), GFP_KERNEL);
	if (epcm == NULL)
		return -ENOMEM;
	epcm->emu = emu;
	epcm->type = CAPTURE_AC97MIC;
	epcm->substream = substream;
	epcm->capture_ipr = IPR_MICBUFFULL|IPR_MICBUFHALFFULL;
	epcm->capture_inte = INTE_MICBUFENABLE;
	epcm->capture_ba_reg = MICBA;
	epcm->capture_bs_reg = MICBS;
	epcm->capture_idx_reg = emu->audigy ? A_MICIDX : MICIDX;
	substream->runtime->private_data = epcm;
	substream->runtime->private_free = snd_emu10k1_pcm_free_substream;
	runtime->hw = snd_emu10k1_capture;
	runtime->hw.rates = SNDRV_PCM_RATE_8000;
	runtime->hw.rate_min = runtime->hw.rate_max = 8000;
	snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				   &hw_constraints_capture_buffer_sizes);
	emu->capture_mic_interrupt = snd_emu10k1_pcm_ac97mic_interrupt;
	emu->pcm_capture_mic_substream = substream;
	return 0;
}

static int snd_emu10k1_capture_mic_close(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);

	emu->capture_mic_interrupt = NULL;
	emu->pcm_capture_mic_substream = NULL;
	return 0;
}

static int snd_emu10k1_capture_efx_open(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_pcm *epcm;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int nefx = emu->audigy ? 64 : 32;
	int idx, err;

	epcm = kzalloc(sizeof(*epcm), GFP_KERNEL);
	if (epcm == NULL)
		return -ENOMEM;
	epcm->emu = emu;
	epcm->type = CAPTURE_EFX;
	epcm->substream = substream;
	epcm->capture_ipr = IPR_EFXBUFFULL|IPR_EFXBUFHALFFULL;
	epcm->capture_inte = INTE_EFXBUFENABLE;
	epcm->capture_ba_reg = FXBA;
	epcm->capture_bs_reg = FXBS;
	epcm->capture_idx_reg = FXIDX;
	substream->runtime->private_data = epcm;
	substream->runtime->private_free = snd_emu10k1_pcm_free_substream;
	runtime->hw = snd_emu10k1_capture_efx;
	if (emu->card_capabilities->emu_model) {
		snd_emu1010_constrain_efx_rate(emu, runtime);
		/*
		 * There are 32 mono channels of 16bits each.
		 * 24bit Audio uses 2x channels over 16bit,
		 * 96kHz uses 2x channels over 48kHz,
		 * 192kHz uses 4x channels over 48kHz.
		 * So, for 48kHz 24bit, one has 16 channels,
		 * for 96kHz 24bit, one has 8 channels,
		 * for 192kHz 24bit, one has 4 channels.
		 * 1010rev2 and 1616(m) cards have double that,
		 * but we don't exceed 16 channels anyway.
		 */
		if (emu->das_mode)
			runtime->hw.channels_max =
				min(16, 32 >> (emu->emu1010.clock_shift +
#if MANIPULATE_FX
					       0));
#else
					       !emu->card_capabilities->emu_in_32));
#endif
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S32_LE;
	} else {
		spin_lock_irq(&emu->reg_lock);
		runtime->hw.channels_min = runtime->hw.channels_max = 0;
		for (idx = 0; idx < nefx; idx++) {
			if (emu->efx_voices_mask[idx/32] & (1 << (idx%32))) {
				runtime->hw.channels_min++;
				runtime->hw.channels_max++;
			}
		}
		epcm->capture_cr_val = emu->efx_voices_mask[0];
		epcm->capture_cr_val2 = emu->efx_voices_mask[1];
		spin_unlock_irq(&emu->reg_lock);
	}
	err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
					 &hw_constraints_efx_capture_channels);
	if (err < 0) {
		kfree(epcm);
		return err;
	}
	snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				   &hw_constraints_capture_buffer_sizes);
	emu->capture_efx_interrupt = snd_emu10k1_pcm_efx_interrupt;
	emu->pcm_capture_efx_substream = substream;

#if LOG_CAP_POS
	log_strm_event(substream, CapLogReg, -100);
#endif
	if (emu->das_mode) {
		emu->emu1010.clock_users++;
		snd_emu10k1_pcm_clock_mutiplier_notify(emu);
	}

	return 0;
}

static int snd_emu10k1_capture_efx_close(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);

#if LOG_CAP_POS
	log_strm_event(substream, CapLogReg, -50);
#endif
	if (emu->das_mode) {
		emu->emu1010.clock_users--;
		snd_emu10k1_pcm_clock_mutiplier_notify(emu);
	}

	emu->capture_efx_interrupt = NULL;
	emu->pcm_capture_efx_substream = NULL;
	return 0;
}

static const struct snd_pcm_ops snd_emu10k1_playback_ops = {
	.open =			snd_emu10k1_playback_open,
	.close =		snd_emu10k1_playback_close,
	.hw_params =		snd_emu10k1_playback_hw_params,
	.hw_free =		snd_emu10k1_playback_hw_free,
	.prepare =		snd_emu10k1_playback_prepare,
	.trigger =		snd_emu10k1_playback_trigger,
	.pointer =		snd_emu10k1_playback_pointer,
};

static const struct snd_pcm_ops snd_emu10k1_capture_ops = {
	.open =			snd_emu10k1_capture_open,
	.close =		snd_emu10k1_capture_close,
	.prepare =		snd_emu10k1_capture_prepare,
	.trigger =		snd_emu10k1_capture_trigger,
	.pointer =		snd_emu10k1_capture_pointer,
};

/* EFX playback */
static const struct snd_pcm_ops snd_emu10k1_efx_playback_ops = {
	.open =			snd_emu10k1_efx_playback_open,
	.close =		snd_emu10k1_efx_playback_close,
	.hw_params =		snd_emu10k1_playback_hw_params,
	.hw_free =		snd_emu10k1_playback_hw_free,
	.prepare =		snd_emu10k1_efx_playback_prepare,
	.trigger =		snd_emu10k1_efx_playback_trigger,
	.pointer =		snd_emu10k1_playback_pointer,
	.copy =			snd_emu10k1_efx_playback_copy,
	.fill_silence =		snd_emu10k1_efx_playback_silence,
};

int snd_emu10k1_pcm(struct snd_emu10k1 *emu, int device)
{
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	int err;

	err = snd_pcm_new(emu->card, "emu10k1", device, 32, 1, &pcm);
	if (err < 0)
		return err;

	pcm->private_data = emu;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_emu10k1_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_emu10k1_capture_ops);

	pcm->info_flags = 0;
	pcm->dev_subclass = SNDRV_PCM_SUBCLASS_GENERIC_MIX;
	strcpy(pcm->name, "ADC Capture/Standard PCM Playback");
	emu->pcm = pcm;

	/* playback substream can't use managed buffers due to alignment */
	for (substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream; substream; substream = substream->next)
		snd_pcm_lib_preallocate_pages(substream, SNDRV_DMA_TYPE_DEV_SG,
					      &emu->pci->dev,
					      64*1024, 64*1024);

	for (substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream; substream; substream = substream->next)
		snd_pcm_set_managed_buffer(substream, SNDRV_DMA_TYPE_DEV,
					   &emu->pci->dev, 64*1024, 64*1024);

	return 0;
}

int snd_emu10k1_pcm_multi(struct snd_emu10k1 *emu, int device)
{
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	int err;

	err = snd_pcm_new(emu->card, "emu10k1", device, 1, 0, &pcm);
	if (err < 0)
		return err;

	pcm->private_data = emu;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_emu10k1_efx_playback_ops);

	pcm->info_flags = 0;
	pcm->dev_subclass = SNDRV_PCM_SUBCLASS_GENERIC_MIX;
	strcpy(pcm->name, "Multichannel Playback");
	emu->pcm_multi = pcm;

	for (substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream; substream; substream = substream->next)
		snd_pcm_lib_preallocate_pages(substream, SNDRV_DMA_TYPE_DEV_SG,
					      &emu->pci->dev,
					      64*1024, 64*1024);

	return 0;
}


static const struct snd_pcm_ops snd_emu10k1_capture_mic_ops = {
	.open =			snd_emu10k1_capture_mic_open,
	.close =		snd_emu10k1_capture_mic_close,
	.prepare =		snd_emu10k1_capture_prepare,
	.trigger =		snd_emu10k1_capture_trigger,
	.pointer =		snd_emu10k1_capture_pointer,
};

int snd_emu10k1_pcm_mic(struct snd_emu10k1 *emu, int device)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(emu->card, "emu10k1 mic", device, 0, 1, &pcm);
	if (err < 0)
		return err;

	pcm->private_data = emu;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_emu10k1_capture_mic_ops);

	pcm->info_flags = 0;
	strcpy(pcm->name, "Mic Capture");
	emu->pcm_mic = pcm;

	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_DEV, &emu->pci->dev,
				       64*1024, 64*1024);

	return 0;
}

static int snd_emu10k1_pcm_efx_voices_mask_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	struct snd_emu10k1 *emu = snd_kcontrol_chip(kcontrol);
	int nefx = emu->audigy ? 64 : 32;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = nefx;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int snd_emu10k1_pcm_efx_voices_mask_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_emu10k1 *emu = snd_kcontrol_chip(kcontrol);
	int nefx = emu->audigy ? 64 : 32;
	int idx;
	
	for (idx = 0; idx < nefx; idx++)
		ucontrol->value.integer.value[idx] = (emu->efx_voices_mask[idx / 32] & (1 << (idx % 32))) ? 1 : 0;
	return 0;
}

static int snd_emu10k1_pcm_efx_voices_mask_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_emu10k1 *emu = snd_kcontrol_chip(kcontrol);
	unsigned int nval[2], bits;
	int nefx = emu->audigy ? 64 : 32;
	int change, idx;
	
	nval[0] = nval[1] = 0;
	for (idx = 0, bits = 0; idx < nefx; idx++)
		if (ucontrol->value.integer.value[idx]) {
			nval[idx / 32] |= 1 << (idx % 32);
			bits++;
		}

	if (bits == 9 || bits == 11 || bits == 13 || bits == 15 || bits > 16)
		return -EINVAL;

	spin_lock_irq(&emu->reg_lock);
	change = (nval[0] != emu->efx_voices_mask[0]) ||
		(nval[1] != emu->efx_voices_mask[1]);
	emu->efx_voices_mask[0] = nval[0];
	emu->efx_voices_mask[1] = nval[1];
	spin_unlock_irq(&emu->reg_lock);
	return change;
}

static const struct snd_kcontrol_new snd_emu10k1_pcm_efx_voices_mask = {
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.name = "Captured FX8010 Outputs",
	.info = snd_emu10k1_pcm_efx_voices_mask_info,
	.get = snd_emu10k1_pcm_efx_voices_mask_get,
	.put = snd_emu10k1_pcm_efx_voices_mask_put
};

static const struct snd_pcm_ops snd_emu10k1_capture_efx_ops = {
	.open =			snd_emu10k1_capture_efx_open,
	.close =		snd_emu10k1_capture_efx_close,
	.prepare =		snd_emu10k1_capture_prepare,
	.trigger =		snd_emu10k1_capture_trigger,
	.pointer =		snd_emu10k1_capture_pointer,
};


/* EFX playback */

#define INITIAL_TRAM_SHIFT     14
#define INITIAL_TRAM_POS(size) ((((size) / 2) - INITIAL_TRAM_SHIFT) - 1)

static void snd_emu10k1_fx8010_playback_irq(struct snd_emu10k1 *emu, void *private_data)
{
	struct snd_pcm_substream *substream = private_data;
	snd_pcm_period_elapsed(substream);
}

static void snd_emu10k1_fx8010_playback_tram_poke1(unsigned short *dst_left,
						   unsigned short *dst_right,
						   unsigned short *src,
						   unsigned int count,
						   unsigned int tram_shift)
{
	/*
	dev_dbg(emu->card->dev,
		"tram_poke1: dst_left = 0x%p, dst_right = 0x%p, "
	       "src = 0x%p, count = 0x%x\n",
	       dst_left, dst_right, src, count);
	*/
	if ((tram_shift & 1) == 0) {
		while (count--) {
			*dst_left-- = *src++;
			*dst_right-- = *src++;
		}
	} else {
		while (count--) {
			*dst_right-- = *src++;
			*dst_left-- = *src++;
		}
	}
}

static void fx8010_pb_trans_copy(struct snd_pcm_substream *substream,
				 struct snd_pcm_indirect *rec, size_t bytes)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_fx8010_pcm *pcm = &emu->fx8010.pcm[substream->number];
	unsigned int tram_size = pcm->buffer_size;
	unsigned short *src = (unsigned short *)(substream->runtime->dma_area + rec->sw_data);
	unsigned int frames = bytes >> 2, count;
	unsigned int tram_pos = pcm->tram_pos;
	unsigned int tram_shift = pcm->tram_shift;

	while (frames > tram_pos) {
		count = tram_pos + 1;
		snd_emu10k1_fx8010_playback_tram_poke1((unsigned short *)emu->fx8010.etram_pages.area + tram_pos,
						       (unsigned short *)emu->fx8010.etram_pages.area + tram_pos + tram_size / 2,
						       src, count, tram_shift);
		src += count * 2;
		frames -= count;
		tram_pos = (tram_size / 2) - 1;
		tram_shift++;
	}
	snd_emu10k1_fx8010_playback_tram_poke1((unsigned short *)emu->fx8010.etram_pages.area + tram_pos,
					       (unsigned short *)emu->fx8010.etram_pages.area + tram_pos + tram_size / 2,
					       src, frames, tram_shift);
	tram_pos -= frames;
	pcm->tram_pos = tram_pos;
	pcm->tram_shift = tram_shift;
}

static int snd_emu10k1_fx8010_playback_transfer(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_fx8010_pcm *pcm = &emu->fx8010.pcm[substream->number];

	return snd_pcm_indirect_playback_transfer(substream, &pcm->pcm_rec,
						  fx8010_pb_trans_copy);
}

static int snd_emu10k1_fx8010_playback_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_fx8010_pcm *pcm = &emu->fx8010.pcm[substream->number];
	unsigned int i;

	for (i = 0; i < pcm->channels; i++)
		snd_emu10k1_ptr_write(emu, TANKMEMADDRREGBASE + 0x80 + pcm->etram[i], 0, 0);
	return 0;
}

static int snd_emu10k1_fx8010_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_fx8010_pcm *pcm = &emu->fx8010.pcm[substream->number];
	unsigned int i;
	
	/*
	dev_dbg(emu->card->dev, "prepare: etram_pages = 0x%p, dma_area = 0x%x, "
	       "buffer_size = 0x%x (0x%x)\n",
	       emu->fx8010.etram_pages, runtime->dma_area,
	       runtime->buffer_size, runtime->buffer_size << 2);
	*/
	memset(&pcm->pcm_rec, 0, sizeof(pcm->pcm_rec));
	pcm->pcm_rec.hw_buffer_size = pcm->buffer_size * 2; /* byte size */
	pcm->pcm_rec.sw_buffer_size = snd_pcm_lib_buffer_bytes(substream);
	pcm->tram_pos = INITIAL_TRAM_POS(pcm->buffer_size);
	pcm->tram_shift = 0;
	snd_emu10k1_ptr_write_multiple(emu, 0,
		emu->gpr_base + pcm->gpr_running, 0,	/* reset */
		emu->gpr_base + pcm->gpr_trigger, 0,	/* reset */
		emu->gpr_base + pcm->gpr_size, runtime->buffer_size,
		emu->gpr_base + pcm->gpr_ptr, 0,	/* reset ptr number */
		emu->gpr_base + pcm->gpr_count, runtime->period_size,
		emu->gpr_base + pcm->gpr_tmpcount, runtime->period_size,
		REGLIST_END);
	for (i = 0; i < pcm->channels; i++)
		snd_emu10k1_ptr_write(emu, TANKMEMADDRREGBASE + 0x80 + pcm->etram[i], 0, (TANKMEMADDRREG_READ|TANKMEMADDRREG_ALIGN) + i * (runtime->buffer_size / pcm->channels));
	return 0;
}

static int snd_emu10k1_fx8010_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_fx8010_pcm *pcm = &emu->fx8010.pcm[substream->number];
	int result = 0;

	spin_lock(&emu->reg_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* follow thru */
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
#ifdef EMU10K1_SET_AC3_IEC958
	{
		int i;
		for (i = 0; i < 3; i++) {
			unsigned int bits;
			bits = SPCS_CLKACCY_1000PPM | SPCS_SAMPLERATE_48 |
			       SPCS_CHANNELNUM_LEFT | SPCS_SOURCENUM_UNSPEC | SPCS_GENERATIONSTATUS |
			       0x00001200 | SPCS_EMPHASIS_NONE | SPCS_COPYRIGHT | SPCS_NOTAUDIODATA;
			snd_emu10k1_ptr_write(emu, SPCS0 + i, 0, bits);
		}
	}
#endif
		result = snd_emu10k1_fx8010_register_irq_handler(emu, snd_emu10k1_fx8010_playback_irq, pcm->gpr_running, substream, &pcm->irq);
		if (result < 0)
			goto __err;
		snd_emu10k1_fx8010_playback_transfer(substream);	/* roll the ball */
		snd_emu10k1_ptr_write(emu, emu->gpr_base + pcm->gpr_trigger, 0, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		snd_emu10k1_fx8010_unregister_irq_handler(emu, &pcm->irq);
		snd_emu10k1_ptr_write(emu, emu->gpr_base + pcm->gpr_trigger, 0, 0);
		pcm->tram_pos = INITIAL_TRAM_POS(pcm->buffer_size);
		pcm->tram_shift = 0;
		break;
	default:
		result = -EINVAL;
		break;
	}
      __err:
	spin_unlock(&emu->reg_lock);
	return result;
}

static snd_pcm_uframes_t snd_emu10k1_fx8010_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_fx8010_pcm *pcm = &emu->fx8010.pcm[substream->number];
	size_t ptr; /* byte pointer */

	if (!snd_emu10k1_ptr_read(emu, emu->gpr_base + pcm->gpr_trigger, 0))
		return 0;
	ptr = snd_emu10k1_ptr_read(emu, emu->gpr_base + pcm->gpr_ptr, 0) << 2;
	return snd_pcm_indirect_playback_pointer(substream, &pcm->pcm_rec, ptr);
}

static const struct snd_pcm_hardware snd_emu10k1_fx8010_playback =
{
	.info =			(SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_RESUME |
				 /* SNDRV_PCM_INFO_MMAP_VALID | */ SNDRV_PCM_INFO_PAUSE |
				 SNDRV_PCM_INFO_SYNC_APPLPTR),
	.formats =		SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_48000,
	.rate_min =		48000,
	.rate_max =		48000,
	.channels_min =		1,
	.channels_max =		1,
	.buffer_bytes_max =	(128*1024),
	.period_bytes_min =	1024,
	.period_bytes_max =	(128*1024),
	.periods_min =		2,
	.periods_max =		1024,
	.fifo_size =		0,
};

static int snd_emu10k1_fx8010_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_emu10k1_fx8010_pcm *pcm = &emu->fx8010.pcm[substream->number];

	runtime->hw = snd_emu10k1_fx8010_playback;
	runtime->hw.channels_min = runtime->hw.channels_max = pcm->channels;
	runtime->hw.period_bytes_max = (pcm->buffer_size * 2) / 2;
	spin_lock_irq(&emu->reg_lock);
	if (pcm->valid == 0) {
		spin_unlock_irq(&emu->reg_lock);
		return -ENODEV;
	}
	pcm->opened = 1;
	spin_unlock_irq(&emu->reg_lock);
	return 0;
}

static int snd_emu10k1_fx8010_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_emu10k1 *emu = snd_pcm_substream_chip(substream);
	struct snd_emu10k1_fx8010_pcm *pcm = &emu->fx8010.pcm[substream->number];

	spin_lock_irq(&emu->reg_lock);
	pcm->opened = 0;
	spin_unlock_irq(&emu->reg_lock);
	return 0;
}

static const struct snd_pcm_ops snd_emu10k1_fx8010_playback_ops = {
	.open =			snd_emu10k1_fx8010_playback_open,
	.close =		snd_emu10k1_fx8010_playback_close,
	.hw_free =		snd_emu10k1_fx8010_playback_hw_free,
	.prepare =		snd_emu10k1_fx8010_playback_prepare,
	.trigger =		snd_emu10k1_fx8010_playback_trigger,
	.pointer =		snd_emu10k1_fx8010_playback_pointer,
	.ack =			snd_emu10k1_fx8010_playback_transfer,
};

int snd_emu10k1_pcm_efx(struct snd_emu10k1 *emu, int device)
{
	struct snd_pcm *pcm;
	struct snd_kcontrol *kctl;
	int err;

	err = snd_pcm_new(emu->card, "emu10k1 efx", device, emu->audigy ? 0 : 8, 1, &pcm);
	if (err < 0)
		return err;

	pcm->private_data = emu;

	if (!emu->audigy)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_emu10k1_fx8010_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_emu10k1_capture_efx_ops);

	pcm->info_flags = 0;
	if (emu->audigy)
		strcpy(pcm->name, "Multichannel Capture");
	else
		strcpy(pcm->name, "Multichannel Capture/PT Playback");
	emu->pcm_efx = pcm;

	if (!emu->card_capabilities->emu_model) {
		// On Sound Blasters, the DSP code copies the EXTINs to FXBUS2.
		// The mask determines which of these and the EXTOUTs the multi-
		// channel capture actually records (the channel order is fixed).
		if (emu->audigy) {
			emu->efx_voices_mask[0] = 0;
			emu->efx_voices_mask[1] = 0xffff;
		} else {
			emu->efx_voices_mask[0] = 0xffff0000;
			emu->efx_voices_mask[1] = 0;
		}
		// TODO: this should be locked while the device is open
		kctl = snd_ctl_new1(&snd_emu10k1_pcm_efx_voices_mask, emu);
		if (!kctl)
			return -ENOMEM;
		kctl->id.device = device;
		err = snd_ctl_add(emu->card, kctl);
		if (err < 0)
			return err;
	} else {
		// On E-MU cards, the DSP code copies the P16VINs/EMU32INs to
		// EXTOUT/FXBUS2. These are already selected & routed by the FPGA,
		// so there is no need to apply additional masking.
	}

	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_DEV, &emu->pci->dev,
				       64*1024, 64*1024);

	return 0;
}

int snd_emu10k1_pcm_das(struct snd_emu10k1 *emu, int device)
{
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;

	int err = snd_pcm_new(emu->card, "emu10k1 efx", device, 1, 1, &pcm);
	if (err < 0)
		return err;

	pcm->private_data = emu;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_emu10k1_efx_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_emu10k1_capture_efx_ops);

	strcpy(pcm->name, "Multichannel Playback/Capture");
	emu->pcm_das = pcm;

	// Playback substream can't use managed buffers due to IOMMU workaround
	substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	snd_pcm_lib_preallocate_pages(substream, SNDRV_DMA_TYPE_DEV_SG,
				      &emu->pci->dev, 64*1024, 64*1024);

	substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	snd_pcm_set_managed_buffer(substream, SNDRV_DMA_TYPE_DEV,
				   &emu->pci->dev, 64*1024, 64*1024);

	return 0;
}
