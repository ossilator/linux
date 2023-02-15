// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *                   Creative Labs, Inc.
 *  Routines for IRQ control of EMU10K1 chips
 */

#include <linux/time.h>
#include <sound/core.h>
#include <sound/emu10k1.h>

#if SNAP_IRQ_HANDLER || LOG_CAP_POS || LOG_PB_POS
static u32 snap_read(struct snd_emu10k1 *emu, unsigned int reg)
{
	return inl(emu->port + reg);
}

static u32 snap_ptr_read(struct snd_emu10k1 *emu, unsigned int reg, unsigned int chn)
{
	outl((reg << 16) | chn, emu->port + PTR);
	return inl(emu->port + DATA);
}

#if LOG_CAP_POS || LOG_PB_POS
static void snap_ptr_write(struct snd_emu10k1 *emu, unsigned int reg, unsigned int chn, u32 val)
{
	outl((reg << 16) | chn, emu->port + PTR);
	outl(val, emu->port + DATA);
}

static void log_irq_event(struct snd_emu10k1 *emu, int log, int val)
{
	snap_ptr_write(emu, A_FXGPREGBASE + emu->fngprs + log, 0,
		       0x7fffffffLL * val / 100);
}
#endif
#endif

irqreturn_t snd_emu10k1_interrupt(int irq, void *dev_id)
{
	struct snd_emu10k1 *emu = dev_id;
	unsigned int status, orig_status;
	int handled = 0;
	int timeout = 0;

#if SNAP_IRQ_HANDLER || LOG_CAP_POS || LOG_PB_POS
	spin_lock(&emu->emu_lock);
	do {
		u32 ipr = snap_read(emu, IPR);
#if LOG_CAP_POS
		if (ipr & (IPR_EFXBUFFULL | IPR_EFXBUFHALFFULL))
			log_irq_event(emu, CapLogReg,
				  emu->snap_cap_size ?
				  REG_VAL_GET(FXIDX_IDX, snap_ptr_read(emu, FXIDX, 0))
				  * 100 / emu->snap_cap_size : -75);
#endif
#if !SNAP_IRQ_DICE
		if (!(ipr & IPR_CHANNELLOOP))
			break;
#endif
#if SNAP_IRQ_HANDLER
		int nums = emu->num_irq_snaps;
		if (nums < ARRAY_SIZE(emu->snap_irq)) {
			u32 wc = snap_read(emu, WC);
#if SNAP_IRQ_DICE
			u32 dice = snap_ptr_read(emu, A_DICE, 0);
#endif
			struct emu_irq_snapshot *is = &emu->snap_irq[nums];
			emu->num_irq_snaps = nums + 1;
			for (int c = 0; c < emu->snap_total; c++) {
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
#if SNAP_IRQ_CLIP
			is->clipl = snap_ptr_read(emu, CLIPL, 0);
			is->cliph = snap_ptr_read(emu, CLIPH, 0);
#endif
#if SNAP_IRQ_HLIP
			is->hlipl = snap_ptr_read(emu, HLIPL, 0);
			is->hliph = snap_ptr_read(emu, HLIPH, 0);
#endif
#if SNAP_IRQ_CLIE
			is->cliel = snap_ptr_read(emu, CLIEL, 0);
			is->clieh = snap_ptr_read(emu, CLIEH, 0);
#endif
			is->wc = wc;
			is->ipr = ipr;
#if SNAP_IRQ_DICE
			is->dice = dice;
#endif
		}
#endif
#if LOG_PB_POS
#if SNAP_IRQ_DICE
		if (ipr & IPR_CHANNELLOOP)
#endif
		log_irq_event(emu, PbLogReg,
			  emu->snap_pb_size ?
			  (REG_VAL_GET(CCCA_CURRADDR, snap_ptr_read(emu, CCCA, emu->snap_voices[0])) -
			  emu->snap_pb_base) * 100 / emu->snap_pb_size : -50);
#endif
	} while (0);
	spin_unlock(&emu->emu_lock);
#endif

	while ((status = inl(emu->port + IPR)) != 0) {
		handled = 1;
		if ((status & 0xffffffff) == 0xffffffff) {
			dev_info(emu->card->dev,
				 "Suspected sound card removal\n");
			break;
		}
		if ((status & IPR_CHANNELLOOP) && emu->das_mode &&
		    (snd_emu10k1_ptr_read(emu, A_DICE, 0) & A_DICE_COUNTER_MASK)) {
			// If another IRQ "punched through" the delay,
			// we mask away the delayed IRQ.
			status &= ~(IPR_CHANNELLOOP | IPR_CHANNELNUMBERMASK);
			if (!status)  // Will usually happen in 2nd iteration
				break;
		}
		if (++timeout == 1000) {
			dev_info(emu->card->dev, "emu10k1 irq routine failure\n");
			break;
		}
		orig_status = status;
		if (status & IPR_PCIERROR) {
			dev_err(emu->card->dev, "interrupt: PCI error\n");
			snd_emu10k1_intr_disable(emu, INTE_PCIERRORENABLE);
			status &= ~IPR_PCIERROR;
		}
		if (status & (IPR_VOLINCR|IPR_VOLDECR|IPR_MUTE)) {
			if (emu->hwvol_interrupt)
				emu->hwvol_interrupt(emu, status);
			else
				snd_emu10k1_intr_disable(emu, INTE_VOLINCRENABLE|INTE_VOLDECRENABLE|INTE_MUTEENABLE);
			status &= ~(IPR_VOLINCR|IPR_VOLDECR|IPR_MUTE);
		}
		if (status & IPR_CHANNELLOOP) {
			struct snd_emu10k1_voice *pvoice;
			int voice;
			int voice_max = status & IPR_CHANNELNUMBERMASK;
			u32 val;

			val = snd_emu10k1_ptr_read(emu, CLIPL, 0);
			pvoice = emu->voices;
			for (voice = 0; voice <= voice_max; voice++) {
				if (voice == 0x20)
					val = snd_emu10k1_ptr_read(emu, CLIPH, 0);
				if (val & 1) {
					if (pvoice->use && pvoice->interrupt != NULL) {
						pvoice->interrupt(emu, pvoice);
						snd_emu10k1_voice_intr_ack(emu, voice);
					} else {
						snd_emu10k1_voice_intr_disable(emu, voice);
					}
				}
				val >>= 1;
				pvoice++;
			}
			val = snd_emu10k1_ptr_read(emu, HLIPL, 0);
			pvoice = emu->voices;
			for (voice = 0; voice <= voice_max; voice++) {
				if (voice == 0x20)
					val = snd_emu10k1_ptr_read(emu, HLIPH, 0);
				if (val & 1) {
					if (pvoice->use && pvoice->interrupt != NULL) {
						pvoice->interrupt(emu, pvoice);
						snd_emu10k1_voice_half_loop_intr_ack(emu, voice);
					} else {
						snd_emu10k1_voice_half_loop_intr_disable(emu, voice);
					}
				}
				val >>= 1;
				pvoice++;
			}
			status &= ~(IPR_CHANNELLOOP | IPR_CHANNELNUMBERMASK);
		}
		if (status & (IPR_ADCBUFFULL|IPR_ADCBUFHALFFULL)) {
			if (emu->capture_interrupt)
				emu->capture_interrupt(emu, status);
			else
				snd_emu10k1_intr_disable(emu, INTE_ADCBUFENABLE);
			status &= ~(IPR_ADCBUFFULL|IPR_ADCBUFHALFFULL);
		}
		if (status & (IPR_MICBUFFULL|IPR_MICBUFHALFFULL)) {
			if (emu->capture_mic_interrupt)
				emu->capture_mic_interrupt(emu, status);
			else
				snd_emu10k1_intr_disable(emu, INTE_MICBUFENABLE);
			status &= ~(IPR_MICBUFFULL|IPR_MICBUFHALFFULL);
		}
		if (status & (IPR_EFXBUFFULL|IPR_EFXBUFHALFFULL)) {
			if (emu->capture_efx_interrupt)
				emu->capture_efx_interrupt(emu, status);
			else
				snd_emu10k1_intr_disable(emu, INTE_EFXBUFENABLE);
			status &= ~(IPR_EFXBUFFULL|IPR_EFXBUFHALFFULL);
		}
		if (status & (IPR_MIDITRANSBUFEMPTY|IPR_MIDIRECVBUFEMPTY)) {
			if (emu->midi.interrupt)
				emu->midi.interrupt(emu, status);
			else
				snd_emu10k1_intr_disable(emu, INTE_MIDITXENABLE|INTE_MIDIRXENABLE);
			status &= ~(IPR_MIDITRANSBUFEMPTY|IPR_MIDIRECVBUFEMPTY);
		}
		if (status & (IPR_A_MIDITRANSBUFEMPTY2|IPR_A_MIDIRECVBUFEMPTY2)) {
			if (emu->midi2.interrupt)
				emu->midi2.interrupt(emu, status);
			else
				snd_emu10k1_intr_disable(emu, INTE_A_MIDITXENABLE2|INTE_A_MIDIRXENABLE2);
			status &= ~(IPR_A_MIDITRANSBUFEMPTY2|IPR_A_MIDIRECVBUFEMPTY2);
		}
		if (status & IPR_INTERVALTIMER) {
			if (emu->timer)
				snd_timer_interrupt(emu->timer, emu->timer->sticks);
			else
				snd_emu10k1_intr_disable(emu, INTE_INTERVALTIMERENB);
			status &= ~IPR_INTERVALTIMER;
		}
		if (status & (IPR_GPSPDIFSTATUSCHANGE|IPR_CDROMSTATUSCHANGE)) {
			if (emu->spdif_interrupt)
				emu->spdif_interrupt(emu, status);
			else
				snd_emu10k1_intr_disable(emu, INTE_GPSPDIFENABLE|INTE_CDSPDIFENABLE);
			status &= ~(IPR_GPSPDIFSTATUSCHANGE|IPR_CDROMSTATUSCHANGE);
		}
		if (status & IPR_FXDSP) {
			if (emu->dsp_interrupt)
				emu->dsp_interrupt(emu);
			else
				snd_emu10k1_intr_disable(emu, INTE_FXDSPENABLE);
			status &= ~IPR_FXDSP;
		}
		if (status & IPR_P16V) {
			if (emu->p16v_interrupt)
				emu->p16v_interrupt(emu);
			else
				outl(0, emu->port + INTE2);
			status &= ~IPR_P16V;
		}
		if (status & IPR_A_GPIO) {
			//u32 gpio, sts, sts2;
			//gpio = inl(emu->port + A_IOCFG);
			//snd_emu1010_fpga_read(emu, EMU_HANA_IRQ_STATUS, &sts);
			//snd_emu1010_fpga_read(emu, EMU_HANA_IRQ_STATUS, &sts2);
			//dev_info(emu->card->dev, "got gpio int, gpi=0x%04x, sts=%#02x, sts2=%#02x\n", gpio, sts, sts2);

			if (emu->gpio_interrupt)
				emu->gpio_interrupt(emu);
			else
				snd_emu10k1_intr_disable(emu, INTE_A_GPIOENABLE);
			status &= ~IPR_A_GPIO;
		}

		if (status) {
			dev_err(emu->card->dev,
				"unhandled interrupt: 0x%08x\n", status);
		}
		outl(orig_status, emu->port + IPR); /* ack all */
	}

	return IRQ_RETVAL(handled);
}
