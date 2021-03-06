/*
 * imx-pcm.c -- ALSA SoC interface for the Freescale i.MX3 CPU's
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Based on imx31-pcm.c by Nicolas Pitre, (C) 2004 MontaVista Software, Inc.
 * and on mxc-alsa-mc13783 (C) 2006-2009 Freescale.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <mach/spba.h>
#include <mach/clock.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "imx-pcm.h"
#include "imx-ssi.h"
#include "imx-esai.h"

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
#include <linux/delay.h>
#include <linux/mxc_asrc.h>
#endif

#ifdef CONFIG_SND_MXC_SOC_IRAM
static bool UseIram = 1;
#else
static bool UseIram;
#endif

/* debug */
#define IMX_PCM_DEBUG 0
#if IMX_PCM_DEBUG
#define dbg(format, arg...) printk(format, ## arg)
#else
#define dbg(format, arg...)
#endif

static const struct snd_pcm_hardware imx_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	.buffer_bytes_max = SND_RAM_SIZE,
	.period_bytes_max = SND_RAM_SIZE / 4,
#else
	.buffer_bytes_max = 64 * 1024,
	.period_bytes_max = 16 * 1024,
#endif
	.period_bytes_min = 2 * SZ_1K,
	.periods_min = 2,
	.periods_max = 255,
	.fifo_size = 0,
};

static uint32_t audio_iram_phys_base_addr;
static void *audio_iram_virt_base_addr;

static struct vm_operations_struct snd_mxc_audio_playback_vm_ops = {
	.open = snd_pcm_mmap_data_open,
	.close = snd_pcm_mmap_data_close,
};

/*
	enable user space access to iram buffer
*/
static int imx_iram_audio_playback_mmap(struct snd_pcm_substream *substream,
					struct vm_area_struct *area)
{
	unsigned long off;
	unsigned long phys;
	unsigned long size;
	int ret = 0;

	area->vm_ops = &snd_mxc_audio_playback_vm_ops;
	area->vm_private_data = substream;

	off = area->vm_pgoff << PAGE_SHIFT;
	phys = audio_iram_phys_base_addr + off;
	size = area->vm_end - area->vm_start;

	if (off + size > SND_RAM_SIZE)
		return -EINVAL;

	area->vm_page_prot = pgprot_nonshareddev(area->vm_page_prot);
	area->vm_flags |= VM_IO;
	ret =
	    remap_pfn_range(area, area->vm_start, phys >> PAGE_SHIFT,
			    size, area->vm_page_prot);
	if (ret == 0)
		area->vm_ops->open(area);

	return ret;
}

/*
     Map nbytes in virtual space
     bytes -audio iram iram partition size
     phys_addr - physical address of iram buffer
     returns - virtual address of the iram buffer or NULL if fail
*/
static void *imx_iram_init(dma_addr_t *phys_addr, size_t bytes)
{
	void *iram_base;

	iram_base = (void *)ioremap((uint32_t) SND_RAM_BASE_ADDR, bytes);

	audio_iram_virt_base_addr = iram_base;
	audio_iram_phys_base_addr = (uint32_t) SND_RAM_BASE_ADDR;
	*phys_addr = (dma_addr_t) SND_RAM_BASE_ADDR;

	return audio_iram_virt_base_addr;

}

/*
     destroy the virtual mapping of the iram buffer
*/

static void imx_iram_free(void)
{
	iounmap(audio_iram_virt_base_addr);
}

static int imx_get_sdma_transfer(int format, int dai_port,
				 struct snd_pcm_substream *substream)
{
	int transfer = -1;

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd = runtime->private_data;
	if (prtd->asrc_enable == 1) {
		if (dai_port == IMX_DAI_SSI0) {
			if (prtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_SSI1_TX0;
			else if (prtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_SSI1_TX0;
		} else if (dai_port == IMX_DAI_SSI1) {
			if (prtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_SSI1_TX1;
			else if (prtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_SSI1_TX1;
		} else if (dai_port == IMX_DAI_SSI2) {
			if (prtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_SSI2_TX0;
			else if (prtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_SSI2_TX0;
		} else if (dai_port == IMX_DAI_SSI3) {
			if (prtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_SSI2_TX1;
			else if (prtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_SSI2_TX1;
		} else if (dai_port & IMX_DAI_ESAI_TX) {
			if (prtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_ESAI;
			else if (prtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_ESAI;
			else
				transfer = MXC_DMA_ASRCC_ESAI;
		}
	} else {
#endif

		if (dai_port == IMX_DAI_SSI0) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI1_16BIT_TX0;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI1_24BIT_TX0;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI1_24BIT_TX0;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI1_16BIT_RX0;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI1_24BIT_RX0;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI1_24BIT_RX0;
			}
		} else if (dai_port == IMX_DAI_SSI1) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI1_16BIT_TX1;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI1_24BIT_TX1;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI1_24BIT_TX1;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI1_16BIT_RX1;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI1_24BIT_RX1;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI1_24BIT_RX1;
			}
		} else if (dai_port == IMX_DAI_SSI2) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI2_16BIT_TX0;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI2_24BIT_TX0;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI2_24BIT_TX0;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI2_16BIT_RX0;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI2_24BIT_RX0;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI2_24BIT_RX0;
			}
		} else if (dai_port == IMX_DAI_SSI3) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI2_16BIT_TX1;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI2_24BIT_TX1;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI2_24BIT_TX1;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI2_16BIT_RX1;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI2_24BIT_RX1;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI2_24BIT_RX1;
			}
		} else if ((dai_port & IMX_DAI_ESAI_TX)
			   || (dai_port & IMX_DAI_ESAI_RX)) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_ESAI_16BIT_TX;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_ESAI_24BIT_TX;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_ESAI_24BIT_TX;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_ESAI_16BIT_RX;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_ESAI_24BIT_RX;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_ESAI_24BIT_RX;
			}
		}
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	}
#endif
	return transfer;
}

static int dma_new_period(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd = runtime->private_data;
	unsigned int dma_size = frames_to_bytes(runtime, runtime->period_size);
	unsigned int offset = dma_size * prtd->period;
	int ret = 0;
	mxc_dma_requestbuf_t sdma_request;

	if (!prtd->active)
		return 0;

	memset(&sdma_request, 0, sizeof(mxc_dma_requestbuf_t));

	dbg("period pos  ALSA %x DMA %x\n", runtime->periods, prtd->period);
	dbg("period size ALSA %x DMA %x Offset %x dmasize %x\n",
	    (unsigned int)runtime->period_size,
	    runtime->dma_bytes, offset, dma_size);
	dbg("DMA addr %x\n", runtime->dma_addr + offset);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sdma_request.src_addr =
		    (dma_addr_t) (runtime->dma_addr + offset);
	else
		sdma_request.dst_addr =
		    (dma_addr_t) (runtime->dma_addr + offset);

	sdma_request.num_of_bytes = dma_size;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mxc_dma_config(prtd->dma_wchannel,
			       &sdma_request, 1, MXC_DMA_MODE_WRITE);
		ret = mxc_dma_enable(prtd->dma_wchannel);
	} else {

		mxc_dma_config(prtd->dma_wchannel,
			       &sdma_request, 1, MXC_DMA_MODE_READ);
		ret = mxc_dma_enable(prtd->dma_wchannel);
	}
	prtd->dma_active = 1;
	prtd->period++;
	prtd->period %= runtime->periods;

	return ret;
}

static void audio_dma_irq(void *data)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd = runtime->private_data;

	prtd->dma_active = 0;
	prtd->periods++;
	prtd->periods %= runtime->periods;

	dbg("irq per %d offset %x\n", prtd->periods,
	    frames_to_bytes(runtime, runtime->period_size) * prtd->periods);

	if (prtd->active)
		snd_pcm_period_elapsed(substream);
	dma_new_period(substream);
}

static int imx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd = runtime->private_data;
	int ret = 0, channel = 0;

	if (prtd->dma_alloc) {
		mxc_dma_free(prtd->dma_wchannel);
		prtd->dma_alloc = 0;
	}

	/* only allocate the DMA chn once */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
		if (prtd->asrc_enable == 1) {
			struct dma_channel_info info;
			mxc_dma_requestbuf_t sdma_request;
			info.asrc.channs = runtime->channels;
			if (prtd->dma_asrc) {
				mxc_dma_free(prtd->dma_asrc);
				prtd->dma_asrc = 0;
			}
			memset(&sdma_request, 0, sizeof(mxc_dma_requestbuf_t));
			/* num_of_bytes can be set any value except for zero */
			sdma_request.num_of_bytes = 0x40;
			channel =
			    mxc_dma_request_ext(prtd->dma_ch,
						"ALSA TX SDMA", &info);

			mxc_dma_config(channel, &sdma_request,
				       1, MXC_DMA_MODE_WRITE);
			prtd->dma_asrc = channel;
			if (prtd->asrc_index == 0)
				prtd->dma_ch = MXC_DMA_ASRC_A_RX;
			else if (prtd->asrc_index == 1)
				prtd->dma_ch = MXC_DMA_ASRC_B_RX;
			else
				prtd->dma_ch = MXC_DMA_ASRC_C_RX;

			channel =
			    mxc_dma_request(MXC_DMA_ASRC_A_RX, "ALSA ASRC RX");
		} else
			channel = mxc_dma_request(prtd->dma_ch, "ALSA TX SDMA");
#else
		channel = mxc_dma_request(prtd->dma_ch, "ALSA TX SDMA");
#endif
		if (channel < 0) {
			pr_err("imx-pcm: error requesting \
					a write dma channel\n");
			return channel;
		}
		ret = mxc_dma_callback_set(channel, (mxc_dma_callback_t)
					   audio_dma_irq, (void *)substream);

	} else {
		channel = mxc_dma_request(prtd->dma_ch, "ALSA RX SDMA");
		if (channel < 0) {
			pr_err("imx-pcm: error requesting \
				a read dma channel\n");
			return channel;
		}
		ret = mxc_dma_callback_set(channel, (mxc_dma_callback_t)
					   audio_dma_irq, (void *)substream);
	}
	prtd->dma_wchannel = channel;
	prtd->dma_alloc = 1;

	prtd->period = 0;
	prtd->periods = 0;
	return 0;
}

static int imx_pcm_hw_params(struct snd_pcm_substream
			     *substream, struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	prtd->dma_ch =
	    imx_get_sdma_transfer(params_format(params),
				  rtd->dai->cpu_dai->id, substream);

	if (prtd->dma_ch < 0) {
		printk(KERN_ERR "imx-pcm: invaild sdma transfer type");
		return -1;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int imx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd = runtime->private_data;

	if (prtd->dma_wchannel) {
		mxc_dma_free(prtd->dma_wchannel);
		prtd->dma_wchannel = 0;
		prtd->dma_alloc = 0;
	}
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	if ((prtd->asrc_enable == 1) && prtd->dma_asrc) {
		mxc_dma_free(prtd->dma_asrc);
		prtd->dma_asrc = 0;
	}
#endif

	return 0;
}

static int imx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mxc_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		prtd->dma_active = 0;
		prtd->active = 1;
		ret = dma_new_period(substream);
		ret = dma_new_period(substream);
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
		if (prtd->asrc_enable == 1) {
			ret = mxc_dma_enable(prtd->dma_asrc);
			asrc_start_conv(prtd->asrc_index);
			/* There is underrun, if immediately enable SSI after
			   start ASRC */
			mdelay(1);
		}
#endif
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		prtd->active = 0;
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
		if (prtd->asrc_enable == 1) {
			mxc_dma_disable(prtd->dma_asrc);
			asrc_stop_conv(prtd->asrc_index);
		}
#endif
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t imx_pcm_pointer(struct
					 snd_pcm_substream
					 *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd = runtime->private_data;
	unsigned int offset = 0;

	offset = (runtime->period_size * (prtd->periods));
	if (offset >= runtime->buffer_size)
		offset = 0;
	dbg("pointer offset %x\n", offset);

	return offset;
}

static int imx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &imx_pcm_hardware);

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	prtd = kzalloc(sizeof(struct mxc_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	runtime->private_data = prtd;
	return 0;
}

static int imx_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *prtd = runtime->private_data;

	kfree(prtd);
	return 0;
}

static int
imx_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_dai *cpu_dai = socdev->machine->dai_link->cpu_dai;
	struct mxc_audio_platform_data *dev_data = cpu_dai->private_data;
	int ext_ram = 0;
	int ret = 0;

	dbg("+imx_pcm_mmap:"
	    "UseIram=%d dma_addr=%x dma_area=%x dma_bytes=%d\n",
	    UseIram, (unsigned int)runtime->dma_addr,
	    runtime->dma_area, runtime->dma_bytes);

	if (dev_data)
		ext_ram = dev_data->ext_ram;

	if ((substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	    || ext_ram || !UseIram) {
		ret =
		    dma_mmap_writecombine(substream->pcm->card->
					  dev, vma,
					  runtime->dma_area,
					  runtime->dma_addr,
					  runtime->dma_bytes);
		return ret;
	} else
		return imx_iram_audio_playback_mmap(substream, vma);
}

struct snd_pcm_ops imx_pcm_ops = {
	.open = imx_pcm_open,
	.close = imx_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = imx_pcm_hw_params,
	.hw_free = imx_pcm_hw_free,
	.prepare = imx_pcm_prepare,
	.trigger = imx_pcm_trigger,
	.pointer = imx_pcm_pointer,
	.mmap = imx_pcm_mmap,
};

static int imx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_dai *cpu_dai = socdev->machine->dai_link->cpu_dai;
	struct mxc_audio_platform_data *dev_data = cpu_dai->private_data;
	int ext_ram = 0;
	size_t size = imx_pcm_hardware.buffer_bytes_max;

	if (dev_data)
		ext_ram = dev_data->ext_ram;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	if ((stream == SNDRV_PCM_STREAM_CAPTURE) || ext_ram || !UseIram)
		buf->area =
		    dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	else
		buf->area = imx_iram_init(&buf->addr, size);

	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	printk(KERN_INFO "DMA Sound Buffers Allocated:"
	       "UseIram=%d buf->addr=%x buf->area=%p size=%d\n",
	       UseIram, buf->addr, buf->area, size);
	return 0;
}

static void imx_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_dai *cpu_dai = socdev->machine->dai_link->cpu_dai;
	struct mxc_audio_platform_data *dev_data = cpu_dai->private_data;
	int ext_ram = 0;
	int stream;

	if (dev_data)
		ext_ram = dev_data->ext_ram;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		if ((stream == SNDRV_PCM_STREAM_CAPTURE)
		    || ext_ram || !UseIram)
			dma_free_writecombine(pcm->card->dev,
					      buf->bytes, buf->area, buf->addr);
		else
			imx_iram_free();
		buf->area = NULL;
	}
}

static u64 imx_pcm_dmamask = 0xffffffff;

static int imx_pcm_new(struct snd_card *card,
		       struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &imx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->playback.channels_min) {
		ret = imx_pcm_preallocate_dma_buffer(pcm,
						     SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = imx_pcm_preallocate_dma_buffer(pcm,
						     SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
      out:
	return ret;
}

struct snd_soc_platform imx_soc_platform = {
	.name = "imx-audio",
	.pcm_ops = &imx_pcm_ops,
	.pcm_new = imx_pcm_new,
	.pcm_free = imx_pcm_free_dma_buffers,
};

EXPORT_SYMBOL_GPL(imx_soc_platform);

MODULE_AUTHOR("Liam Girdwood");
MODULE_DESCRIPTION("Freescale i.MX3x PCM DMA module");
MODULE_LICENSE("GPL");
