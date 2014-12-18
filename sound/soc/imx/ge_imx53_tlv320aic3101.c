/***********************************************************************
 *
 * @Authors: Manfred Schlaegl, Ginzinger electronic systems GmbH
 * @Descr: ge_imx53_tlv320aic3101 specific code
 * @TODO:
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 ***********************************************************************/
/***********************************************************************
 *	@History:
 *	2011KW49 - manfred.schlaegl:
 *		* using ssi1_ext_clk
 *		* copy from emtrion-kernel (based on emtrion aic-driver)
 ***********************************************************************/


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/dma.h>
#include <mach/clock.h>

#include "../codecs/tlv320aic3x.h"
#include "imx-ssi.h"
#include "imx-pcm.h"

/*
 * used clock for codec
 */
#define GE_IMX53_TLV320AIC3101_CLK_NAME 	"ssi_ext1_clk"
static struct clk *ge_imx53_tlv320aic3101_ssi_ext1_clk=NULL;

/* SSI BCLK and LRC master */
#define TLV320_SSI_MASTER	1

static struct ge_imx53_tlv320aic3101_priv card_priv;

static void ge_imx53_tlv320aic3101_init_dam(int ssi_port, int dai_port)
{
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;
	/* uses SSI1 or SSI2 via AUDMUX port dai_port for audio */

	/* reset port ssi_port & dai_port */
	__raw_writel(0, DAM_PTCR(ssi_port));
	__raw_writel(0, DAM_PTCR(dai_port));
	__raw_writel(0, DAM_PDCR(ssi_port));
	__raw_writel(0, DAM_PDCR(dai_port));

	/* set to synchronous */
	ssi_ptcr |= AUDMUX_PTCR_SYN;
	dai_ptcr |= AUDMUX_PTCR_SYN;

#if TLV320_SSI_MASTER
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TFSDIR;
	ssi_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TCLKDIR;
	ssi_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
#else
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  ssi_port --> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TFSDIR;
	dai_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, ssi_port);

	/* set Tx Clock direction and source ssi_port--> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TCLKDIR;
	dai_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, ssi_port);
#endif

	__raw_writel(ssi_ptcr, DAM_PTCR(ssi_port));
	__raw_writel(dai_ptcr, DAM_PTCR(dai_port));
	__raw_writel(ssi_pdcr, DAM_PDCR(ssi_port));
	__raw_writel(dai_pdcr, DAM_PDCR(dai_port));
}

static int ge_imx53_tlv320aic3101_dai_audio_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;

	int ret;

	ssi_mode->network_mode = 1;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		pr_err("%s: failed set cpu dai format\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		pr_err("%s: failed set codec dai format\n", __func__);
		return ret;
	}
 
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
					clk_get_rate(ge_imx53_tlv320aic3101_ssi_ext1_clk), SND_SOC_CLOCK_OUT);
	if (ret) {
		pr_err("%s: failed setting codec sysclk\n", __func__);
		return ret;
	}
	snd_soc_dai_set_tdm_slot(cpu_dai, 0xffffffc, 0xffffffc, 2, 0);

	ret = snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0,
					SND_SOC_CLOCK_IN);
	if (ret) {
		pr_err("can't set CPU system clock IMX_SSP_SYS_CLK\n");
		return ret;
	}

	return 0;

}

static int ge_imx53_tlv320aic3101_dai_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void ge_imx53_tlv320aic3101_dai_shutdown(struct snd_pcm_substream *substream)
{
}

static struct snd_soc_ops ge_imx53_tlv320aic3101_dai_ops = {
	.hw_params = ge_imx53_tlv320aic3101_dai_audio_hw_params,
};


static int ge_imx53_tlv320aic3101_dai_init(struct snd_soc_codec *codec)
{
	return 0;
}

/* ge_imx53 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ge_imx53_tlv320aic3101_dai = {
	.name = "tlv320aic3x",
	.stream_name = "tlv320aic3x",
	.codec_dai = &aic3x_dai,
	.init = ge_imx53_tlv320aic3101_dai_init,
	.ops = &ge_imx53_tlv320aic3101_dai_ops,
};

static struct snd_soc_card snd_soc_card_ge_imx53 = {
	.name = "ge_imx53_tlv320aic3101_snd",
	.platform = &imx_soc_platform,
	.dai_link = &ge_imx53_tlv320aic3101_dai,
	.num_links = 1,
	//	.remove = ge_imx53_tlv320aic3101_card_remove,
};

static struct snd_soc_device ge_imx53_tlv320aic3101_snd_devdata = {
	.card = &snd_soc_card_ge_imx53,
	.codec_dev = &soc_codec_dev_aic3x,
};

static int __devinit ge_imx53_tlv320aic3101_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct snd_soc_dai *tlv320_cpu_dai;

	ge_imx53_tlv320aic3101_ssi_ext1_clk = clk_get(NULL, GE_IMX53_TLV320AIC3101_CLK_NAME);
	if(IS_ERR(ge_imx53_tlv320aic3101_ssi_ext1_clk)) {
		printk(KERN_ERR "%s.%s: unable to get %s\n",__FILE__,__FUNCTION__,GE_IMX53_TLV320AIC3101_CLK_NAME);
		return -1;
	}
	
	ge_imx53_tlv320aic3101_init_dam(plat->src_port, plat->ext_port);

	tlv320_cpu_dai=0;
	if (plat->src_port == 2)
		tlv320_cpu_dai = imx_ssi_dai[2];
	else if (plat->src_port == 1)
		tlv320_cpu_dai = imx_ssi_dai[0];
	else if (plat->src_port == 7)
		tlv320_cpu_dai = imx_ssi_dai[4];
	
	ge_imx53_tlv320aic3101_dai.cpu_dai = tlv320_cpu_dai;

	return 0;
}

static int ge_imx53_tlv320aic3101_remove(struct platform_device *pdev)
{
	clk_put(ge_imx53_tlv320aic3101_ssi_ext1_clk);	
	return 0;
}

static struct platform_driver ge_imx53_tlv320aic3101_audio_driver = {
	.probe = ge_imx53_tlv320aic3101_probe,
	.remove = ge_imx53_tlv320aic3101_remove,
	.driver = {
		.name = "ge_imx53_tlv320aic3101",
	},
};

static struct platform_device *ge_imx53_tlv320aic3101_snd_device;

static int __init ge_imx53_tlv320aic3101_init(void)
{
	int ret;

	ret = platform_driver_register(&ge_imx53_tlv320aic3101_audio_driver);
	if (ret)
		return -ENOMEM;

	ge_imx53_tlv320aic3101_snd_device = platform_device_alloc("soc-audio", 2);
	if (!ge_imx53_tlv320aic3101_snd_device)
		return -ENOMEM;

	platform_set_drvdata(ge_imx53_tlv320aic3101_snd_device, &ge_imx53_tlv320aic3101_snd_devdata);
	ge_imx53_tlv320aic3101_snd_devdata.dev = &ge_imx53_tlv320aic3101_snd_device->dev;
	ret = platform_device_add(ge_imx53_tlv320aic3101_snd_device);

	if (ret)
		platform_device_put(ge_imx53_tlv320aic3101_snd_device);

	return ret;
}

static void __exit ge_imx53_tlv320aic3101_exit(void)
{
	platform_driver_unregister(&ge_imx53_tlv320aic3101_audio_driver);
	platform_device_unregister(ge_imx53_tlv320aic3101_snd_device);
}

module_init(ge_imx53_tlv320aic3101_init);
module_exit(ge_imx53_tlv320aic3101_exit);

MODULE_AUTHOR("Ginzinger electronic systems GmbH");
MODULE_DESCRIPTION("TLV320AIC3101 Driver for ge_imx53 based boards");
MODULE_LICENSE("GPL");
