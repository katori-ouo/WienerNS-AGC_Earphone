/********************************************************************************************************
 * @file	mic_spk_app.c
 *
 * @brief	This is the source file for B91m
 *
 * @author	Driver Group
 * @date	2019
 *
 * @copyright Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#include "app_config.h"
#if (USB_DEMO_TYPE == USB_MIC_SPEAKER)
#include "usb_default.h"
#include "application/usbstd/usb.h"
#include "channel_process.h"
#include <string.h>

#include "algo/nsx.h"
#include "algo/agc.h"

void *pNS_instL = NULL;
void *AGC_instL = NULL;
InnoTalkAgc_config_t agcConfig;
#if LIIROPT
void *HPF_instL = NULL;
#define nstage        (3)   //hpf阶数
#endif

#if (MCU_CORE_B91)
#define MIC_DMA_CHN DMA2
#define SPK_DMA_CHN DMA3
#define MIC_BUFFER_SIZE 2048
#define SPK_BUFFER_SIZE 2048

#define MEMCPY_START_THD (FRAME_LEN << 1)
#define TIMER_TICK (1000 * FRAME_LEN / 16)

#define ALIGN_BUFFER(addr, size) ((addr) & ((size)-1))
#define TO_NUM(x, type) (((x) + sizeof(type) - 1) / (sizeof(type)))

unsigned short iso_in_buff[MIC_BUFFER_SIZE];
unsigned short audio_rx[MIC_BUFFER_SIZE];
unsigned short iso_out_buff[SPK_BUFFER_SIZE];

volatile unsigned int mcu_rx_r = 0;

#define RAW_PATH 0

#if LIIROPT
/**
 * @brief 对输入的信号以128点为块做hpf
 *
 * @param in 输入信号
 * @param out 输出信号
 * @param len 信号长度, 以byte为单位
 */
void hpfprocess(void* inst, signed short *in, signed short *out, int len)
{
    signed short *data = in;
    signed short *post = out;

    while(len){
    	InnoTalkHpf_ProcessFloat(inst, data, post);
        len -= FRAME_LEN;
        data += FRAME_LEN;
        post += FRAME_LEN;
    }
}
#endif

/**
 * @brief 对输入的信号以128点为块做ns
 *
 * @param in 输入信号
 * @param out 输出信号
 * @param len 信号长度, 以byte为单位
 */
void audioProcess(void* ns_inst, void* agc_inst, signed short *in, signed short *out, int len)
{
    signed short *data = in;
    signed short *post = out;
    signed short tmp[FRAME_LEN] = { 0 };

    while (len) {
    	InnoTalkNsx_ProcessCore(ns_inst, data, tmp);
    	InnoTalkAgc_Process(agc_inst, tmp, post, agcConfig);

        len -= FRAME_LEN;
        data += FRAME_LEN;
        post += FRAME_LEN;
    }
}


int iso_in_w = 0;
int iso_in_r = 0;

int iso_out_w = 0;
int iso_out_r = 0;


void audio_to_usb_buff()
{
    /* step 1. get dma rx write length and tx read length in bytes */
    volatile unsigned int rx_w =
        ((audio_get_rx_dma_wptr(MIC_DMA_CHN) - (unsigned int)audio_rx) >> 1);
    int available_rx_num = 0, unused_usb_num = 0;
    signed short l[MEMCPY_START_THD >> 1] = {0};
    signed short r[MEMCPY_START_THD >> 1] = {0};
    signed short post_l[MEMCPY_START_THD >> 1] = {0};
    // signed short post_r[MEMCPY_START_THD >> 1] = {0};

    /* step 2. calculate how much data in audio_rx buffer */
    available_rx_num =
        (rx_w >= mcu_rx_r) ? (rx_w - mcu_rx_r) : (MIC_BUFFER_SIZE + rx_w - mcu_rx_r);

    /* step 2.1. calculate how much space in usb buffer can be used */
    unused_usb_num = (iso_in_w >= iso_in_r) ? (MIC_BUFFER_SIZE - (iso_in_w - iso_in_r))
                                            : (iso_in_r - iso_in_w);

    /* step 3. copy data from audio_rx to audio_tx if all the conditions are
       satisfied */
    if (available_rx_num >= MEMCPY_START_THD && unused_usb_num >= MEMCPY_START_THD) {
    	gpio_set_high_level(LED1);
/* step 3.1 split data into left channel and right channel */
#if 1
        int size = split_data_to_lr((signed short *)(audio_rx + mcu_rx_r), l, r,
                                    MEMCPY_START_THD);
        // ! 调用语音处理算法
        audioProcess(pNS_instL,AGC_instL, l, post_l, size);
        // delay_ms(2);
        converge_lr_to_data(post_l, r, (signed short *)(iso_in_buff + iso_in_w), size);
#else
        delay_ms(6);
        memcpy((char *)(iso_in_buff + iso_in_w),
               (char *)(audio_rx + mcu_rx_r), MEMCPY_START_THD << 1);
#endif

        iso_in_w = ALIGN_BUFFER(iso_in_w + MEMCPY_START_THD, MIC_BUFFER_SIZE);
        mcu_rx_r = ALIGN_BUFFER(mcu_rx_r + MEMCPY_START_THD, MIC_BUFFER_SIZE);
    } else {
        gpio_set_low_level(LED1);
    }
    return;
}


#if 0

_attribute_ram_code_sec_ void timer1_irq_handler(void)
{
    if (timer_get_irq_status(TMR_STA_TMR1)) {
        // gpio_toggle(LED1);
        // gpio_set_high_level(LED1);
        // core_interrupt_disable();
        timer_clr_irq_status(TMR_STA_TMR1);
        timer_set_cap_tick(TIMER1, sys_clk.pclk * TIMER_TICK);

        audio_to_usb_buff();

        timer_start(TIMER1);
        // core_interrupt_enable();
    }
}

void interrupt_init()
{
    /* init led */
    // gpio_set_low_level(LED1);

    // gpio_set_high_level(LED1);
    plic_preempt_feature_en();
    // core_interrupt_enable();
    plic_set_priority(IRQ1_SYSTIMER, IRQ_PRI_LEV3);
    plic_set_priority(IRQ3_TIMER1, IRQ_PRI_LEV2);

    /******** timer1 init********/
    plic_interrupt_enable(IRQ3_TIMER1);
    timer_set_init_tick(TIMER1, 0);
    timer_set_mode(TIMER1, TIMER_MODE_SYSCLK);
    timer_set_cap_tick(TIMER1, sys_clk.pclk * TIMER_TICK);
    timer_start(TIMER1);
}
#else

_attribute_ram_code_sec_ void timer0_irq_handler(void)
{
    if (timer_get_irq_status(TMR_STA_TMR0)) {
        // gpio_toggle(LED1);
        // gpio_set_high_level(LED1);
        // core_interrupt_disable();
        timer_clr_irq_status(TMR_STA_TMR0);
        timer_set_cap_tick(TIMER0, sys_clk.pclk * TIMER_TICK);

        audio_to_usb_buff();

        timer_start(TIMER0);
        // core_interrupt_enable();
    }
}

void interrupt_init()
{
    /* init led */
    // gpio_set_low_level(LED1);

    // gpio_set_high_level(LED1);
    plic_preempt_feature_en();
    // core_interrupt_enable();
    plic_set_priority(IRQ1_SYSTIMER, IRQ_PRI_LEV3);
    plic_set_priority(IRQ4_TIMER0, IRQ_PRI_LEV2);

    /******** timer1 init********/
    plic_interrupt_enable(IRQ4_TIMER0);
    timer_set_init_tick(TIMER0, 0);
    timer_set_mode(TIMER0, TIMER_MODE_SYSCLK);
    timer_set_cap_tick(TIMER0, sys_clk.pclk * TIMER_TICK);
    timer_start(TIMER0);
}
#endif

/**
 * @brief     This function serves to send data to USB. only adaptive mono 16bit
 * @param[in] audio_rate - audio rate. This value is matched with usb_default.h
 * :MIC_SAMPLE_RATE.
 * @return    none.
 */

void audio_tx_data_to_usb(audio_sample_rate_e audio_rate)
{
    unsigned char length = 0;
    /* update by timer irq function */
#if RAW_PATH
    iso_in_w = ((audio_get_rx_dma_wptr(MIC_DMA_CHN) - (unsigned int)iso_in_buff) >> 1);
#endif
    usbhw_reset_ep_ptr(USB_EDP_MIC); // reset pointer of Endpoint7's buf

    switch (audio_rate) {
    case AUDIO_8K:
        length = 8 * MIC_CHANNLE_COUNT;
        break;
    case AUDIO_16K:
        length = 16 * MIC_CHANNLE_COUNT;
        break;
    case AUDIO_32K:
        length = 32 * MIC_CHANNLE_COUNT;
        break;
    case AUDIO_48K:
        length = 48 * MIC_CHANNLE_COUNT;
        break;
    default:
        length = 16 * MIC_CHANNLE_COUNT;
        break;
    }

    for (unsigned char i = 0; i < length && iso_in_r != iso_in_w; i++) {
        // short md = iso_in_buff[iso_in_r++ & (MIC_BUFFER_SIZE - 1)];
        short md = iso_in_buff[iso_in_r];
#if 1
        reg_usb_ep7_dat = md;
        reg_usb_ep7_dat = md >> 8;
#else
        reg_usb_ep7_dat = 0x12;
		reg_usb_ep7_dat = 0x34;
#endif
        iso_in_r = ALIGN_BUFFER(iso_in_r + 1, MIC_BUFFER_SIZE);
    }
    usbhw_data_ep_ack(USB_EDP_MIC);
}

/**
 * @brief     This function servers to set USB Input.
 * @param[in] none
 * @return    none.
 */
void audio_rx_data_from_usb()
{
    unsigned char len = reg_usb_ep_ptr(USB_EDP_SPEAKER);
    usbhw_reset_ep_ptr(USB_EDP_SPEAKER);
    for (unsigned int i = 0; i < len; i += 4) {
        /* read twice, low first */
        signed short d0 = reg_usb_ep6_dat;
        d0 |= reg_usb_ep6_dat << 8;
        signed short d1 = reg_usb_ep6_dat;
        d1 |= reg_usb_ep6_dat << 8;

        iso_out_buff[iso_out_w++ & (SPK_BUFFER_SIZE - 1)] = d0;
        iso_out_buff[iso_out_w++ & (SPK_BUFFER_SIZE - 1)] = d1;
    }
    /* set #6 ep to busy status */
    usbhw_data_ep_ack(USB_EDP_SPEAKER);
}

int num_iso_in = 0;
volatile int num_iso_out = 0;

_attribute_ram_code_sec_ void usb_endpoint_irq_handler(void)
{
    /////////////////////////////////////
    // ISO IN (host send IN command)
    /////////////////////////////////////
    if (usbhw_get_eps_irq() & FLD_USB_EDP7_IRQ) {
        usbhw_clr_eps_irq(FLD_USB_EDP7_IRQ); // clear interrupt flag of endpoint 7
        /////// get MIC input data ///////////////////////////////
        audio_tx_data_to_usb(AUDIO_16K);
        num_iso_in++;
        if ((num_iso_in & 0x7f) == 0)
            gpio_toggle(LED2);
    }

    /////////////////////////////////////
    // ISO OUT
    /////////////////////////////////////
    if (usbhw_get_eps_irq() & FLD_USB_EDP6_IRQ) {
        usbhw_clr_eps_irq(FLD_USB_EDP6_IRQ);
        ///////////// output to audio fifo out ////////////////
        /* return ack */
        audio_rx_data_from_usb();
        num_iso_out++;
        if ((num_iso_out & 0x7f) == 0)
            gpio_toggle(LED3);
    }
}

void user_init(void)
{

#if (CHIP_VER_A0 == CHIP_VER)
    audio_set_codec_supply(CODEC_2P8V);
#endif
    gpio_function_en(LED1 | LED2 | LED3 | LED4);
    gpio_output_en(LED1 | LED2 | LED3 | LED4);

    reg_usb_ep6_buf_addr = 0x40; // 192 max
    reg_usb_ep7_buf_addr = 0x20; // 32
    reg_usb_ep8_buf_addr = 0x00;
    reg_usb_ep_max_size = (192 >> 3);
    plic_interrupt_enable(IRQ11_USB_ENDPOINT); // enable usb endpoint interrupt
    plic_set_priority(IRQ11_USB_ENDPOINT, IRQ_PRI_LEV3);

    // 1.enable USB DP pull up 1.5k
    usb_set_pin_en();
    // 2.enable USB manual interrupt(in auto interrupt mode,USB device would be USB
    // printer device)
    usb_init_interrupt();
    // 3.enable global interrupt
    core_interrupt_enable();
    usbhw_set_irq_mask(USB_IRQ_RESET_MASK | USB_IRQ_SUSPEND_MASK);
    /* 1  "AUDIO_ADC_16K_DAC_48K" - mic sampling=16K,spk sampling=48K;
     * "AUDIO_16K"-mic sampling=16K,spk sampling=16K,
     * should match the setting  in usb_default.h
     * 2  mic and spk  channel count should be the same (1 or 2 ) in usb_default.h
     * 3  channel count =1,MONO_BIT_16;channel count =2  STEREO_BIT_16
     */
#if 0
    audio_set_dmic_pin(DMIC_GROUPD_D4_DAT_D5_D6_CLK);
    audio_init(DMIC_IN_TO_BUF_TO_LINE_OUT, AUDIO_16K, STEREO_BIT_16);
#else
	audio_init(AMIC_IN_TO_BUF_TO_LINE_OUT, AUDIO_16K, STEREO_BIT_16);
    // ! 定义ADC寄存器的HPF
	audio_set_codec_adc_wnf(CODEC_ADC_WNF_MODE3); //HFP
#endif

#if RAW_PATH
    audio_rx_dma_chain_init(MIC_DMA_CHN, iso_in_buff, MIC_BUFFER_SIZE * 2);
#else
    interrupt_init();
    audio_rx_dma_chain_init(MIC_DMA_CHN, audio_rx, MIC_BUFFER_SIZE * 2);
#endif
    audio_tx_dma_chain_init(SPK_DMA_CHN, iso_out_buff, SPK_BUFFER_SIZE * 2);

    // ! 初始化agc和ns等算法
#if LIIROPT
    float rcoeff[nstage] = {-0.8876, 0.9983, -0.9989}; // hpf r系数, 和matlab的k系数倒序
    float lcoeff[nstage + 1] = {-0.94252, 0.11082, 0.00464, -0.000016}; //hpf l系数, 和matlab的v系数倒序
    InnoTalkHpf_Create(&HPF_instL);
    InnoTalkHpf_InitCore(HPF_instL, rcoeff, lcoeff, nstage);
#endif
	InnoTalkNsx_Create(&pNS_instL);
	InnoTalkNsx_InitCore(pNS_instL, (uint32_t)16000);

	InnoTalkAgc_Create(&AGC_instL);
	int32_t minLevel = 0;
	int32_t maxLevel = 255;
	// int agcMode = kAgcModeAdaptiveDigital;
	InnoTalkAgc_Init(AGC_instL, minLevel, maxLevel, (uint32_t)16000);
	agcConfig.compressionGaindB = (int16_t)18;
	agcConfig.limiterEnable = (uint8_t)1;
	agcConfig.targetLevelDbfs = (int16_t)3;
	agcConfig.SilenceGainFall = (int16_t)110;
	agcConfig.AgcVadLowThr = (int16_t)0;                //JT:AGC中VAD判定低门限,Q10，推荐值为0
	agcConfig.AgcVadUppThr = (int16_t)1024;             //JT:AGC中VAD判定高门限,Q10，推荐值为1024
	agcConfig.GateLowThr = (int16_t)0;                  //JT:Gate判定低门限，推荐值为0
	agcConfig.GateUppThr = (int16_t)2500;               //JT:Gate判定高门限，推荐值为2500
	agcConfig.GateVadSensitivity = (int16_t)6;             //JT:Gate计算对语音活性的敏感度，推荐值为6
	InnoTalkAgc_set_config(AGC_instL, agcConfig);

}
unsigned int t = 0;
int temp = 0;
void main_loop(void)
{

    usb_handle_irq();
    if (clock_time_exceed(t, 2000) && (num_iso_out)) // 2ms
    {
        if (num_iso_out == temp) {
            memset(iso_out_buff, 0,
                   SPK_BUFFER_SIZE * 2); // if host pause playback clear buff.
            num_iso_out = 0;
        }

        gpio_toggle(LED4);
        temp = num_iso_out;
        t = stimer_get_tick() | 1;
    }
}
#elif (MCU_CORE_B92)

#define MIC_DMA_CHN DMA2
#define SPK_DMA_CHN DMA3
#define MIC_BUFFER_SIZE 4096
#define SPK_BUFFER_SIZE 4096

unsigned short iso_in_buff[MIC_BUFFER_SIZE];
unsigned short iso_out_buff[SPK_BUFFER_SIZE];

void user_init(void)
{
    gpio_function_en(LED1 | LED2 | LED3 | LED4);
    gpio_output_en(LED1 | LED2 | LED3 | LED4);

    reg_usb_ep6_buf_addr = 0x40; // 192 max
    reg_usb_ep7_buf_addr = 0x20; // 32
    reg_usb_ep8_buf_addr = 0x00;
    reg_usb_ep_max_size = (192 >> 3);
    plic_interrupt_enable(IRQ11_USB_ENDPOINT); // enable usb endpoint interrupt

    // 1.enable USB DP pull up 1.5k
    usb_set_pin_en();
    // 2.enable USB manual interrupt(in auto interrupt mode,USB device would be USB
    // printer device)
    usb_init_interrupt();
    // 3.enable global interrupt
    core_interrupt_enable();
#if (USB_MODE == AISO)
    audio_data_fifo0_path_sel(USB_AISO_IN_FIFO, USB_AISO_OUT_FIFO);
    audio_rx_dma_chain_init(DMA0, (unsigned short *)iso_in_buff, MIC_BUFFER_SIZE * 2,
                            FIFO0);
    while (audio_get_rx_wptr(FIFO0) < 64)
        ;
    audio_tx_dma_chain_init(DMA1, (unsigned short *)iso_in_buff, MIC_BUFFER_SIZE * 2,
                            FIFO0);
#endif
}
void main_loop(void)
{
    usb_handle_irq();
}
#endif

#endif
