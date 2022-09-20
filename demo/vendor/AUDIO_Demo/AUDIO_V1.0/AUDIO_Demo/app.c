/********************************************************************************************************
 * @file	app.c
 *
 * @brief	This is the source file for B91m
 *
 * @author	Driver Group
 * @date	2019
 *
 * @par     Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#include "app_sin_data.h"
#include <string.h>

#include "algo/nsx.h"
#include "algo/agc.h"

unsigned int dma_rx_irq_cnt = 0;
#if ((AUDIO_MODE == LINEIN_TO_LINEOUT) || (AUDIO_MODE == AMIC_TO_LINEOUT) ||                       \
     (AUDIO_MODE == DMIC_TO_LINEOUT) || (AUDIO_MODE == EXT_CODEC_LINEIN_LINEOUT))
/* in bytes */
#define AUDIO_BUFF_SIZE 4096
/* DMIC_LINEOUT taotal 4kb, array length divded by 2 because the type of AUDIO_BUFF is int16 */
volatile signed short AUDIO_BUFF[AUDIO_BUFF_SIZE >> 1] __attribute__((aligned(4)));

volatile signed short audio_rx[AUDIO_BUFF_SIZE >> 1] __attribute__((aligned(4)));
volatile signed short audio_tx[AUDIO_BUFF_SIZE >> 1] __attribute__((aligned(4)));

volatile unsigned int mcu_src_rb = 0;
volatile unsigned int mcu_dst_wb = 0;
/* in bytes */
#define MEMCPY_START_THD (512)
/* addr除以4096的余数 */
#define ALIGN_BUFFER(addr) ((addr) & (AUDIO_BUFF_SIZE - 1))

void *pNS_instL = NULL;
void *AGC_instL = NULL;
InnoTalkAgc_config_t agcConfig;
#if LIIROPT
void *HPF_instL = NULL;
#define nstage        (3)   //hpf阶数
#endif

#elif (AUDIO_MODE == BUFFER_TO_LINEOUT)
dma_chain_config_t tx_dma_list_config[2];
#define MIC_BUFFER_SIZE (441 * 4)

#elif (AUDIO_MODE == FLASH_TO_LINEOUT)
#define FLASH_16k_SIZE 0x3400
#define FLASH_BUFF_ZERO 1024

#define FLASH_48k_SIZE 0xF000
#define AUIDO_BUFF_SIZE 4096
#define AUIDO_THD_SIZE 1024
unsigned char flash_read_16K_buff[FLASH_16k_SIZE] __attribute__((aligned(4)));
unsigned char flash_read_48K_buff[FLASH_48k_SIZE] __attribute__((aligned(4)));
unsigned char audio_buff[AUIDO_BUFF_SIZE] __attribute__((aligned(4)));

volatile unsigned int tx_rptr_out = 0;
volatile unsigned int tx_wptr_out = 0;
volatile unsigned int flash_rptr = 0;
volatile unsigned short remaining;
volatile unsigned char ex_cnt;
volatile unsigned char swith;
unsigned long t;
#endif


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
        len -= FRAME_LEN * sizeof(signed short);
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

        len -= FRAME_LEN * sizeof(signed short);
        data += FRAME_LEN;
        post += FRAME_LEN;
    }
}


int split_data_to_lr(signed short *data __attribute__((unused)), signed short *l, signed short *r, int len)
{
    for (int i = 0; i < (len >> 2); i++) {
        l[i] = data[i * 2];
        r[i] = data[i * 2 + 1];
    }
    return len >> 1;
}

int converge_lr_to_data(signed short *l __attribute__((unused)), signed short *r __attribute__((unused)), signed short *data, int len)
{
    for (int i = 0; i < (len >> 1); i++) {
        data[i * 2] = l[i];
        data[i * 2 + 1] = r[i];
    }
    return len << 1;
}

void audio_loopback()
{
    /* step 1. get dma rx write length and tx read length in bytes */
    volatile unsigned int rx_wb = audio_get_rx_dma_wptr(DMA2) - (unsigned int)audio_rx;
    volatile unsigned int tx_rb = audio_get_tx_dma_rptr(DMA3) - (unsigned int)audio_tx;
    int available_rx_size = 0, unused_tx_size = 0;
    signed short l[MEMCPY_START_THD >> 2] = {0};
    signed short r[MEMCPY_START_THD >> 2] = {0};
    signed short post_l[MEMCPY_START_THD >> 2] = {0};
    // signed short post_r[MEMCPY_START_THD >> 2] = {0};


    /* step 2. calculate how much data in audio_rx buffer */
    if (rx_wb >= mcu_src_rb) {
        available_rx_size = rx_wb - mcu_src_rb;
    } else {
        available_rx_size = AUDIO_BUFF_SIZE + rx_wb - mcu_src_rb;
    }

    /* step 2.1. calculate how much space in audio_tx buffer can be used */
    if (mcu_dst_wb >= tx_rb) {
        unused_tx_size = AUDIO_BUFF_SIZE - (mcu_dst_wb - tx_rb);
    } else {
        unused_tx_size = tx_rb - mcu_dst_wb;
    }

    if (unused_tx_size == 10000) {

    }

    /* step 3. copy data from audio_rx to audio_tx if all the conditions are satisfied */
    // if (available_rx_size > MEMCPY_START_THD && unused_tx_size > MEMCPY_START_THD) {
    if (available_rx_size >= MEMCPY_START_THD) {
        /* copy data from mic buffer to spk buffer */
        // memcpy((char *)(audio_tx + (mcu_dst_wb >> 1)),
        //       (char *)(audio_rx + (mcu_src_rb >> 1)), MEMCPY_START_THD);

        /* step 3.1 split data into left channel and right channel */
        int bytes = split_data_to_lr((signed short *)(audio_rx + (mcu_src_rb >> 1)),
                         l, r, MEMCPY_START_THD);

        /* switch left channel with right channel */
        //for (int i = 0; i < (MEMCPY_START_THD >> 2); i++) {
        //    (audio_tx + (mcu_dst_wb >> 1))[i * 2] = r[i];
        //    (audio_tx + (mcu_dst_wb >> 1))[i * 2 + 1] = l[i];
        //}

        // ! 调用语音处理算法
        audioProcess(pNS_instL, AGC_instL, l, post_l, bytes);
        // FIXME: 耳机只播放左声道
        converge_lr_to_data(post_l, r, (signed short *)(audio_tx + (mcu_dst_wb >> 1)), bytes);

        /* test mute one channel */
        // for (int i = 0; i < (MEMCPY_START_THD >> 2); i++) {
        //     volatile signed short *data = audio_tx + (mcu_dst_wb >> 1);
        //     data[i * 2 + 1] = 0;
        // }
        mcu_dst_wb = ALIGN_BUFFER(mcu_dst_wb + MEMCPY_START_THD);
        mcu_src_rb = ALIGN_BUFFER(mcu_src_rb + MEMCPY_START_THD);
    }
    return;
}

_attribute_ram_code_sec_ void timer1_irq_handler(void)
{
    if (timer_get_irq_status(TMR_STA_TMR1)) {
        gpio_set_high_level(LED2);
        core_interrupt_disable();
        timer_clr_irq_status(TMR_STA_TMR1);
        timer_set_cap_tick(TIMER1, sys_clk.pclk * 5500); // 5.5ms
        /* for test */
        // timer_set_cap_tick(TIMER1, sys_clk.pclk * 1000 * 2000); // 2s
        // delay_us(250 * 1000); // 250ms
        // gpio_set_low_level(LED2);

        /* TODO do algorithm */
        audio_loopback();

        timer_start(TIMER1);
        core_interrupt_enable();
    }
}

void interrupt_init()
{
    /* init led */
    gpio_function_en(LED2);
    gpio_output_en(LED2);
    gpio_set_low_level(LED2);

    // gpio_set_high_level(LED2);
    plic_preempt_feature_en();
    core_interrupt_enable();
    plic_set_priority(IRQ1_SYSTIMER, IRQ_PRI_LEV3);
    plic_set_priority(IRQ3_TIMER1, IRQ_PRI_LEV2);

    /******** timer1 init********/
    plic_interrupt_enable(IRQ3_TIMER1);
    timer_set_init_tick(TIMER1, 0);
    timer_set_mode(TIMER1, TIMER_MODE_SYSCLK);
    timer_set_cap_tick(TIMER1, sys_clk.pclk * 7500); // 7.5ms
    timer_start(TIMER1);
}


void audio_codec_adc_enable(int en)
{
    BM_SET(reg_audio_codec_dac_ctr, FLD_AUDIO_CODEC_DAC_SOFT_MUTE); // dac mute

    if (en) {

#if CODEC_DAC_DCDC_1V8_EN
        //			analog_write_reg8 (0x02, 0xc4);		//flash 2.8V trim
        audio_set_codec_supply(CODEC_2P8V);
#endif

        BM_SET(reg_audio_codec_adc12_ctr, FLD_AUDIO_CODEC_ADC12_SOFT_MUTE); /*adc mute*/

		BM_CLR(reg_audio_codec_adc12_ctr, FLD_AUDIO_CODEC_ADC1_SB | FLD_AUDIO_CODEC_ADC2_SB); /*active adc1 and adc2  channel*/
        BM_CLR(reg_audio_codec_adc2_ctr, FLD_AUDIO_CODEC_ADC12_SB);
        BM_CLR(reg_audio_codec_vic_ctr, FLD_AUDIO_CODEC_SB | FLD_AUDIO_CODEC_SB_ANALOG |
                                            FLD_AUDIO_CODEC_SLEEP_ANALOG); /*disable sleep mode ,disable sb_analog,disable global standby*/
        BM_CLR(reg_audio_codec_mic1_ctr, FLD_AUDIO_CODEC_MICBIAS1_SB);

        BM_CLR(reg_audio_codec_adc12_ctr, FLD_AUDIO_CODEC_ADC12_SOFT_MUTE); /*adc unmute*/
    } else {

        /*******************************************/
        BM_SET(reg_audio_codec_adc12_ctr, FLD_AUDIO_CODEC_ADC12_SOFT_MUTE); /*adc mute*/

        BM_SET(reg_audio_codec_mic1_ctr, FLD_AUDIO_CODEC_MICBIAS1_SB);
        BM_SET(reg_audio_codec_adc2_ctr, FLD_AUDIO_CODEC_ADC12_SB);
        BM_SET(reg_audio_codec_adc12_ctr, FLD_AUDIO_CODEC_ADC1_SB | FLD_AUDIO_CODEC_ADC2_SB);

        BM_CLR(reg_audio_codec_adc12_ctr, FLD_AUDIO_CODEC_ADC12_SOFT_MUTE);

#if CODEC_DAC_DCDC_1V8_EN
        //		analog_write_reg8 (0x02, 0x44);		//flash/DAC 1.8V trim
        audio_set_codec_supply(CODEC_1P8V);
#endif
    }

    BM_CLR(reg_audio_codec_dac_ctr, FLD_AUDIO_CODEC_DAC_SOFT_MUTE); // dac unmute
}

void user_init()
{
#if (CHIP_VER_A0 == CHIP_VER)
    audio_set_codec_supply(CODEC_2P8V);
#endif
    gpio_function_en(LED1 | LED2 | LED3 | LED4);
    gpio_output_en(LED1 | LED2 | LED3 | LED4);
#if (AUDIO_MODE == LINEIN_TO_LINEOUT)
    audio_init(LINE_IN_TO_BUF_TO_LINE_OUT, AUDIO_16K, MONO_BIT_16);
    audio_rx_dma_chain_init(DMA2, (unsigned short *)AUDIO_BUFF, AUDIO_BUFF_SIZE);
    audio_tx_dma_chain_init(DMA3, (unsigned short *)AUDIO_BUFF, AUDIO_BUFF_SIZE);

#elif (AUDIO_MODE == AMIC_TO_LINEOUT)

// ! 初始化
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

    audio_init(AMIC_IN_TO_BUF_TO_LINE_OUT, AUDIO_16K, STEREO_BIT_16);
    // audio_rx_dma_chain_init(DMA2, (unsigned short *)AUDIO_BUFF, AUDIO_BUFF_SIZE);
    // audio_tx_dma_chain_init(DMA3, (unsigned short *)AUDIO_BUFF, AUDIO_BUFF_SIZE);
    interrupt_init();
	audio_rx_dma_chain_init(DMA2, (unsigned short *)audio_rx, AUDIO_BUFF_SIZE);
	audio_tx_dma_chain_init(DMA3, (unsigned short *)audio_tx, AUDIO_BUFF_SIZE);
	// ! 定义ADC寄存器的HPF
    audio_set_codec_adc_wnf(CODEC_ADC_WNF_MODE3); //HFP
#elif (AUDIO_MODE == DMIC_TO_LINEOUT)
    /* this branch */
    /* init state */
    drc_init(&g_state);
    audio_set_dmic_pin(DMIC_GROUPD_D4_DAT_D5_D6_CLK);
    // audio_init(DMIC_IN_TO_BUF_TO_LINE_OUT, AUDIO_16K, MONO_BIT_16);
    audio_init(DMIC_IN_TO_BUF_TO_LINE_OUT, AUDIO_16K, STEREO_BIT_16);
    // audio_rx_dma_chain_init(DMA2, (unsigned short *)AUDIO_BUFF, AUDIO_BUFF_SIZE);
    // audio_tx_dma_chain_init(DMA3, (unsigned short *)AUDIO_BUFF, AUDIO_BUFF_SIZE);
    interrupt_init();
    audio_rx_dma_chain_init(DMA2, (unsigned short *)audio_rx, AUDIO_BUFF_SIZE);
    audio_tx_dma_chain_init(DMA3, (unsigned short *)audio_tx, AUDIO_BUFF_SIZE);

#elif (AUDIO_MODE == BUFFER_TO_LINEOUT)
    audio_init(BUF_TO_LINE_OUT, AUDIO_44EP1K, MONO_BIT_16);
    audio_tx_dma_config(DMA3, (unsigned short *)(&sin_44K1_d1[0]), MIC_BUFFER_SIZE,
                        &tx_dma_list_config[0]);
    audio_tx_dma_add_list_element(&tx_dma_list_config[0], &tx_dma_list_config[1],
                                  (unsigned short *)(&sin_44K1_d2[0]), MIC_BUFFER_SIZE);
    audio_tx_dma_add_list_element(&tx_dma_list_config[1], &tx_dma_list_config[0],
                                  (unsigned short *)(&sin_44K1_d1[0]), MIC_BUFFER_SIZE);
    audio_tx_dma_en();

#elif (AUDIO_MODE == FLASH_TO_LINEOUT)
    flash_read_page(0x8000, FLASH_16k_SIZE, (unsigned char *)flash_read_16K_buff);
    flash_read_page(0xe000, FLASH_48k_SIZE, (unsigned char *)flash_read_48K_buff);
    audio_init(BUF_TO_LINE_OUT, AUDIO_48K, MONO_BIT_16);
    audio_tx_dma_chain_init(DMA3, (unsigned short *)audio_buff, AUIDO_BUFF_SIZE);
#elif (AUDIO_MODE == EXT_CODEC_LINEIN_LINEOUT)
    /* WM8731 for demo*/
    audio_i2s_init(PWM_PWM0_PB4, I2C_GPIO_SDA_B3, I2C_GPIO_SCL_B2);
    audio_rx_dma_chain_init(DMA0, (unsigned short *)AUDIO_BUFF, AUDIO_BUFF_SIZE);
    audio_tx_dma_chain_init(DMA1, (unsigned short *)AUDIO_BUFF, AUDIO_BUFF_SIZE);

#endif
}

#if (AUDIO_MODE == FLASH_TO_LINEOUT)
_attribute_ram_code_sec_ void audio_data_fifo(unsigned char *pdata, unsigned int buff_len)
{
    unsigned short unused_buff;
    tx_rptr_out = ((audio_get_tx_dma_rptr(DMA3) - (unsigned int)audio_buff));
    if ((tx_wptr_out & (AUIDO_BUFF_SIZE - 1)) > tx_rptr_out) {
        unused_buff = AUIDO_BUFF_SIZE - (tx_wptr_out & (AUIDO_BUFF_SIZE - 1)) + tx_rptr_out;
    } else {
        unused_buff = tx_rptr_out - tx_wptr_out;
    }

    if (unused_buff > AUIDO_THD_SIZE) {
        gpio_toggle(LED3);
        memcpy(audio_buff + tx_wptr_out, pdata + flash_rptr, AUIDO_THD_SIZE);
        tx_wptr_out = (tx_wptr_out + AUIDO_THD_SIZE) & (AUIDO_BUFF_SIZE - 1);
        flash_rptr = (flash_rptr + AUIDO_THD_SIZE);
        if (flash_rptr >= (buff_len)) {
            flash_rptr = 0;
        }
    }
}
#endif

/////////////////////////////////////////////////////////////////////
// main loop flow
/////////////////////////////////////////////////////////////////////
void main_loop(void)
{
#if (AUDIO_MODE == FLASH_TO_LINEOUT)
#if 1
    audio_data_fifo(flash_read_48K_buff, FLASH_48k_SIZE);
#endif
    /////////////////////adc  power on/down test /////////////////////////////////////////
#if 0
    audio_data_fifo(flash_read_48K_buff,FLASH_48k_SIZE);
    if(clock_time_exceed(t,4000000))
    {
          t = stimer_get_tick()|1;
         gpio_toggle(LED4);
         ex_cnt++;
         if(ex_cnt&1)
          {
                 audio_codec_dac_power_down();

          }
          else
          {
                audio_codec_dac_power_on();

          }

      }
#endif
    /////////////////////exchange dac sample rate  test
    ////////////////////////////////////////////
#if 0
    if(swith==0)
    {
        audio_data_fifo(flash_read_48K_buff,FLASH_48k_SIZE);
    }
    else

    {
        audio_data_fifo(flash_read_16K_buff,FLASH_16k_SIZE);
    }

    if(clock_time_exceed(t,3000000))
{
    t = stimer_get_tick()|1;
    gpio_toggle(LED4);
    ex_cnt++;
    if(ex_cnt&1)
    {
        audio_pause_out_path();
        audio_change_sample_rate(AUDIO_16K);
        memset(audio_buff,0,AUIDO_BUFF_SIZE);//Clear data that does not match the dac sampling rate
        audio_resume_out_path();
        swith=1;
        flash_rptr=0;
    }
    else
    {
        audio_pause_out_path();
        audio_change_sample_rate(AUDIO_48K);
        memset(audio_buff,0,AUIDO_BUFF_SIZE);//Clear data that does not match the dac sampling rate
        audio_resume_out_path();
        swith=0;
        flash_rptr=0;
    }

}
#endif
#endif
}
