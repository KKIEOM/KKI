/*
 * NanoQplus3, Operating System for small IoT embedded devices.
 * Copyright (C) 2017  Electronics and Telecommunications Research Institute (ETRI)
 *
 *     This file is part of NanoQplus3.
 *
 *     NanoQplus3 is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU Lesser General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     NanoQplus3 is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU Lesser General Public License for more details.
 *
 *     You should have received a copy of the GNU Lesser General Public License
 *     along with NanoQplus3.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file cc1200_hal.c
 *
 * CC1200 Hardware Abstraction Layer for "WPAN_DEV_M"
 *
 * @author Haeyong Kim (ETRI)
 * @date 2014. 7. 17.
 */

#include <string.h>

#define NDEBUG
#include "kconf.h"
#include "nos_debug.h"

#ifdef CC1200_M
#include "platform.h"
#include "critical_section.h"
#include "cc120x_const.h"
#include "cc1200_hal.h"
#include "cc1200_lib.h"
#include "cc1200_interface.h"


// TX done or Fail (channel busy)
volatile int cc1200_tx_result;

// To roll back TXFIFO pointer (for retransmission)
uint8_t cc1200_txfifo_read_ptr;

// CS THR defined by USER (CC1200_config)
int8_t  carrier_sense_threshold;

// LQI is measured whenever RX frame
volatile uint8_t cc1200_lqi;
volatile int8_t cc1200_rssi;

extern uint8_t calibration_complete;
extern uint8_t fs_chp_cal[28], fs_vco2_cal[28], fs_vco4_cal[28];
// Automatic implementation by "SmartRF Studio 7"
extern uint8_t cc1200_user_config_4_8(uint8_t slot);
extern uint8_t cc1200_user_config_38_4(uint8_t slot);
extern uint8_t cc1200_user_config_150(uint8_t slot);
extern uint8_t cc1200_user_config_50(uint8_t slot);
extern uint8_t cc1200_user_config_50_4gfsk(uint8_t slot);
extern uint8_t cc1200_user_config_100(uint8_t slot);
extern uint8_t cc1200_user_config_500(uint8_t slot);
extern uint8_t cc1200_user_config_250(uint8_t slot);
extern uint8_t cc1200_user_config_400_4gfsk(uint8_t slot);

int cc1200_init(uint8_t slot)
{
    uint8_t part_ver;

    NOS_DEBUG_START;
    /* MCU <--> mg2420 Interface Init. SPI, DMA, GPIO, EXTI*/
    cc1200_interface_init(slot);
    nos_delay_us(100);

    /* Reset CC1200, restore default configurations*/
    cc1200_reset(slot);
    nos_delay_us(100);

    /* SPI test */
    cc1200_get_register(0, CC120X_PARTVERSION, &part_ver);
    NOS_DEBUG_PRINTF(("CC1200 Part Version: %x", part_ver));

    /* CC1120 USER configurations by Smart RF Studio*/
    //carrier_sense_threshold = cc1200_user_config_50(slot);
    //carrier_sense_threshold = cc1200_user_config_500(slot);
    //carrier_sense_threshold = cc1200_user_config_400_4gfsk(slot);
    carrier_sense_threshold = cc1200_user_config_250(slot);
    //carrier_sense_threshold = cc1200_user_config_150(slot);
    //carrier_sense_threshold = cc1200_user_config_50_4gfsk(slot);
    //carrier_sense_threshold = cc1200_user_config_50(slot);

    /* CC1200 Calibration */
    cc1200_manual_calib(slot);
    //cc1200_predefine_calibration(slot);

    /* CC1200 NanoQplus configurations */
    // GPIO out configurations  (GPIO3 is working??)
    cc1200_disable_intr(slot, WPAN_DEV_INTR_ALL);
    cc1200_set_register(slot, CC120X_IOCFG0, GPIOx_CFG_MARC_MCU_WAKEUP);
    cc1200_set_register(slot, CC120X_IOCFG1, GPIO1_CFG_CCA_STATUS);
    cc1200_set_register(slot, CC120X_IOCFG2, GPIOx_CFG_PKT_SYNC_RXTX);
#if CONTINUOUS_SWITCH
    cc1200_set_register(slot, CC120X_IOCFG3, GPIOx_CFG_ANTENNA_SELECT);
#endif
    //cc1200_set_register(slot, CC120X_IOCFG2, GPIO2_CFG_TXONCCA_DONE);
    //cc1200_set_register(slot, CC120X_IOCFG0, GPIO0_CFG_TXONCCA_FAILED);
    //cc1200_set_register(slot, CC120X_IOCFG2, GPIOx_CFG_CARRIER_SENSE);

    cc1200_clear_intr_flag(slot, WPAN_DEV_INTR_ALL);
    cc1200_enable_intr(slot, WPAN_DEV_INTR_ALL);

    // TX end, RX end state
    cc1200_set_register(slot, CC120X_RFEND_CFG1,     RXOFF_MODE__RX|RX_TIME__DISABLE_TIMEOUT|RX_TIME_QUAL__CONTINUE_RX_ON_TIMEOUT_IF_PKT_REACHED);
#if CONTINUOUS_SWITCH
    //cc1200_set_register(slot, CC120X_RFEND_CFG0,     TXOFF_MODE__RX | ANT_DIV_RX_TERM_CFG__CONT_SWITCH_ANT_DIV_ON_PQT);
    cc1200_set_register(slot, CC120X_RFEND_CFG0,     TXOFF_MODE__RX | ANT_DIV_RX_TERM_CFG__CONT_SWITCH_ANT_DIV_ON_CS);
#else
    cc1200_set_register(slot, CC120X_RFEND_CFG0,     TXOFF_MODE__RX);
#endif
    //cc1200_set_register(slot, CC120X_RFEND_CFG0,     TXOFF_MODE__RX|TERM_ON_BAD_PACKET_EN__ENABLED);

    // RX method
    //cc1200_set_register(slot, CC120X_PKT_CFG1,       (1<<7)|APPEND_STATUS__ENABLED|CRC__ENABLED_CRC16_INIT_ONES);        // FEC en, CRC en, append RXFIFO(CRC, RSSI, LQI)
    cc1200_set_register(slot, CC120X_PKT_CFG1,       APPEND_STATUS__ENABLED|CRC__ENABLED_CRC16_INIT_ONES);        // CRC en, append RXFIFO(CRC, RSSI, LQI)
    cc1200_set_register(slot, CC120X_FIFO_CFG,       CRC_AUTOFLUSH__ENABLED);    //off --> on: SFRX strobe must be issued first

    // TX method
    cc1200_set_register(slot, CC120X_PKT_LEN,        128);        // max length is 128 (including length byte)
    cc1200_set_register(slot, CC120X_PKT_CFG0,       LENGTH__CONFIGURED_BY_FIRST_BYTE_AFTER_SYNCWORD);        // variable packet length mode. first byte is length.
#if CONTINUOUS_SWITCH
    cc1200_set_register(slot, CC120X_PKT_CFG2,       CCA_MODE__UNLESS_RXING);
#else
    cc1200_set_register(slot, CC120X_PKT_CFG2,       CCA_MODE__RSSI_BELOW_THR_UNLESS_NOT_RXING);
#endif
    //cc1200_set_register(slot, CC120X_PKT_CFG2,       CCA_MODE__ALWAYS);
    //cc1200_set_register(slot, CC120X_PKT_CFG2,       CCA_MODE__RSSI_BELOW_THR_AND_LBT_REQ_MET);   // If channel is busy, TX will wait forever (will not return)

    return EXIT_SUCCESS;
}



int cc1200_set_channel(uint8_t slot, uint8_t channel)
{
    uint32_t xosc = 400;
    uint32_t lo_divider = 4;
    uint32_t frequency, set_frequency, f_vco;

    NOS_DEBUG_START;
    cc1200_rx_enable(slot, FALSE);
    switch (channel)
    {
        case 1:
            frequency = CC1200_CHANNEL_1;
        break;
        case 2:
            frequency = CC1200_CHANNEL_2;
        break;
        case 3:
            frequency = CC1200_CHANNEL_3;
        break;
        case 4:
            frequency = CC1200_CHANNEL_4;
        break;
        case 5:
            frequency = CC1200_CHANNEL_5;
        break;
        case 6:
            frequency = CC1200_CHANNEL_6;
        break;
        case 7:
            frequency = CC1200_CHANNEL_7;
        break;
        case 8:
            frequency = CC1200_CHANNEL_8;
        break;
        case 9:
            frequency = CC1200_CHANNEL_9;
        break;
        case 10:
            frequency = CC1200_CHANNEL_10;
        break;
        case 11:
            frequency = CC1200_CHANNEL_11;
        break;
        case 12:
            frequency = CC1200_CHANNEL_12;
        break;
#if 0
        case 13:
            frequency = CC1200_CHANNEL_13;
        break;
        case 14:
            frequency = CC1200_CHANNEL_14;
        break;
        case 15:
            frequency = CC1200_CHANNEL_15;
        break;
        case 16:
            frequency = CC1200_CHANNEL_16;
        break;
        case 17:
            frequency = CC1200_CHANNEL_17;
        break;
        case 18:
            frequency = CC1200_CHANNEL_18;
        break;
        case 19:
            frequency = CC1200_CHANNEL_19;
        break;
        case 20:
            frequency = CC1200_CHANNEL_20;
        break;
#endif
    #if FIELD_NORWAY
        #if 0
        case 21:
            frequency = CC1200_CHANNEL_21;
        break;
        case 22:
            frequency = CC1200_CHANNEL_22;
        break;
        case 23:
            frequency = CC1200_CHANNEL_23;
        break;
        case 24:
            frequency = CC1200_CHANNEL_24;
        break;
        case 25:
            frequency = CC1200_CHANNEL_25;
        break;
        case 26:
            frequency = CC1200_CHANNEL_26;
        break;
        case 27:
            frequency = CC1200_CHANNEL_27;
        break;
        case 28:
            frequency = CC1200_CHANNEL_28;
        break;
        #endif
    #endif
        default:
        {
            printf("\r\nInvalid Channel!:%d", channel);
            cc1200_rx_enable(slot, TRUE);
            return EXIT_FAIL;
        }
    }
    //frequency += 1;
    f_vco = frequency*lo_divider;
    set_frequency = (f_vco*65536)/xosc;
    cc1200_set_register(slot, CC120X_FREQ2, (uint8_t)(set_frequency>>16));    //Frequency Configuration [23:16]
    cc1200_set_register(slot, CC120X_FREQ1, (uint8_t)(set_frequency>>8));    //Frequency Configuration [15:8]
    cc1200_set_register(slot, CC120X_FREQ0, (uint8_t)(set_frequency));    //Frequency Configuration [15:0]

    if(calibration_complete)
    {
        cc1200_set_register(slot, CC120X_FS_VCO4, fs_vco4_cal[channel-1]);
        cc1200_set_register(slot, CC120X_FS_VCO2, fs_vco2_cal[channel-1]);
        cc1200_set_register(slot, CC120X_FS_CHP, fs_chp_cal[channel-1]);
    }

    wpan_dev[slot].phyCurrentChannel = channel;
    cc1200_rx_enable(slot, TRUE);
    return EXIT_SUCCESS;
}


uint8_t cc1200_get_channel(uint8_t slot)
{
    NOS_DEBUG_START;
    return wpan_dev[slot].phyCurrentChannel;
}


int cc1200_write_frame(uint8_t slot, uint8_t *ppdu_p)
{
    NOS_DEBUG_START;
    if (ppdu_p[0] < 2)
    {
        NOS_ERROR_PRINTF(("too short"));
        return EXIT_INVALID_ARGS;
    }

    //@haekim NOS_ENTER_CRITICAL_SECTION();
    cc1200_txfifo_push_single(slot, ppdu_p[0]-2);   // subtract CRC-16 (inserted automatically)
    cc1200_txfifo_push_burst(slot, ppdu_p+1, ppdu_p[0]-2);
    //@haekim NOS_EXIT_CRITICAL_SECTION();
    NOS_DEBUG_END;
    return ppdu_p[0];
}

//zine
bool cc1200_cca(uint8_t slot)
{
#if 0
#if CONTINUOUS_SWITCH
    uint8_t count = 0;
#endif
    uint16_t cca_us;
    NOS_DEBUG_START;
    cca_us = wpan_dev[slot].phyCCADuration*wpan_dev[slot].symbol_period;   //8*14 = 132us
    for (int i=0; i<= cca_us/10; i++)
    {

        if (!cc1200_read_gpio(slot,1))
        {
        #if CONTINUOUS_SWITCH
            count++;
            if(count > 2)
        #endif
                return FALSE;
        }
        state = cc1200_check_and_get_state(slot);
        if (state != CC120X_STATE_RX)
            NOS_ERROR_PRINTF(("%i", state));
        nos_delay_us(9);
    }
    return TRUE;
#else
    // if there is an error in RXFIFO, getting RSSI failed forever.
    uint8_t reg_val;
    cc1200_change_state(slot, CC120X_STATE_RX);  // resolve RXFIFO_OVERFLOW
    for (int i=0; i<wpan_dev[slot].phyCCADuration; i++)
    {
        cc1200_get_register(slot, CC120X_RSSI0, &reg_val);
        if (reg_val & 0x02) //wait until CARRIER_SENSE_VALID is set to 1
        {
            if (reg_val & 0x04) // Check CARRIER_SENSE. carrier detected or RXFIFO_ERROR(overflow)
                return FALSE;
            else
                return TRUE;
        }
        nos_delay_us(wpan_dev[slot].symbol_period);
    }
    NOS_DEBUG_PRINTF((" >> CCA failed because of RXFIFO_ERROR."));
    cc1200_change_state(slot, CC120X_STATE_RX);  // resolve RXFIFO_OVERFLOW
    return FALSE;
#endif
}



// if there is an error in RXFIFO, getting RSSI failed forever.
int8_t cc1200_get_rssi(uint8_t slot)
{
    uint8_t reg_val;
    int8_t rssi;
    int i;

    NOS_DEBUG_START;
    for (i=0; i<100; i++)
    {
        cc1200_get_register(slot, CC120X_RSSI0, &reg_val);
        if (reg_val & 0x01) //wait until RSSI_VALID is set to 1
        {
            cc1200_get_register(slot, CC120X_RSSI1, (uint8_t*)&rssi);

            if (((int32_t)(rssi)+CC1200_RSSI_OFFSET) < -128)
                return (-128);
            return rssi + CC1200_RSSI_OFFSET;
        }
        nos_delay_us(wpan_dev[slot].symbol_period);
    }
    NOS_ERROR_PRINTF(("ERROR(cc1200_get_rssi): You should not see this message\n\n\r"));
    cc1200_change_state(slot, CC120X_STATE_RX);  // resolve RXFIFO_OVERFLOW
    nos_delay_us(wpan_dev[slot].symbol_period);
    cc1200_get_register(slot, CC120X_RSSI1, (uint8_t*)&rssi);
    if (((int32_t)(rssi)+CC1200_RSSI_OFFSET) < -128)
        return (-128);
    return rssi + CC1200_RSSI_OFFSET;
#if 0
    do
    {
        cc1200_get_register(slot, CC120X_RSSI0, &reg_val);
    } while (!(reg_val & 0x01)); //wait until RSSI_VALID is set to 1
    cc1200_get_register(slot, CC120X_RSSI1, (uint8_t*)&rssi);
    return rssi;
#endif
}


/*
sensitivity for the PHY.  (refer. specification page 13)
-103, -110, ..., -127 dBm
ed 0:  (sensitivity +10)
...       (ed must be linear to RSSI)
ed 40: (max)
*/
uint8_t cc1200_get_receiver_ed(uint8_t slot)
{
    int16_t ed = 0;
    for (int i=0; i<wpan_dev[slot].phyCCADuration; i++)
    {
        ed += cc1200_get_rssi(slot);
        nos_delay_us(wpan_dev[slot].symbol_period);
    }
    ed += cc1200_get_rssi(slot);

    // ed 0(calm)   ed 20(threshold)  ed 40(noisy)
    ed = ed/9 -carrier_sense_threshold +20;
    if (ed < 0)
        ed = 0;
    else if (ed > 40)
        ed = 40;
    return ed;
}


WPAN_DEV_INTR_T cc1200_check_intr_src(uint8_t slot)
{
    uint8_t marc;
    NOS_DEBUG_START;

    cc1200_get_register(slot, CC120X_MARC_STATUS1, &marc);
    if (marc == MARC_STATUS_OUT__RX_FINISHED)
    {
        NOS_DEBUG_NOTIFY;
        cc1200_tx_result = WPAN_DEV_INTR_RX_END;
    }
    else if (marc == MARC_STATUS_OUT__TX_FINISHED)
    {
        NOS_DEBUG_NOTIFY;
        cc1200_tx_result = WPAN_DEV_INTR_TX_END;
    }
    else if (marc == MARC_STATUS_OUT__TX_ON_CCA_FAILED)
    {
        NOS_DEBUG_NOTIFY;
        cc1200_tx_result = WPAN_DEV_INTR_TX_FAIL;
    }
    else if (marc == MARC_STATUS_OUT__TXFIFO_OVERFLOW ||
             marc == MARC_STATUS_OUT__TXFIFO_UNDERFLOW)
    {
        NOS_DEBUG_NOTIFY;
        cc1200_tx_result = WPAN_DEV_INTR_TXFIFO_ERROR;
    }
    else if ((marc == MARC_STATUS_OUT__RXFIFO_OVERFLOW) ||
            (marc == MARC_STATUS_OUT__RXFIFO_UNDERFLOW))
    {
        NOS_DEBUG_NOTIFY;
        cc1200_tx_result = WPAN_DEV_INTR_RXFIFO_ERROR;
    }
    else if ((marc == MARC_STATUS_OUT__NO_FAILURE))
    {
        NOS_DEBUG_NOTIFY;
        cc1200_tx_result = WPAN_DEV_INTR_NONE;
    }
    else
    {
        NOS_DEBUG_PRINTF(("RX error (MARC_STATUS1: %x)", marc));
        cc1200_tx_result = WPAN_DEV_INTR_NONE;
    }
    return (WPAN_DEV_INTR_T)cc1200_tx_result;
}

uint32_t time_test_start;
uint32_t time_test_end;

int cc1200_tx_request(uint8_t slot, uint8_t seq, bool ack_req)
{
    uint8_t state, txfifo_len, frame_len;


    NOS_DEBUG_START;
    // wait until ongoing TX is done.
    // 3416 us
    state = cc1200_check_and_get_state(slot);
    // 3420 us
    if ((state == CC120X_STATE_TX) || (state == CC120X_STATE_FSTXON))
    {
        NOS_ERROR_PRINTF(("already in TX state"));
        return EXIT_ERROR;
    }

    // store rollback info for ACK requested TX. also used to check frame len even if ack is not requested
    cc1200_get_register(slot, CC120X_TXFIRST, &cc1200_txfifo_read_ptr);
    // 3434 us




    // Check CCA
    if (cc1200_cca(slot) == FALSE)
    {
        return EXIT_FAILURE;   // channel is not clear
    }



    // prepare to send (no more RX)
    // 3453 us
    cc1200_change_state(slot, CC120X_STATE_FSTXON); // +791~818 us
    // 4244, 4249 us (for EB transmit: 791 us)

    /*** atomic TX process: nuri_soria(about 33us)***/
    NOS_ENTER_CRITICAL_SECTION();
    // check if TXFIFO is OK.  if ACK has been transmitted while waiting, TXFIFO must not be ok.
    frame_len = cc1200_fifo_direct_read(slot, cc1200_txfifo_read_ptr);  // get the length of frame
    cc1200_get_register(slot, CC120X_NUM_TXBYTES, &txfifo_len); // get total TX fifo length
    if (frame_len >= txfifo_len) // usually, txfifo len = frame len +1
    {
        NOS_EXIT_CRITICAL_SECTION();

        // prepare to receive
        cc1200_change_state(slot, CC120X_STATE_RX);
        //cc1200_flush_txfifo(slot);    // will be called by wpan_dev_tx
        NOS_ERROR_PRINTF(("TXFIFO(%u) < Frame(%u)", txfifo_len, frame_len));
        return EXIT_ERROR;
    }

    // TX request to Modem
    cc1200_tx_result = 0xFF;    // 0xFF means TX started, but has not been done yet.

    //4271 us
    cc1200_command_strobe(slot, CC120X_STX);
    //cc1200_change_state(slot, CC120X_STATE_TX);
    NOS_EXIT_CRITICAL_SECTION();
    // 4276 us

    /*** END of atomic TX process***/
    time_test_start = nos_timer_get_time(WPAN_MAC_TIMER);

    // Transition time from RX state to TX state takes 69us.
    nos_delay_us(500);
    #if 0
    for (int i=0; i<150; i++)
    {
        if (cc1200_get_chip_status(slot) == CC120X_STATE_TX)
            break;
        nos_delay_us(1);
    }
    #endif

    // wait until ongoing TX is done. (150kbps:52.08us/byte, 250kbps: 31.25us/byte)
    uint16_t loop = 0;
    while (cc1200_check_and_get_state(slot) == CC120X_STATE_TX) // Do not use cc1200_is_busy(). It returns FALSE too early.
    {
        if (loop++ > 10000)
        {
            cc1200_change_state(slot, CC120X_STATE_RX);
            //cc1200_flush_txfifo(slot);
            NOS_ERROR_PRINTF(("TX TIMEOUT (underflow?)"));
            return EXIT_TIMEOUT_ERROR;  // txfifo_error
        }
        nos_delay_us(10);
    }
    // 5767 us (+1491 us)

    time_test_end = nos_timer_get_time(WPAN_MAC_TIMER);

    //printf("\n\rtime:%u", nos_timer_get_time(WPAN_MAC_TIMER));  //4552us

    // prepare to receive. after TX, CC1200 state will be in RX state automatically
    #ifdef CC1190_M
    cc1190_rx_gain_mode(slot);  // +380 us
    #endif
    // 6148 us (+381 us)

    // If ISR was not called
    if(cc1200_tx_result == 0xFF)
    {
        cc1200_check_intr_src(slot);
    }

    // We expect cc1200_check_intr_src() has been called by ISR.
    if (cc1200_tx_result == WPAN_DEV_INTR_TX_END)
    {
        // 6162 us
        return EXIT_SUCCESS;
    }
    else if (cc1200_tx_result == WPAN_DEV_INTR_TX_FAIL) //CCA fail
    {
        return EXIT_FAILURE;
    }
    else
    {
        if (cc1200_tx_result == WPAN_DEV_INTR_TXFIFO_ERROR)
        {
            cc1200_flush_txfifo(slot);
        }
        else if (cc1200_tx_result == WPAN_DEV_INTR_RXFIFO_ERROR)
        {
            cc1200_flush_rxfifo(slot);
        }
        else if (cc1200_tx_result == WPAN_DEV_INTR_RX_END)
        {
            // TX end IRQ might be overwritten by ACK RX end IRQ?
        }
        else if (cc1200_tx_result == WPAN_DEV_INTR_NONE)
        {
            // no failure --> means what???
        }
        // IRQ was not executed. Need to fix something
        cc1200_change_state(slot, CC120X_STATE_RX);
        NOS_ERROR_PRINTF(("Unexpected INTR(%u)", cc1200_tx_result));
        return EXIT_ERROR;
    }
}



int cc1200_ack_transmit(uint8_t slot, uint8_t *ack_ppdu_p)
{
    //uint8_t prev_fifo_data[4], status, tx_first, tx_last;

    NOS_DEBUG_START;
    ack_ppdu_p[0] = 0x03;   // CC1200 is not IEEE 802.15.4 compatible (no FCS). need to fix length 5-->3
    // Flush TXFIFO. refer cc1200_flush_txfifo(slot);
    cc1200_change_state(slot, CC120X_STATE_IDLE);
    cc1200_command_strobe(slot, CC120X_SFTX);   // must be issued in IDLE or ERR state
    // make RF ready to send ACK quickly.
    //cc1200_change_state(slot, CC120X_STATE_FSTXON);
    cc1200_command_strobe(slot, CC120X_SFSTXON);

    #ifdef CC1190_M
    cc1190_tx_gain_mode(slot);
    #endif

    cc1200_txfifo_push_burst(slot, ack_ppdu_p, 4);

    //cc1200_change_state(slot, CC120X_STATE_TX);
    cc1200_command_strobe(slot, CC120X_STX);

#if 1
    // Transition time from RX state to TX state takes 69us.
    // If CCA fail, transition will not occur.
    for (int i=0; i<150; i++)
    {
        if (cc1200_get_chip_status(slot) == CC120X_STATE_TX)
            break;
        nos_delay_us(1);
    }

    // wait until ongoing TX is done. (150kbps:52.08us/byte, 250kbps: 31.25us/byte)
    uint16_t loop = 0;
    while (cc1200_check_and_get_state(slot) == CC120X_STATE_TX)
    {
        if (loop++ > 500)
        {
            cc1200_flush_txfifo(slot);
            cc1200_change_state(slot, CC120X_STATE_RX);
            NOS_ERROR_PRINTF(("TX loop (underflow?)"));
            return EXIT_UNKNOWN_ERROR;  // txfifo_error
        }
        nos_delay_us(10);
        //putchar('#');    // Do not use cc1200_is_busy(). It returns FALSE too early.
    }
#endif
    #ifdef CC1190_M
    cc1190_rx_gain_mode(slot);//380us
    #endif
    return EXIT_SUCCESS;

#if 0
    // change CCA configuration
    //cc1120_set_register(slot, CC112X_PKT_CFG2, CCA_MODE__ALWAYS);

    // save previous txfifo data
    cc1120_get_register(slot, CC112X_TXFIRST, &tx_first);
    cc1120_get_register(slot, CC112X_TXLAST, &tx_last);
    cc1120_fifo_direct_burst_access(slot, 0x00, TRUE, prev_fifo_data, 4);

    // push ack frame into 0x00~0x03
    cc1120_set_register(slot, CC112X_TXFIRST, 0);
    cc1120_set_register(slot, CC112X_TXLAST, 0);


    ....

    // Restore CCA configuration
    //cc1200_set_register(slot, CC120X_PKT_CFG2, CCA_MODE__RSSI_BELOW_THR_UNLESS_NOT_RXING);

    cc1200_fifo_direct_burst_access(slot, 0x00, FALSE, prev_fifo_data, 4);
    cc1200_set_register(slot, CC120X_TXFIRST, tx_first);
    cc1200_set_register(slot, CC120X_TXLAST, tx_last);

    if (cc1200_get_chip_status(slot) == CC120X_STATE_TXFIFO_ERROR)
    {
        NOS_DEBUG_ERROR;
        printf("\n\n\rThis message must not be seen.\n\n");
        cc1200_flush_txfifo(slot);
        return EXIT_UNKNOWN_ERROR;
    }
    else
        return EXIT_SUCCESS;
#endif
}


int cc1200_txfifo_rollback(uint8_t slot)
{
    NOS_DEBUG_START;
    cc1200_set_register(slot, CC120X_TXFIRST, cc1200_txfifo_read_ptr);
    return EXIT_SUCCESS;
}


bool cc1200_rxfifo_is_empty(uint8_t slot)
{
    uint8_t reg_val;

    cc1200_get_register(slot, CC120X_MODEM_STATUS1, &reg_val);
    if (reg_val & RX_STATUS__RXFIFO_OVERFLOW)
    {
        NOS_ERROR_PRINTF(("rxfifo overflow"));
        cc1200_flush_rxfifo(slot);
        return TRUE;
    }
    else if (reg_val & RX_STATUS__RXFIFO_UNDERFLOW)
    {
        NOS_ERROR_PRINTF(("rxfifo underflow"));
        cc1200_flush_rxfifo(slot);
        return TRUE;
    }
    return (reg_val & RX_STATUS__RXFIFO_EMPTY) ? TRUE : FALSE;
#if 0
        uint8_t rxbyte;
        uint8_t status;
        status = cc1200_get_chip_status(slot);
        if (status == CC120X_STATE_RXFIFO_ERROR)
        {
            NOS_DEBUG_ERROR;
            cc1200_flush_rxfifo(slot);
            return TRUE;
        }
        else if (status == CC120X_STATE_TXFIFO_ERROR)
        {
            NOS_DEBUG_ERROR;
            cc1200_flush_txfifo(slot);
            return TRUE;
        }
        else if (status == CC120X_STATE_FSTXON || status == CC120X_STATE_IDLE)
        {
            NOS_DEBUG_NOTIFY;
           cc1200_rx_enable(slot, TRUE);
        }
        cc1200_get_register(slot, CC120X_NUM_RXBYTES, &rxbyte);
        NOS_DEBUG_END;
        return (rxbyte == 0) ? TRUE : FALSE;
#endif
}


// CC120X_RXLAST stores The last filled byte not an empty byte.
// Refer examples.
// But RXFIFO works well.

// (example1)
// init:                                                  RXFIRST:0, RXLAST:0, RXNUM:0
// 40bytes(length byte:37) on the air : RXFIRST:0, RXLAST:39,  RXNUM:40
// 40bytes(length byte:37) on the air : RXFIRST:0, RXLAST:79,  RXNUM:80
// 40bytes(length byte:37) on the air : RXFIRST:0, RXLAST:119,  RXNUM:120
// pop 40bytes:                                     RXFIRST:40, RXLAST:119,  RXNUM:80
// pop 40bytes:                                     RXFIRST:80, RXLAST:119,  RXNUM:40
// pop 40bytes:                                     RXFIRST:119, RXLAST:119,  RXNUM:0

// (example2)
// init:                                                  RXFIRST:0, RXLAST:0, RXNUM:0
// 40bytes(length byte:37) on the air : RXFIRST:0, RXLAST:39,  RXNUM:40
// 40bytes(length byte:37) on the air : RXFIRST:0, RXLAST:79,  RXNUM:80
// pop 40bytes:                                     RXFIRST:40, RXLAST:79,  RXNUM:40
// pop 40bytes:                                     RXFIRST:79, RXLAST:79,  RXNUM:0
// 40bytes(length byte:37) on the air : RXFIRST:79, RXLAST:118,  RXNUM:40
// pop 40bytes:                                     RXFIRST:118, RXLAST:118,  RXNUM:0
int cc1200_read_frame(uint8_t slot, uint8_t *ppdu_p)
{

    NOS_DEBUG_START;
    cc1200_rxfifo_pop_single(slot, &ppdu_p[0]);
    ppdu_p[0] += 2; // add 'append status length(2)'
    if ((ppdu_p[0]&0x80))   // length > 127
    {
        NOS_ERROR_PRINTF(("len>127 error"));
        return EXIT_ERROR;
    }
    cc1200_rxfifo_pop_burst(slot, ppdu_p+1, ppdu_p[0]);
    cc1200_lqi = ppdu_p[ppdu_p[0]];
    if (!(cc1200_lqi & CC120X_LQI_CRC_OK_BM))
    {
        //CRC check FAIL
        NOS_ERROR_PRINTF(("CRC error"));
        return EXIT_ERROR;
    }
    cc1200_lqi &= CC120X_LQI_EST_BM;
    cc1200_rssi = ppdu_p[ppdu_p[0]-1];
    //printf("\r\nrssi:%d, lqi:%d", cc1200_rssi+CC1200_RSSI_OFFSET, ppdu_p[ppdu_p[0]]&0x3F);
    NOS_DEBUG_END;
    return ppdu_p[0];
}


//If PKT_CFG1.APPEND_STATUS is enabled, the value is automatically added to the last byte appended after the payload.
//The value can also be read from the LQI_VAL register.
//A low value indicates a better link than what a high value does
uint8_t cc1200_get_lqi(uint8_t slot)
{
    #if 0
    // NOT vaild if uses append_status???
    uint8_t val;
    cc1200_get_register(slot, CC120X_LQI_VAL, &val);
    val &= CC120X_LQI_EST_BM;
    return val;
    #else
    return cc1200_lqi;    // CC1200 gives weird lqi.
    //return (uint8_t)((int16_t)cc1200_rssi + 128 + CC1200_RSSI_OFFSET);  //returns RSSI+128
    #endif
}

int8_t cc1200_get_rssi_value(uint8_t slot)
{
    return cc1200_rssi + CC1200_RSSI_OFFSET;
}

/*
 * CC1200 does not have IEEE 802.15.4 compliant address decoding and filtering
 * function. So below functions related to addresses have nothing to do with the
 * something like chip registers.
 */
uint16_t cc1200_pan_id;
int cc1200_set_pan_id(uint8_t slot, uint16_t panid)
{
    NOS_DEBUG_START;
    cc1200_pan_id = panid;
    return EXIT_SUCCESS;
}
uint16_t cc1200_get_pan_id(uint8_t slot)
{
    NOS_DEBUG_START;
    return cc1200_pan_id;
}

uint16_t cc1200_short_addr;
int cc1200_set_short_addr(uint8_t slot, uint16_t short_addr)
{
    NOS_DEBUG_START;
    cc1200_short_addr = short_addr;
    return EXIT_SUCCESS;
}
uint16_t cc1200_get_short_addr(uint8_t slot)
{
    NOS_DEBUG_START;
    return cc1200_short_addr;
}

uint8_t cc1200_ext_addr[8];
int cc1200_set_ext_addr(uint8_t slot, const uint8_t *ext_addr_p)
{
    NOS_DEBUG_START;
    memcpy(cc1200_ext_addr, ext_addr_p, 8);
    return EXIT_SUCCESS;
}
int cc1200_get_ext_addr(uint8_t slot, uint8_t *ext_addr_wp)
{
    NOS_DEBUG_START;
    if (ext_addr_wp == NULL)
    {
        NOS_DEBUG_ERROR;
        return EXIT_INVALID_ARGS;
    }
    else
    {
        memcpy(ext_addr_wp, cc1200_ext_addr, 8);
        NOS_DEBUG_END;
        return EXIT_SUCCESS;
    }
}


int cc1200_set_tx_power(uint8_t slot, int dbm)
{
    NOS_DEBUG_START;
#ifdef CC1190_M
    switch(dbm)
    {
        case -40:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x41);
        break;
        case -24:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x42);
        break;
        case -12:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x44);
        break;
        case -11:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x44);
        break;
        case -6:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x44);
        break;
        case -3:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x44);
        break;
        case 0:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x45);
        break;
        case 1:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x45);
        break;
        case 2:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x45);
        break;
        case 3:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x46);
        break;
        case 4:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x46);
        break;
        case 5:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x47);
        break;
        case 6:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x48);
        break;
        case 7:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x49);
        break;
        case 8:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x4a);
        break;
        case 9:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x4b);
        break;
        case 10:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x4c);
        break;
        case 11:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x4d);
        break;
        case 12:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x4e);
        break;
        case 13:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x4f);
        break;
        case 14:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x51);
        break;
        case 15:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x52);
        break;

        case 17:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x53);
        break;
        case 18:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x55);
        break;
        case 19:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x58);
        break;
        case 20:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x5A);
        break;
        case 21:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x5D);
        break;
        case 22:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x60);
        break;
        case 23:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x63);
        break;
        case 24:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x66);
        break;
        case 25:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x6B);
        break;
        case 26:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x6F);
        break;
        case 27:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x77);
        break;
        default:
            cc1200_set_register(slot, CC120X_PA_CFG1, 0x53);
        break;
    }
#else
    int pa_power_ramp;
    // Refer 44page in CC1200 user guide.
    if (dbm < -16)
        dbm = -16; // min: -16dBm
    pa_power_ramp = (dbm + 18) * 2 -1;
    if (pa_power_ramp > 0x3F)
        pa_power_ramp = 0x3F;   // max: 14dBm (15 dBm in real??)

    pa_power_ramp |= (1 << 6); //PA_CFG2_RESERVED6
    cc1200_set_register(slot, CC120X_PA_CFG1, pa_power_ramp);
#endif
    return EXIT_SUCCESS;
}


int cc1200_rx_enable(uint8_t slot, bool en)
{
    NOS_DEBUG_START;
    if (en)
    {
        cc1200_change_state(slot, CC120X_STATE_RX);
    }
    else
    {
        cc1200_change_state(slot, CC120X_STATE_IDLE);
    }
    return EXIT_SUCCESS;
}


int cc1200_flush_txfifo(uint8_t slot)
{
    NOS_DEBUG_START;
    cc1200_change_state(slot, CC120X_STATE_IDLE);
    cc1200_command_strobe(slot, CC120X_SFTX);   // must be issued in IDLE or ERR state
    cc1200_change_state(slot, CC120X_STATE_RX);
    return EXIT_SUCCESS;
}


int cc1200_flush_rxfifo(uint8_t slot)
{
    NOS_DEBUG_START;
    cc1200_change_state(slot, CC120X_STATE_IDLE);
    cc1200_command_strobe(slot, CC120X_SFRX);   // must be issued in IDLE or ERR state
    cc1200_change_state(slot, CC120X_STATE_RX);
    return EXIT_SUCCESS;
}


const WPAN_DEV_CMD cc1200_cmd =
{
    cc1200_init,
    cc1200_enable_intr,
    cc1200_disable_intr,
    cc1200_clear_intr_flag,
    cc1200_get_intr_flag,
    cc1200_check_intr_src,

    cc1200_rx_enable,
    cc1200_set_tx_power,
    cc1200_set_channel,
    cc1200_get_channel,
    cc1200_set_pan_id,
    cc1200_get_pan_id,
    cc1200_set_short_addr,
    cc1200_get_short_addr,
    cc1200_set_ext_addr,
    cc1200_get_ext_addr,

    /* TX */
    cc1200_flush_txfifo,
    cc1200_write_frame,
    cc1200_tx_request,
    cc1200_txfifo_rollback,
    NULL,
    cc1200_ack_transmit,
    cc1200_cca,
    cc1200_get_receiver_ed,

    /* RX */
    cc1200_flush_rxfifo,
    cc1200_read_frame,
    cc1200_rxfifo_is_empty,
    cc1200_get_lqi,
    cc1200_get_rssi,
    cc1200_get_rssi_value,
    cc1200_is_busy,


    /*Security */
    NULL,
    NULL,
};


#endif //CC1200_M
