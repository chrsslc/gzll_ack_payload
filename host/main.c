/**
 * Copyright (c) 2012 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * This project requires that a device that runs the
 * @ref gzll_device_m_ack_payload_example is used as a counterpart for
 * receiving the data. This can be on either an nRF5x device or an nRF24Lxx device
 * running the \b gzll_device_m_ack_payload example in the nRFgo SDK.
 *
 * This example listens for a packet and sends an ACK
 * when a packet is received. The contents of the first payload byte of
 * the received packet is output on the GPIO Port BUTTONS.
 * The contents of GPIO Port LEDS are sent in the first payload byte (byte 0)
 * of the ACK packet.
 */
#include "nrf_gzll.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_error.h"
#include "nrf_gzll_error.h"
#include "stdlib.h"
#include "string.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SPI_STACK.h"

#define DATASIZE 64
	char String[DATASIZE];
/*****************************************************************************/
/** @name Configuration  */
/*****************************************************************************/
#define PIPE_NUMBER             0  /**< Pipe 0 is used in this example. */
#define TX_PAYLOAD_LENGTH       1  /**< 1-byte payload length is used when transmitting. */
#define CRS_SPI_SLAVE       1  
#define MAXSTACKHEIGHT 10

int stackheight = 0;
#if CRS_SPI_SLAVE == 1
  #include "sdk_config.h"
  #include "nrf_drv_spis.h"
  #include "nrf_gpio.h"
  #include "boards.h"
  #include "app_error.h"
  #include <string.h>

  #define SPIS_INSTANCE 1 /**< SPIS instance index. */
  static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
  #define TEST_STRING "FROMNRF2XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
  static uint8_t       blank_tx_buf[] = TEST_STRING;           /**< TX buffer. */
  static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
  static uint8_t       new_tx_buf[] = TEST_STRING;           /**< TX buffer. */
  static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
  static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

  int messageCount = 0;
  static volatile bool spis_xfer_done; /**< Flag used to indicate that SPIS instance completed the transfer. */
#endif

static uint8_t                  m_data_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; /**< Placeholder for data payload received from host. */
static uint8_t                  m_ack_payload[TX_PAYLOAD_LENGTH];                  /**< Payload to attach to ACK sent to device. */

#if GZLL_TX_STATISTICS
static nrf_gzll_tx_statistics_t m_statistics; /**< Struct containing transmission statistics. */
#endif

#if GZLL_PA_LNA_CONTROL

#define GZLL_PA_LNA_TIMER       CONCAT_2(NRF_TIMER, GZLL_PA_LNA_TIMER_NUM) /**< Convert timer number into timer struct. */

/**< PA/LNA structure configuration. */
static nrf_gzll_pa_lna_cfg_t m_pa_lna_cfg = {
    .lna_enabled        = GZLL_LNA_ENABLED,
    .pa_enabled         = GZLL_PA_ENABLED,
    .lna_active_high    = GZLL_LNA_ACTIVE_HIGH,
    .pa_active_high     = GZLL_PA_ACTIVE_HIGH,
    .lna_gpio_pin       = GZLL_PA_LNA_CRX_PIN,
    .pa_gpio_pin        = GZLL_PA_LNA_CTX_PIN,
    .pa_gpiote_channel  = GZLL_PA_LNA_TX_GPIOTE_CHAN,
    .lna_gpiote_channel = GZLL_PA_LNA_RX_GPIOTE_CHAN,
    .timer              = GZLL_PA_LNA_TIMER,
    .ppi_channels[0]    = GZLL_PA_LNA_PPI_CHAN_1,
    .ppi_channels[1]    = GZLL_PA_LNA_PPI_CHAN_2,
    .ppi_channels[2]    = GZLL_PA_LNA_PPI_CHAN_3,
    .ppi_channels[3]    = GZLL_PA_LNA_PPI_CHAN_4,
    .ramp_up_time       = GZLL_PA_LNA_RAMP_UP_TIME
};

static int32_t m_rssi_sum    = 0; /**< Variable used to calculate average RSSI. */
static int32_t m_packets_cnt = 0; /**< Transmitted packets counter. */
#endif


#if CRS_SPI_SLAVE == 1
  void spis_event_handler(nrf_drv_spis_event_t event)
  {
      if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
      {
          spis_xfer_done = true;
          //NRF_LOG_INFO(" Transfer completed. Received:");// %s",m_rx_buf);
          //NRF_LOG_INFO("Received: %.*s\n", (int)sizeof(m_rx_buf), m_rx_buf);
          //printf("value of a_static: %.4s\n", a_static);
          //printf("test");
          //NRF_LOG_INFO("value of m_rx_buf: %.*s\n", (int)sizeof(m_rx_buf), m_rx_buf);
          NRF_LOG_INFO("value of m_rx_buf: %.64s\n", m_rx_buf);
          //printf("value of m_rx_buf: %.*s\n", (int)sizeof(m_rx_buf), m_rx_buf);
          //NRF_LOG_INFO(" Transfer completed. Received: %s",(uint32_t)m_rx_buf);
         // m_tx_buf[sizeof(TEST_STRING)] = "NORDIC";
         //m_tx_buf[2] = "Z";
         
         messageCount++;
         if (messageCount == 10) {
            messageCount = 0;
         }

         //set new_tx_buf = m_data_payload
         //new_tx_buf[63] = messageCount + '0';
      
       //   memcpy(m_tx_buf,
         //    new_tx_buf,
        //     sizeof m_tx_buf);

                //memset(m_rx_buf, 0, m_length);
             
  //        memcpy(m_tx_buf,
    //         m_data_payload,
    ///         sizeof m_data_payload);

          //m_data_payload

  
          //APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));

          //TODO pop the stack, if applicable, and populate m_tx_buf
          if (stackheight > 0)
          {
            
              memset(String, 0, sizeof(String));
             if(pop(String))
             {
               printf("Element %s\n",String);
              
               memcpy(m_tx_buf,
                  String,
                  sizeof String);
               APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
              //stackheight--;
          
              }
                else 
                {
                    printf("Error Popping Data\n");
                    memcpy(m_tx_buf,
                        blank_tx_buf,
                        sizeof blank_tx_buf);
                    APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
                }
       
          }
          else
          {
              
                    memcpy(m_tx_buf,
                        blank_tx_buf,
                        sizeof blank_tx_buf);
                    APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
          }
  
      }
  }
#endif  

/**
 * @brief Function to read the button state.
 *
 * @return Returns states of the buttons.
 */
static uint8_t input_get(void)
{
    uint8_t result = 0;
    for (uint32_t i = 0; i < BUTTONS_NUMBER; i++)
    {
        if (bsp_button_is_pressed(i))
        {
            result |= (1 << i);
        }
    }

    return ~(result);
}


/**
 * @brief Function to control the LED outputs.
 *
 * @param[in] val Desirable state of the LEDs.
 */
static void output_present(uint8_t val)
{
    uint32_t i;

    for (i = 0; i < LEDS_NUMBER; i++)
    {
        if (val & (1 << i))
        {
            bsp_board_led_on(i);
        }
        else
        {
            bsp_board_led_off(i);
        }
    }
}


static void spi_init(void)
{
  nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin               = APP_SPIS_CS_PIN;
    spis_config.miso_pin              = APP_SPIS_MISO_PIN;
    spis_config.mosi_pin              = APP_SPIS_MOSI_PIN;
    spis_config.sck_pin               = APP_SPIS_SCK_PIN;

    APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler));


}

/**
 * @brief Initialize the BSP modules.
 */
static void ui_init(void)
{
    uint32_t err_code;

    // Initialize application timer.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // BSP initialization.
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    // Set up logger.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Gazell ACK payload example. Host mode.");
    NRF_LOG_FLUSH();

    bsp_board_init(BSP_INIT_LEDS);
}


/*****************************************************************************/
/** @name Gazell callback function definitions.  */
/*****************************************************************************/
/**
 * @brief RX data ready callback.
 *
 * @details If a data packet was received, the first byte is written to LEDS.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    // Pop packet and write first byte of the payload to the GPIO port.
    bool result_value = nrf_gzll_fetch_packet_from_rx_fifo(pipe,
                                                           m_data_payload,
                                                           &data_payload_length);

    if (!result_value)
    {
        NRF_LOG_ERROR("RX fifo error ");
    }

    if (data_payload_length > 0)
    {
      //receive data here
       // output_present(m_data_payload[0]);
       printf("value of payload %s\r\n", m_data_payload);
        NRF_LOG_INFO("SUCCESSFULY RECEIVED DATA FROM device, %s", m_data_payload);
        //TODO: add this m_data_payload to the stack
        
             
        //  memcpy(m_tx_buf,
        //     m_data_payload,
         //    sizeof m_data_payload);
          if (stackheight >= MAXSTACKHEIGHT)
          {
              pop(String);
              printf("discarded %s\r\n", String);
          }
             if(!push(m_data_payload))
                {
			printf("Something went horribly wrong pushing data, but the program will carry on\n");	
		}	
             
          //m_data_payload

  
          //APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
    }

    // Read buttons and load ACK payload into TX queue.
    m_ack_payload[0] = input_get(); // Button logic is inverted.

    result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_ack_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
    }

#if GZLL_PA_LNA_CONTROL
    m_rssi_sum += rx_info.rssi;
    m_packets_cnt++;
#endif
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_disabled()
{
}

#if GZLL_PA_LNA_CONTROL
/**
 * @brief Function for configuring front end control in Gazell.
 */
static bool front_end_control_setup(void)
{
    bool result_value = true;

    // Configure pins controlling SKY66112 module.
    nrf_gpio_cfg_output(GZLL_PA_LNA_CHL_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_CPS_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_ANT_SEL_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_CSD_PIN);

    // Turn on front end module.
    nrf_gpio_pin_clear(GZLL_PA_LNA_CHL_PIN);
    nrf_gpio_pin_clear(GZLL_PA_LNA_CPS_PIN);
    nrf_gpio_pin_clear(GZLL_PA_LNA_ANT_SEL_PIN);
    nrf_gpio_pin_set(GZLL_PA_LNA_CSD_PIN);

    // PA/LNA configuration must be called after @ref nrf_gzll_init() and before @ref nrf_gzll_enable()
    result_value = nrf_gzll_set_pa_lna_cfg(&m_pa_lna_cfg);

    return result_value;
}
#endif

void queue_init()
{


	
		
	

}

/*****************************************************************************/
/**
 * @brief Main function.
 * @return ANSI required int return type.
 */
/*****************************************************************************/
int main()
{

  queue_init();
   // struct Queue Queue1;
    // Set up the user interface.
    ui_init();
    
  #if CRS_SPI_SLAVE == 1
    spi_init();
  #endif
    // Enable the constant latency sub power mode to minimize the time it takes
    // for the SPIS peripheral to become active after the CSN line is asserted
    // (when the CPU is in sleep mode).
    #if CRS_SPI_SLAVE == 1
      
      NRF_POWER->TASKS_CONSTLAT = 1;
    #endif  

    // Initialize Gazell.
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    GAZELLE_ERROR_CODE_CHECK(result_value);

#if GZLL_PA_LNA_CONTROL
    // Initialize external PA/LNA control.
    result_value = front_end_control_setup();
    GAZELLE_ERROR_CODE_CHECK(result_value);
#endif

#if GZLL_TX_STATISTICS
    // Turn on transmission statistics gathering.
    result_value = nrf_gzll_tx_statistics_enable(&m_statistics);
    GAZELLE_ERROR_CODE_CHECK(result_value);
#endif

    // Load data into TX queue.
    m_ack_payload[0] = input_get();

    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
        NRF_LOG_FLUSH();
    }

    // Enable Gazell to start sending over the air.
    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);
    
    NRF_LOG_INFO("Gzll ack payload host example started.");


        #if CRS_SPI_SLAVE == 1
                memset(m_rx_buf, 0, m_length);
                spis_xfer_done = false;
                
                APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
        #endif  
    
    while (true)
    {
        NRF_LOG_FLUSH();

        #if CRS_SPI_SLAVE == 1

            if (spis_xfer_done)
            {
                memset(m_rx_buf, 0, m_length);
                spis_xfer_done = false;

                //TODO
                //actually, here, we need the transmit buffer to reference the stack created 
                
                //APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
          
            }

            __WFE();

        #endif
        #if CRS_SPI_SLAVE == 0
          __WFE();
        #endif

#if GZLL_PA_LNA_CONTROL
        if (m_packets_cnt >= 1000)
        {
            CRITICAL_REGION_ENTER();

            // Print info about average RSSI.
            NRF_LOG_RAW_INFO("\r\n");
            NRF_LOG_INFO("Average RSSI: %d", (m_rssi_sum / m_packets_cnt));
            m_packets_cnt = 0;
            m_rssi_sum    = 0;

            CRITICAL_REGION_EXIT();
        }
#endif

#if GZLL_TX_STATISTICS
        if (m_statistics.packets_num >= 1000)
        {
            CRITICAL_REGION_ENTER();

            // Print all transmission statistics.
            NRF_LOG_RAW_INFO("\r\n");
            NRF_LOG_INFO("Total received packets:   %4u", m_statistics.packets_num);
            NRF_LOG_INFO("Total CRC failures:       %03u\r\n", m_statistics.timeouts_num);

            for (uint8_t i = 0; i < nrf_gzll_get_channel_table_size(); i++)
            {
                NRF_LOG_INFO("Channel %u: %03u packets received, %03u CRC failures.",
                             i,
                             m_statistics.channel_packets[i],
                             m_statistics.channel_timeouts[i]);
            }

            CRITICAL_REGION_EXIT();

            // Reset statistics buffers.
            nrf_gzll_reset_tx_statistics();
        }
#endif
    }
}
