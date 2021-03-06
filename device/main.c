/*
*/

#include "bsp.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_gzll.h"
#include "nrf_gzll_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_clock.h"

//This is apparently used to , when a button is pressed, send 1000 messages and calculate latency values
#define CRS_BUTTON_ENABLED 1
#define CRS_NFC_ENABLED 1
#define CRS_TEMP_SENSOR_ENABLED 1

int messageCount = 0;

#if CRS_TEMP_SENSOR_ENABLED == 1
  #include "nrf_gpio.h"
  #include "nrf_delay.h"
  #include <stdio.h>

  static float temp = 0;
  #include "ds18b20.h"
#endif

#define PIPE_NUMBER             0   /**< Pipe 0 is used in this example. */
#define TX_PAYLOAD_LENGTH       32   /**< 1-byte payload length is used when transmitting. */
#define MAX_TX_ATTEMPTS         100 /**< Maximum number of transmission attempts */
#define CRS_TIMERS 1
#define BLANK_STRING ""

static uint8_t       blank_data_payload[] = BLANK_STRING;           /**< TX buffer. */
static uint8_t                  m_data_payload[TX_PAYLOAD_LENGTH];                /**< Payload to send to Host. */
static uint8_t                  new_data_payload[TX_PAYLOAD_LENGTH];                /**< Payload to send to Host. */
static uint8_t                  m_ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; /**< Placeholder for received ACK payloads from Host. */
uint32_t tick1;
uint32_t tick2;

#if CRS_NFC_ENABLED == 1
    #include "nfc_t2t_lib.h"
    #include "nfc_uri_msg.h"
    static const uint8_t m_url[] = {'n', 'o', 'r', 'd', 'i', 'c', 's', 'e', 'm', 'i', '.',
                                'c', 'o', 'm','/','s','t','a','r','t','5','2','8','4','0','d','k'};
    uint8_t             m_ndef_msg_buf[256];        ///< Buffer for the NFC NDEF message.
#endif

#if CRS_BUTTON_ENABLED == 1  
  #include "boards.h"
  #include "nrf.h"
  #include "nrf_drv_gpiote.h"
  #include <stdbool.h>
  #ifdef BSP_BUTTON_1
      #define PIN_IN1 BSP_BUTTON_1
  #endif

  #ifdef BSP_LED_1
      #define PIN_OUT1 BSP_LED_1
  #endif
#endif

#if GZLL_TX_STATISTICS
  static nrf_gzll_tx_statistics_t m_statistics;   /**< Struct containing transmission statistics. */
#endif

#if CRS_TIMERS == 1
  APP_TIMER_DEF(m_repeated_timer_id);
  APP_TIMER_DEF(m_repeated_temperature_timer_id);

  static void clock_init(void)
  {
      uint32_t err_code;
      err_code = nrf_drv_clock_init();
      APP_ERROR_CHECK(err_code);

      nrf_drv_clock_lfclk_request(NULL);
  }
  static void start_TEMPERATURE_timer()
  {
      ret_code_t err_code;
      err_code = app_timer_start(m_repeated_temperature_timer_id, APP_TIMER_TICKS(2000), NULL);
      APP_ERROR_CHECK(err_code);
       if (err_code == NRF_SUCCESS)
          {
            //printf("successfully started timer\r\n");
          }
          else
          {
            printf("failed to start temperature timer\r\n");
          }

  }

  static void start_latency_timer()
  {
      ret_code_t err_code;
      err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(10000), NULL);
      APP_ERROR_CHECK(err_code);
      if (err_code == NRF_SUCCESS)
          {
            //printf("successfully started timer\r\n");
          }
          else
          {
            printf("failed to start timer\r\n");
          }

  }
#endif  //end CRS_TIMERS

#if CRS_NFC_ENABLED == 1

  /**
   * @brief Callback function for handling NFC events.
   */
  static void nfc_callback(void * p_context, nfc_t2t_event_t event, const uint8_t * p_data, size_t data_length)
  {
      (void)p_context;

      switch (event)
      {
          case NFC_T2T_EVENT_FIELD_ON:
              printf("NFC EVENT FIELD ON\r\n");
              break;
          case NFC_T2T_EVENT_FIELD_OFF:
              printf("NFC EVENT FIELD OFF\r\n");
              break;
          case NFC_T2T_EVENT_DATA_READ:
              printf("NFC EVENT Data Read\r\n");
              break;
          default:
              break;
      }
  }


  static void nfc_init(void)
  {
      ret_code_t err_code;
      uint32_t   len = sizeof(m_ndef_msg_buf);

      /* Set up NFC */
      err_code = nfc_t2t_setup(nfc_callback, NULL);
      APP_ERROR_CHECK(err_code);

      /* Provide information about available buffer size to encoding function */

      /* Encode URI message into buffer */
      err_code = nfc_uri_msg_encode(NFC_URI_HTTP_WWW,
                                    m_url,
                                    sizeof(m_url),
                                    m_ndef_msg_buf,
                                    &len);

      APP_ERROR_CHECK(err_code);

      /* Set created message as the NFC payload */
      err_code = nfc_t2t_payload_set(m_ndef_msg_buf, len);
      APP_ERROR_CHECK(err_code);

      /* Start sensing NFC field */
      err_code = nfc_t2t_emulation_start();
      APP_ERROR_CHECK(err_code);
  }
#endif  //End CRS_NFC_ENABLED

/**
 * @brief Function to read the button states.
 *
 * @return Returns states of buttons.
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
 #if CRS_BUTTON_ENABLED == 0
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);
 #endif
    // Set up logger
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Gazell ACK payload example. Device mode.");
    NRF_LOG_FLUSH();

    bsp_board_init(BSP_INIT_LEDS);
}



void CRS_SEND_TEMP_GAZELLE_MSG()
{
    temp = ds18b20_get_temp();
    printf("Temperature_orig %.3f \r\n",temp);

    bool     result_value         = false;
    uint32_t m_ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    //clear the data payload. reinitiate
    memset(new_data_payload,
                 0,
                 TX_PAYLOAD_LENGTH);
    
    int ret = snprintf(new_data_payload, sizeof new_data_payload, "%.3f", temp);

     messageCount++;
     if (messageCount == 10) {
        messageCount = 0;
     }

     new_data_payload[strlen(new_data_payload)] = 0x7C;
     new_data_payload[strlen(new_data_payload)] = messageCount + '0';

      memcpy(m_data_payload,
         new_data_payload,
         sizeof m_data_payload);

    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);

    if (result_value)
    {  
        NRF_LOG_INFO("Message sent\r\n");
    }
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
    }
}

void CRS_SEND_BUTTON_GAZELLE_MSG(char identifier)
{
    printf("button pressed %c \r\n",identifier);

    bool     result_value         = false;
    uint32_t m_ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    //clear the data payload. reinitiate
     memset(new_data_payload,
             0,
             TX_PAYLOAD_LENGTH);
    
    int ret = snprintf(new_data_payload, sizeof new_data_payload, "Button Pressed: %c", identifier);

     messageCount++;
     if (messageCount == 10) {
        messageCount = 0;
     }
     new_data_payload[strlen(new_data_payload)] = 0x7C;
     new_data_payload[strlen(new_data_payload)] = messageCount + '0';
  
      memcpy(m_data_payload,
         new_data_payload,
         sizeof m_data_payload);

    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);
    if (result_value)
    {
        NRF_LOG_INFO("Message sent\r\n");
    }
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
    }
}

 static void repeated_timer_handler(void * p_context)
{
  CRS_SEND_TEMP_GAZELLE_MSG();
  printf("Timer interrupt handler reached\r\n");  
}

static void repeated_timer_temperature_handler(void * p_context)
{
  temp = ds18b20_get_temp_method_2();
  //   printf("Temperature: %.3f \r\n", temp);
  temp = ds18b20_get_temp();
  //  printf("Temperature_orig %.3f \r\n",temp);
 }

void in_pin_handler1(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{
   CRS_SEND_BUTTON_GAZELLE_MSG('31');
}

uint32_t latency_array[1001];
uint32_t latency_array_length;

void latency_array_init()
{
  int i;
  /* initialize elements of array n to 0 */         
   for ( i = 1; i < 1001; i++ ) {
      latency_array[ i ] = 0; /* set element at location i to i + 100 */
   }
   latency_array_length = 0;
}

void set_next_latency_value(uint32_t argValue)
{
  #if CRS_BUTTON_ENABLED == 1
    if (latency_array_length == 1) {
        latency_array_init();
    }
  #if CRS_BUTTON_ENABLED == 0
    if (latency_array_length == 1000) {
        latency_array_init();
    }
  #endif

  latency_array_length++;
  latency_array[ latency_array_length ] = argValue;
}

uint32_t min_latency()
{
  int i;
  uint32_t minvalue;
  minvalue = latency_array[ 1 ];
    for ( i = 1; i <= latency_array_length; i++ ) {
        if (minvalue > latency_array[ i ]) /* set element at location i to i + 100 */
        {
            minvalue = latency_array[ i ];
        }
     }
    return minvalue;
}

uint32_t max_latency()
{
  int i;
  uint32_t maxvalue = 0;
    for ( i = 1; i <= latency_array_length; i++ ) {
        if (maxvalue < latency_array[ i ]) /* set element at location i to i + 100 */
        {
            maxvalue = latency_array[ i ];
        }
     }
    return maxvalue;
}

float avg_latency()
{
    int i;
    float sum = 0;
    for (i = 1; i <= latency_array_length; i++)
    {
        sum += latency_array[ i ];
    }
    return sum / (float)latency_array_length;
}

void receive_button_press(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    bool     result_value         = false;
    uint32_t m_ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (tx_info.payload_received_in_ack)
    {
        result_value =
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, m_ack_payload, &m_ack_payload_length);
        
        tick2 = app_timer_cnt_get();
        uint32_t tick_difference;
        tick_difference = app_timer_cnt_diff_compute(tick2, tick1);
        set_next_latency_value(app_timer_ms(tick_difference));
        
        if (latency_array_length == 1)
        {
            NRF_LOG_INFO("Message has gone round trip\r\n");
            uint32_t minimum;
            uint32_t maximum;
            float average;
            minimum = min_latency();
            maximum = max_latency();
            average = avg_latency();
            printf("min = %d\r\n", minimum);
            printf("max = %d\r\n", maximum);
            printf("avg = %.2f\r\n", average);       
        } 
        if (!result_value)
        {
            NRF_LOG_ERROR("RX fifo error ");
        }

        if (m_ack_payload_length > 0)
        {
            output_present(m_ack_payload[0]);
        }
    }
}

/*****************************************************************************/
/** @name Gazell callback function definitions  */
/*****************************************************************************/
/**
 * @brief TX success callback.
 *
 * @details If an ACK was received, another packet is sent.
 */
void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{

  #if CRS_BUTTON_ENABLED == 1
      receive_button_press(pipe, tx_info);
  #endif
  #if CRS_BUTTON_ENABLED == 0
      bool     result_value         = false;
      uint32_t m_ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

      if (tx_info.payload_received_in_ack)
      {
          // Pop packet and write first byte of the payload to the GPIO port.
          result_value = nrf_gzll_fetch_packet_from_rx_fifo(pipe, m_ack_payload, &m_ack_payload_length);
        
          tick2 = app_timer_cnt_get();
          uint32_t tick_difference;
          tick_difference = app_timer_cnt_diff_compute(tick2, tick1);
          
          set_next_latency_value(app_timer_ms(tick_difference));
          
          if (latency_array_length == 1000)
          {
              uint32_t minimum;
              uint32_t maximum;
              float average;
              minimum = min_latency();
              maximum = max_latency();
              average = avg_latency();
              printf("min = %d\r\n", minimum);
              printf("max = %d\r\n", maximum);
              printf("avg = %.2f\r\n", average);
          }
 
          if (!result_value)
          {
              NRF_LOG_ERROR("RX fifo error ");
          }

          if (m_ack_payload_length > 0)
          {
              output_present(m_ack_payload[0]);
          }
      }

      // Load data payload into the TX queue.
      start_latency_timer();
      tick1 = app_timer_cnt_get();
      m_data_payload[0] = input_get();

      result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_data_payload, TX_PAYLOAD_LENGTH);
      if (!result_value)
      {
          NRF_LOG_ERROR("TX fifo error ");
      }

  #if GZLL_PA_LNA_CONTROL
      m_rssi_sum += tx_info.rssi;
      m_packets_cnt++;
  #endif
#endif
}


/**
 * @brief TX failed callback.
 *
 * @details If the transmission failed, send a new packet.
 *
 * @warning This callback does not occur by default since NRF_GZLL_DEFAULT_MAX_TX_ATTEMPTS
 * is 0 (inifinite retransmits).
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    NRF_LOG_ERROR("Gazell transmission failed");

    // Load data into TX queue.
    #if CRS_BUTTON_ENABLED == 0
        m_data_payload[0] = input_get();

        bool result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_data_payload, TX_PAYLOAD_LENGTH);
        if (!result_value)
        {
            NRF_LOG_ERROR("TX fifo error ");
        }
    #endif
}

/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
}

/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_disabled()
{
}

static void create_temperature_timers()
{
  ret_code_t err_code;

  err_code = app_timer_create(&m_repeated_temperature_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_temperature_handler);

  APP_ERROR_CHECK(err_code);
  if (err_code == NRF_SUCCESS)
  {
    printf("sucessfully created temp timer\r\n");
  }
  else
  {
    printf("failed to create temp timer\r\n");
  }
}

static void create_timers()
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_repeated_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler);
    APP_ERROR_CHECK(err_code);
    if (err_code == NRF_SUCCESS){
    printf("successfully created timer\r\n");
    }
    else
    {
  
    printf("failed to created timer\r\n");
    }
}

#if CRS_BUTTON_ENABLED
  void CRS_BUTTON_INTERRUPT_INIT()
  {
       ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);


    err_code = nrf_drv_gpiote_out_init(PIN_OUT1, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

  
    err_code = nrf_drv_gpiote_in_init(PIN_IN1, &in_config, in_pin_handler1);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN1, true);
  }
#endif

/*****************************************************************************/
/**
 * @brief Main function.
 *
 * @return ANSI required int return type.
 */
/*****************************************************************************/

int main()
{
    #if CRS_BUTTON_ENABLED == 1
          CRS_BUTTON_INTERRUPT_INIT();
    #endif
    ret_code_t err_code;

   #if CRS_TEMP_SENSOR_ENABLED == 1
    ds18b20_setResolution(12);
   #endif

    // Set up the user interface (buttons and LEDs).
    ui_init();

    latency_array_init();

    clock_init();
    create_timers();
    create_temperature_timers();
    start_TEMPERATURE_timer();

    #if CRS_NFC_ENABLED == 1
        nfc_init();
    #endif

    // Initialize Gazell.
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Attempt sending every packet up to MAX_TX_ATTEMPTS times.
    nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS);

    #if GZLL_TX_STATISTICS
        // Turn on transmission statistics gathering.
        result_value = nrf_gzll_tx_statistics_enable(&m_statistics);
        GAZELLE_ERROR_CODE_CHECK(result_value);
    #endif

    // Load data into TX queue.
    m_data_payload[0] = input_get();

    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
        NRF_LOG_FLUSH();
    }

    // Enable Gazell to start sending over the air.
    start_latency_timer();
    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);
    
    NRF_LOG_INFO("Gzll ack payload device example started.");

    while (true)
    {
        NRF_LOG_FLUSH();
        __WFE();

      #if GZLL_TX_STATISTICS
          if (m_statistics.packets_num >= 1000)
          {
              CRITICAL_REGION_ENTER();

              // Print all transmission statistics.
              NRF_LOG_RAW_INFO("\r\n");
              NRF_LOG_INFO("Total transmitted packets:   %4u",  m_statistics.packets_num);
              NRF_LOG_INFO("Total transmission time-outs: %03u\r\n", m_statistics.timeouts_num);

              for (uint8_t i = 0; i < nrf_gzll_get_channel_table_size(); i++)
              {
                  NRF_LOG_INFO("Channel %u: %03u packets transmitted, %03u transmissions failed.",
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
