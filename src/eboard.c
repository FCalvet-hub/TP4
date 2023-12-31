

/********************** inclusions *******************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eboard.h"
#include "eboard_port.h"
#include "euart.h"

/********************** macros and definitions *******************************/

#define NEW_LINE_ ("\r\n")
#define RB_TX_BUFFER_SIZE_ (1024)
#define RB_RX_BUFFER_SIZE_ (256)

/********************** internal data declaration ****************************/

typedef struct {
  void *hgpio;
  bool input;
} eboard_gpio_descriptor_t_;

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/

static eboard_gpio_descriptor_t_ gpios_[EBOARD_GPIO__CNT] = {
    {hgpio : NULL, input : false}, // LED3
    {hgpio : NULL, input : false}, // LED1
    {hgpio : NULL, input : false}, // LED2
    {hgpio : NULL, input : true},  // USER BTN
};

static uint8_t tx_buffer_[RB_TX_BUFFER_SIZE_];
static uint8_t rx_buffer_[RB_RX_BUFFER_SIZE_];
static euart_t heuart_;
static euart_t *const pheuart_ = &heuart_;

/********************** external data definition *****************************/

static char elog_buffer_[32];
static char elog_user_buffer_[ELOG_MAXLEN];
char *const elog_msg = elog_user_buffer_;
int elog_msg_len;

/********************** internal functions definition ************************/

/********************** external functions definition ************************/

void eboard_uart_init(void *phuart) {
  euart_init(pheuart_, phuart, tx_buffer_, RB_TX_BUFFER_SIZE_, rx_buffer_,
             RB_RX_BUFFER_SIZE_);
}

void eboard_gpio_init(eboard_gpio_idx_t idx, void *hgpio) {
  gpios_[idx].hgpio = hgpio;
}

void eboard_gpio_write(eboard_gpio_idx_t idx, bool value) {
  if (EBOARD_GPIO__CNT <= idx) {
    return;
  }

  eboard_gpio_descriptor_t_ *hgpio = gpios_ + idx;
  if (hgpio->input) {
    return;
  }

  eboard_hal_port_gpio_write((void *)hgpio->hgpio, value);
}

bool eboard_gpio_read(eboard_gpio_idx_t idx) {
  if (EBOARD_GPIO__CNT <= idx) {
    return false;
  }

  eboard_gpio_descriptor_t_ *hgpio = gpios_ + idx;
  return eboard_hal_port_gpio_read((void *)hgpio->hgpio);
}

void eboard_led_red(bool value) { eboard_gpio_write(EBOARD_GPIO_LEDR, value); }

void eboard_led_green(bool value) {
  eboard_gpio_write(EBOARD_GPIO_LEDG, value);
}

void eboard_led_blue(bool value) { eboard_gpio_write(EBOARD_GPIO_LEDB, value); }

bool eboard_switch(void) { return eboard_gpio_read(EBOARD_GPIO_SW); }

size_t eboard_uart_tx_len(void) { return euart_write_buffer_len(pheuart_); }

size_t eboard_uart_write(const uint8_t *buffer, size_t size) {
  return euart_write(pheuart_, buffer, size);
}

size_t eboard_uart_write_byte(uint8_t byte) {
  return eboard_uart_write(&byte, 1);
}

size_t eboard_uart_swrite(const char *str) {
  size_t len = strlen(str);
  if (0 == len) {
    return 0;
  }
  return eboard_uart_write((const uint8_t *)str, len + 1);
}

size_t eboard_uart_swrite_line(const char *str) {
  size_t ret = eboard_uart_swrite(str);
  ret += eboard_uart_swrite(NEW_LINE_);
  return ret;
}

size_t eboard_uart_rx_len(void) { return euart_read_buffer_len(pheuart_); }

size_t eboard_uart_read(uint8_t *buffer, size_t size) {
  return euart_read(pheuart_, buffer, size);
}

size_t eboard_uart_read_byte(uint8_t *pbyte) {
  return eboard_uart_read(pbyte, 1);
}

size_t eboard_uart_sread(char *str, size_t max_size) {
  size_t ret = eboard_uart_read((uint8_t *)str, max_size - 1);
  str[ret] = '\0';
  return ret;
}

void eboard_log(const char *str) {
  sprintf(elog_buffer_, "[%lu] ", eboard_osal_port_get_time());
  eboard_uart_swrite(elog_buffer_);
  eboard_uart_swrite(str);
  eboard_uart_swrite_line((elog_msg_len < (ELOG_MAXLEN - 1)) ? "" : " ...");
}

// port uart
void eboard_hal_port_uart_error(void *huart) {
  // TODO: ¿?
}

void eboard_hal_port_uart_rx_irq(void *huart, uint16_t size) {
  euart_rx_irq(pheuart_, huart, size);
}

void eboard_hal_port_uart_tx_irq(void *huart) {
  euart_tx_irq(pheuart_, (void *)huart);
}

void eboard_init(void) {
  eboard_uart_init((void *)p_huart_selected_);

  for (eboard_gpio_idx_t idx = 0; idx < EBOARD_GPIO__CNT; ++idx) {
    eboard_gpio_init(idx, (void *)(driver_gpios_ + idx));
  }
}

/********************** end of file ******************************************/
