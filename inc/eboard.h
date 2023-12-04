//**
*@file eboard.h *@author Fernando Calvet(fernando.n.calvet @gmail.com) *
        @brief Hardware interfase *@version 0.1 * @date 2023 -
    12 -
    04 * *@copyright Copyright(c) 2023 * * /

#ifndef INC_EBOARD_H_
#define INC_EBOARD_H_

/********************** CPP guard ********************************************/
#ifdef __cplusplus
        extern "C" {
#endif

/********************** inclusions *******************************************/

/*! @brief  Include standard C libraries.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/********************** macros ***********************************************/

/*! @brief  Maximum length of the ELOG message.
 */
#define ELOG_MAXLEN (64)

/*! @brief  Conditional compilation for verbose logging.
 *
 * If enabled:
 * - `ELOG` macro acquires a critical section lock.
 * - Formats the message using `snprintf` and stores it in `elog_msg` with a
 * maximum length of `ELOG_MAXLEN`.
 * - Calls `eboard_log` to output the message.
 * - Releases the critical section lock.
 *
 * If disabled:
 * - `ELOG` macro does nothing.
 */
#ifdef EBOARD_CONFIG_VERBOSE
#define ELOG(...)                                                              \
  taskENTER_CRITICAL();                                                        \
  {                                                                            \
    elog_msg_len = snprintf(elog_msg, (ELOG_MAXLEN - 1), __VA_ARGS__);         \
    eboard_log(elog_msg);                                                      \
  }                                                                            \
  taskEXIT_CRITICAL()
#else
#define ELOG(...)
#endif

  /********************** typedef
   * **********************************************/

  /*! @brief  Enumeration for E-Board GPIO indices.
   */
  typedef enum {
    EBOARD_GPIO_LEDR, /*! @brief< Red LED */
    EBOARD_GPIO_LEDG, /*! @brief< Green LED */
    EBOARD_GPIO_LEDB, /*! @brief< Blue LED */
    EBOARD_GPIO_SW,   /*! @brief< Switch */
    EBOARD_GPIO__CNT, /*! @brief< Number of GPIOs */
  } eboard_gpio_idx_t;

  /********************** external data declaration
   * ****************************/

  /*! @brief  Pointer to the ELOG message buffer.
   */
  extern char *const elog_msg;

  /*! @brief  Length of the ELOG message.
   */
  extern int elog_msg_len;

  /********************** external functions declaration
   * ***********************/

  /*! @brief  Gets the current time in milliseconds using the OSAL port.
   *
   * @return Current time in milliseconds.
   */
  uint32_t eboard_osal_port_get_time(void);

  /*! @brief  Delays execution for a specified number of milliseconds using the
   * OSAL port.
   *
   * @param time_ms Time to delay in milliseconds.
   */
  void eboard_osal_port_delay(uint32_t time_ms);

  /*! @brief  Initializes the UART using the provided handle.
   *
   * @param phuart Pointer to the UART handle.
   */
  void eboard_uart_init(void *phuart);

  /*! @brief  Initializes the GPIO specified by index using the provided handle.
   *
   * @param idx Index of the GPIO.
   * @param hgpio Pointer to the GPIO handle.
   */
  void eboard_gpio_init(eboard_gpio_idx_t idx, void *hgpio);

  /*! @brief  Writes a value to the specified GPIO.
   *
   * @param idx Index of the GPIO.
   * @param value Value to write (true/false).
   */
  void eboard_gpio_write(eboard_gpio_idx_t idx, bool value);

  /*! @brief  Reads the value of the specified GPIO.
   *
   * @param idx Index of the GPIO.
   * @return Value read from the GPIO (true/false).
   */
  bool eboard_gpio_read(eboard_gpio_idx_t idx);

  /*! @brief  Turns the red LED on or off.
   *
   * @param value Whether to turn the LED on (true) or off (false).
   */
  void eboard_led_red(bool value);

  /*! @brief  Turns the green LED on or off.
   *
   * @param value Whether to turn the LED on (true) or off (false).
   */
  void eboard_led_green(bool value);

  /*! @brief  Turns the blue LED on or off.
   *
   * @param value Whether to turn the LED on (true) or off (false).
   */
  void eboard_led_blue(bool value);

  /********************** eboard_switch *************************************/

  /*! @brief  Reads the state of the switch.
   *
   * @return True if the switch is pressed, false otherwise.
   */
  bool eboard_switch(void);

  /********************** eboard_uart_tx_len
   * *************************************/

  /*! @brief  Gets the number of bytes currently in the UART transmit buffer.
   *
   * @return Number of bytes in the transmit buffer.
   */
  size_t eboard_uart_tx_len(void);

  /********************** eboard_uart_write
   * *************************************/

  /*! @brief  Writes a block of data to the UART.
   *
   * @param buffer Pointer to the data buffer.
   * @param size Number of bytes to write.
   * @return Number of bytes written.
   */
  size_t eboard_uart_write(const uint8_t *buffer, size_t size);

  /********************** eboard_uart_write_byte
   * ***********************************/

  /*! @brief  Writes a single byte to the UART.
   *
   * @param byte The byte to write.
   * @return Number of bytes written (always 1).
   */
  size_t eboard_uart_write_byte(uint8_t byte);

  /********************** eboard_uart_swrite
   * *************************************/

  /*! @brief  Writes a null-terminated string to the UART.
   *
   * @param str Pointer to the string to write.
   * @return Number of bytes written.
   */
  size_t eboard_uart_swrite(const char *str);

  /********************** eboard_uart_swrite_line
   * *********************************/

  /*! @brief  Writes a null-terminated string to the UART followed by a newline
   * character.
   *
   * @param str Pointer to the string to write.
   * @return Number of bytes written.
   */
  size_t eboard_uart_swrite_line(const char *str);

  /********************** eboard_uart_rx_len
   * *************************************/

  /*! @brief  Gets the number of bytes currently in the UART receive buffer.
   *
   * @return Number of bytes in the receive buffer.
   */
  size_t eboard_uart_rx_len(void);

  /********************** eboard_uart_read
   * *************************************/

  /*! @brief  Reads a block of data from the UART.
   *
   * @param buffer Pointer to the buffer to store the read data.
   * @param size Maximum number of bytes to read.
   * @return Number of bytes read.
   */
  size_t eboard_uart_read(uint8_t * buffer, size_t size);

  /********************** eboard_uart_read_byte
   * ***********************************/

  /*! @brief  Reads a single byte from the UART.
   *
   * @param pbyte Pointer to the variable to store the read byte.
   * @return Number of bytes read (always 1).
   */
  size_t eboard_uart_read_byte(uint8_t * pbyte);

  /********************** eboard_uart_sread
   * *************************************/

  /*! @brief  Reads a string from the UART up to a maximum length.
   *
   * @param str Pointer to the buffer to store the read string.
   * @param max_size Maximum number of characters to read (excluding null
   * terminator).
   * @return Number of characters read.
   */
  size_t eboard_uart_sread(char *str, size_t max_size);

  /********************** eboard_hal_port_uart_error
   * *********************************/

  /*! @brief  Callback function called by the HAL when an error occurs in the
   * UART port.
   *
   * @param huart Pointer to the UART handle.
   */
  void eboard_hal_port_uart_error(void *huart);

  /********************** eboard_hal_port_uart_rx_irq
   * *********************************/

  /*! @brief  Callback function called by the HAL when data is received on the
   * UART.
   *
   * @param huart Pointer to the UART handle.
   * @param size Number of bytes received.
   */
  void eboard_hal_port_uart_rx_irq(void *huart, uint16_t size);

  /********************** eboard_hal_port_uart_tx_irq
   * *********************************/

  /*! @brief  Callback function called by the HAL when data transmission
   * completes on the UART.
   *
   * @param huart Pointer to the UART handle.
   */
  void eboard_hal_port_uart_tx_irq(void *huart);

  /********************** eboard_hal_port_gpio_write
   * *********************************/

  /*! @brief  Writes a digital value to a GPIO using the HAL.
   *
   * @param handle Pointer to the HAL GPIO handle.
   * @param value Value to write (true/false).
   */
  void eboard_hal_port_gpio_write(void *handle, bool value);

  /********************** eboard_hal_port_gpio_read
   * *********************************/

  /*! @brief  Reads the digital value of a GPIO using the HAL.
   *
   * @param handle Pointer to the HAL GPIO handle.
   * @return Value read from the GPIO (true/false).
   */
  bool eboard_hal_port_gpio_read(void *handle);

  /********************** eboard_log *************************************/

  /*! @brief  Writes a log message to the E-Board log buffer.
   *
   * @param str Pointer to the null-terminated log message string.
   */
  void eboard_log(const char *str);

  /********************** eboard_init *************************************/

  /*! @brief  Initializes the E-Board hardware and software modules.
   */
  void eboard_init(void);

/********************** End of CPP guard *************************************/
#ifdef __cplusplus
}
#endif

#endif /* INC_EBOARD_H_ */
/********************** end of file ******************************************/
