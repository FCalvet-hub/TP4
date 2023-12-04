

#ifndef INC_EUART_H_
#define INC_EUART_H_

#include "eringbuffer.h"

#define EUART_HAL_BUFFER_SIZE (16)

/********************** euart_buffer_t *************************************/

/*! @brief  Type definition for an E-Board UART buffer.
 *
 * This structure combines/*! @b a ring buffer and a pre-allocated buffer for
 * efficient UART communication.
 */
typedef struct {
  /*!
   * Embedded ring buffer for storing data.
   */
  eringbuffer_t rb;

  /*!
   * Pre-allocated buffer for holding UART data.
   */
  uint8_t pbuffer[EUART_HAL_BUFFER_SIZE];
} euart_buffer_t;

/********************** euart_t *************************************/

/*! @brief  Type definition for an E-Board UART handle.
 *
 * This structure encapsulates state and buffers for managing an individual UART
 * peripheral.
 */
typedef struct {
  /*!
   * Pointer to the hardware-specific UART handle.
   */
  void *phardware_handle;

  /*!
   * Flag indicating if the transmit buffer is free and ready for writing.
   */
  bool tx_free;

  /*!
   * Transmit buffer for holding data to be sent over the UART.
   */
  euart_buffer_t tx;

  /*!
   * Receive buffer for storing data received from the UART.
   */
  euart_buffer_t rx;
} euart_t;

/********************** euart_hal_receive *************************************/

/*! @brief  HAL callback function for receiving data from the UART.
 *
 * This function is called by the HAL when data is received on the specified
 * UART.
 *
 * @param phardware_handle Pointer to the hardware-specific UART handle.
 * @param pbuffer Pointer to the buffer to store the received data.
 * @param size Number of bytes received.
 */
void euart_hal_receive(void *phardware_handle, uint8_t *pbuffer, size_t size);

/********************** euart_hal_send *************************************/

/*! @brief  HAL callback function for sending data over the UART.
 *
 * This function is called by the HAL when the previous transmission completes
 * or the buffer is free, allowing for sending new data.
 *
 * @param phardware_handle Pointer to the hardware-specific UART handle.
 * @param pbuffer Pointer to the buffer containing the data to send.
 * @param size Number of bytes to send.
 */
void euart_hal_send(void *phardware_handle, uint8_t *pbuffer, size_t size);

/********************** euart_init *************************************/

/*! @brief  Initializes an E-Board UART handle.
 *
 * This function sets up the provided handle with the specified hardware
 * pointer, transmit/receive buffers, and buffer sizes.
 *
 * @param phandle Pointer to the E-Board UART handle.
 * @param phardware_handle Pointer to the hardware-specific UART handle.
 * @param ptx_buffer Pointer to the transmit buffer.
 * @param tx_buffer_size Size of the transmit buffer in bytes.
 * @param prx_buffer Pointer to the receive buffer.
 * @param rx_buffer_size Size of the receive buffer in bytes.
 */
void euart_init(euart_t *phandle, void *phardware_handle, uint8_t *ptx_buffer,
                size_t tx_buffer_size, uint8_t *prx_buffer,
                size_t rx_buffer_size);

/********************** euart_write_buffer_len
 * *************************************/

/*! @brief  Gets the length of data currently in the E-Board UART transmit
 * buffer.
 *
 * This function returns the number of bytes waiting to be sent over the UART.
 *
 * @param phandle Pointer to the E-Board UART handle.
 * @return Length of data in the transmit buffer in bytes.
 */
size_t euart_write_buffer_len(euart_t *phandle);

/********************** euart_write *************************************/

/*! @brief  Writes data to the E-Board UART.
 *
 * This function attempts to write the provided data to the UART, handling
 * buffering and potential blocking.
 *
 * @param phandle Pointer to the E-Board UART handle.
 * @param buffer Pointer to the data buffer to write.
 * @param size Number of bytes to write.
 * @return Number of bytes written (may be less than requested if buffer full).
 */
size_t euart_write(euart_t *phandle, const uint8_t *buffer, size_t

// Continued from previous file...

/********************** euart_read_buffer_len *************************************/

/*! @brief  Gets the length of data currently in the E-Board UART receive buffer.
 *
 * This function returns the number of bytes available to be read from the UART.
 *
 * @param phandle Pointer to the E-Board UART handle.
 * @return Length of data in the receive buffer in bytes.
 */
size_t euart_read_buffer_len(euart_t *phandle);

/********************** euart_read *************************************/

/*! @brief  Reads data from the E-Board UART.
 *
 * This function attempts to read data from the UART, handling buffering and potential blocking.
 *
 * @param phandle Pointer to the E-Board UART handle.
 * @param buffer Pointer to the buffer to store the read data.
 * @param size Number of bytes to read (maximum to be read).
 * @return Number of bytes read (may be less than requested if buffer empty).
 */
size_t euart_read(euart_t *phandle, uint8_t *buffer, size_t size);

/********************** euart_rx_irq *************************************/

/*! @brief  HAL callback function for UART receive interrupt.
 *
 * This function is called by the HAL when data is received on the UART and the receive buffer has reached a certain threshold.
 *
 * @param phandle Pointer to the E-Board UART handle.
 * @param phardware_handle Pointer to the hardware-specific UART handle.
 * @param size Number of bytes received.
 */
void euart_rx_irq(euart_t *phandle, void *phardware_handle, size_t size);

/********************** euart_tx_irq *************************************/

/*! @brief  HAL callback function for UART transmit interrupt.
 *
 * This function is called by the HAL when data transmission completes or the buffer is free.
 *
 * @param phandle Pointer to the E-Board UART handle.
 * @param phardware_handle Pointer to the hardware-specific UART handle.
 */
void euart_tx_irq(euart_t *phandle, void *phardware_handle);

#endif /* INC_EUART_H_ */
