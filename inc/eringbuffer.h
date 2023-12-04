

#ifndef INC_ERINGBUFFER_H_
#define INC_ERINGBUFFER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/********************** eringbuffer_t *************************************/

/*! @brief  Type definition for an E-Board ring buffer.
 *
 * This structure represents a circular buffer for storing data efficiently.
 */
typedef struct {
  /*!
   * Pointer to the buffer memory.
   */
  uint8_t *buffer;

  /*!
   * Total size of the buffer in bytes.
   */
  size_t size;

  /*!
   * Current number of bytes written to the buffer.
   */
  size_t len;

  /*!
   * Write index (position to write the next byte).
   */
  size_t w;

  /*!
   * Read index (position to read the next byte).
   */
  size_t r;
} eringbuffer_t;

/********************** eringbuffer_init *************************************/

/*! @brief  Initializes a ring buffer.
 *
 * This function sets up the internal state of the provided ring buffer with the
 * specified buffer pointer and size.
 *
 * @param rb Pointer to the ring buffer to initialize.
 * @param buffer Pointer to the buffer memory.
 * @param size Size of the buffer in bytes.
 */
void eringbuffer_init(eringbuffer_t *rb, uint8_t *buffer, size_t size);

/********************** eringbuffer_size *************************************/

/*! @brief  Gets the total size of the ring buffer.
 *
 * @param rb Pointer to the ring buffer.
 * @return Total size of the buffer in bytes.
 */
size_t eringbuffer_size(const eringbuffer_t *rb);

/********************** eringbuffer_len *************************************/

/*! @brief  Gets the current length of data in the ring buffer.
 *
 * This function returns the number of bytes currently written to the buffer,
 * not including the space available for writing.
 *
 * @param rb Pointer to the ring buffer.
 * @return Current length of data in bytes.
 */
size_t eringbuffer_len(const eringbuffer_t *rb);

/********************** eringbuffer_free *************************************/

/*! @brief  Gets the amount of free space available in the ring buffer.
 *
 * This function calculates the space remaining in the buffer for writing data.
 *
 * @param rb Pointer to the ring buffer.
 * @return Amount of free space available in bytes.
 */
size_t eringbuffer_free(const eringbuffer_t *rb);

/********************** eringbuffer_is_full
 * *************************************/

/*! @brief  Checks if the ring buffer is full.
 *
 * This function determines if there is no more space available for writing data
 * to the buffer.
 *
 * @param rb Pointer to the ring buffer.
 * @return True if the buffer is full, false otherwise.
 */
bool eringbuffer_is_full(const eringbuffer_t *rb);

/********************** eringbuffer_is_empty
 * *************************************/

/*! @brief  Checks if the ring buffer is empty.
 *
 * This function determines if there is no data currently stored in the buffer.
 *
 * @param rb Pointer to the ring buffer.
 * @return True if the buffer is empty, false otherwise.
 */
bool eringbuffer_is_empty(const eringbuffer_t *rb);

/********************** eringbuffer_write_byte
 * *************************************/

/*! @brief  Writes a single byte to the ring buffer.
 *
 * This function efficiently writes a byte to the buffer, updating internal
 * indexes and handling wrap-around.
 *
 * @param rb Pointer to the ring buffer.
 * @param byte The byte to write.
 * @return Number of bytes written (always 1).
 */
size_t eringbuffer_write_byte(eringbuffer_t *rb, uint8_t byte);

/********************** eringbuffer_write *************************************/

/*! @brief  Writes a block of data to the ring buffer.
 *
 * This function efficiently writes a buffer of bytes to the ring buffer,
 * handling wrap-around and partial writes if necessary.
 *
 * @param rb Pointer to the ring buffer.
 * @param buffer Pointer to the data buffer to write.
 * @param size Number of bytes to write.
 * @return Number of bytes written.
 */
size_t eringbuffer_write(eringbuffer_t *rb, const uint8_t *buffer, size_t size);

// Continued from previous file...

/********************** eringbuffer_read_byte
 * *************************************/

/*! @brief  Reads a single byte from the ring buffer.
 *
 * This function efficiently reads a byte from the buffer, updating internal
 * indexes and handling wrap-around.
 *
 * @param rb Pointer to the ring buffer.
 * @param byte Pointer to store the read byte.
 * @return Number of bytes read (always 1).
 */
size_t eringbuffer_read_byte(eringbuffer_t *rb, uint8_t *byte);

/********************** eringbuffer_read *************************************/

/*! @brief  Reads a block of data from the ring buffer.
 *
 * This function efficiently reads a buffer of bytes from the ring buffer,
 * handling wrap-around and partial reads if necessary.
 *
 * @param rb Pointer to the ring buffer.
 * @param buffer Pointer to the buffer to store the read data.
 * @param size Number of bytes to read.
 * @return Number of bytes read.
 */
size_t eringbuffer_read(eringbuffer_t *rb, uint8_t *buffer, size_t size);

#endif /* INC_ERINGBUFFER_H_ */
