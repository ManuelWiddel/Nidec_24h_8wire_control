#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "main.h"
#include <stdint.h>

/*
 * Maximum number of visualization streams.
 */
#define VISUALIZER_MAX_STREAMS 32


/*
 * Scaling factor.
 *
 * Actual value = integer value / 1000
 *
 * Example:
 *
 * 25123 -> 25.123
 * 3300  -> 3.300
 */
#define VISUALIZER_SCALE 1000


/*
 * Initialize the visualizer.
 *
 * Sends the stream names to the PC.
 *
 * Example:
 *
 * const char *names[] =
 * {
 *     "Temperature",
 *     "Pressure",
 *     "Voltage"
 * };
 *
 * Visualizer_Init(
 *     &huart1,
 *     3,
 *     names
 * );
 */
void Visualizer_Init(
    UART_HandleTypeDef *uart,
    uint8_t number_of_streams,
    const char *names[]
);


/*
 * Send one sample.
 *
 * values[] contains scaled integer values.
 *
 * Example:
 *
 * int32_t values[3] =
 * {
 *     25123,
 *     1013250,
 *     3300
 * };
 *
 * Visualizer_Send(
 *     values,
 *     3
 * );
 *
 * Python receives:
 *
 * 25.123,1013.250,3.300
 */
void Visualizer_Send(
    const int32_t values[],
    uint8_t number_of_values,
	uint32_t sample_time_ms
);

#endif /* VISUALIZER_H */
