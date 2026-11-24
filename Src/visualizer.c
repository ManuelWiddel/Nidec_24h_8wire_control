#include "visualizer.h"

#include <string.h>


/* ============================================================
 * PRIVATE VARIABLES
 * ============================================================ */

static UART_HandleTypeDef *visualizer_uart = NULL;

static uint8_t visualizer_number_of_streams = 0;


/* ============================================================
 * PRIVATE FUNCTION
 *
 * Send one character
 * ============================================================ */

static void Visualizer_SendChar(char c)
{
    HAL_UART_Transmit(
        visualizer_uart,
        (uint8_t *)&c,
        1,
        HAL_MAX_DELAY
    );
}


/* ============================================================
 * PRIVATE FUNCTION
 *
 * Send a string
 * ============================================================ */

static void Visualizer_SendString(const char *text)
{
    HAL_UART_Transmit(
        visualizer_uart,
        (uint8_t *)text,
        strlen(text),
        HAL_MAX_DELAY
    );
}


/* ============================================================
 * PRIVATE FUNCTION
 *
 * Send signed 32-bit integer
 *
 * Does NOT use printf.
 * ============================================================ */

static void Visualizer_SendInt32(int32_t value)
{
    char buffer[12];

    uint8_t index = 0;

    uint32_t magnitude;


    /* --------------------------------------------------------
     * Handle negative numbers
     *
     * Special care is required for INT32_MIN because its
     * positive equivalent cannot be represented by int32_t.
     * -------------------------------------------------------- */

    if (value < 0)
    {
        Visualizer_SendChar('-');

        magnitude =
            (uint32_t)(-(value + 1));

        magnitude += 1U;
    }
    else
    {
        magnitude =
            (uint32_t)value;
    }


    /* --------------------------------------------------------
     * Convert number to ASCII
     * -------------------------------------------------------- */

    if (magnitude == 0)
    {
        Visualizer_SendChar('0');

        return;
    }


    while (magnitude > 0)
    {
        buffer[index++] =
            '0' + (magnitude % 10U);

        magnitude /= 10U;
    }


    /* --------------------------------------------------------
     * Send characters in correct order
     * -------------------------------------------------------- */

    while (index > 0)
    {
        index--;

        Visualizer_SendChar(
            buffer[index]
        );
    }
}


/* ============================================================
 * PRIVATE FUNCTION
 *
 * Send scaled integer as decimal number.
 *
 * Scale = 1000
 *
 * Examples:
 *
 * 25123   -> 25.123
 * 3300    -> 3.300
 * 125     -> 0.125
 * -12500  -> -12.500
 * ============================================================ */


static void Visualizer_SendScaledInt(int32_t value)
{
    int32_t integer_part;
    int32_t remainder;
    uint32_t fractional_part;

    /* Handle sign separately */
    if (value < 0)
    {
        Visualizer_SendChar('-');

        /* Safe even for INT32_MIN */
        uint32_t magnitude =
            (uint32_t)(-(value + 1)) + 1U;

        integer_part =
            magnitude / VISUALIZER_SCALE;

        fractional_part =
            magnitude % VISUALIZER_SCALE;
    }
    else
    {
        integer_part =
            value / VISUALIZER_SCALE;

        fractional_part =
            value % VISUALIZER_SCALE;
    }

    /* Send integer portion */
    Visualizer_SendInt32(integer_part);

    /* Decimal point */
    Visualizer_SendChar('.');

    /* Send exactly three fractional digits */
    Visualizer_SendChar(
        '0' + (fractional_part / 100U)
    );

    fractional_part %= 100U;

    Visualizer_SendChar(
        '0' + (fractional_part / 10U)
    );

    fractional_part %= 10U;

    Visualizer_SendChar(
        '0' + fractional_part
    );
}



/* ============================================================
 * INITIALIZE VISUALIZER
 * ============================================================ */

void Visualizer_Init(
    UART_HandleTypeDef *uart,
    uint8_t number_of_streams,
    const char *names[]
)
{
    if (uart == NULL)
    {
        return;
    }

    if (names == NULL)
    {
        return;
    }

    if (number_of_streams == 0)
    {
        return;
    }


    /* --------------------------------------------------------
     * Limit number of streams
     * -------------------------------------------------------- */

    if (number_of_streams >
        VISUALIZER_MAX_STREAMS)
    {
        number_of_streams =
            VISUALIZER_MAX_STREAMS;
    }


    /* --------------------------------------------------------
     * Store configuration
     * -------------------------------------------------------- */

    visualizer_uart = uart;

    visualizer_number_of_streams =
        number_of_streams;


    /* --------------------------------------------------------
     * Send configuration header
     * -------------------------------------------------------- */

    Visualizer_SendString(
        "#NAMES:"
    );


    /* --------------------------------------------------------
     * Send stream names
     * -------------------------------------------------------- */

    for (uint8_t i = 0;
         i < number_of_streams;
         i++)
    {
        if (i > 0)
        {
            Visualizer_SendChar(',');
        }

        Visualizer_SendString(
            names[i]
        );
    }


    /* --------------------------------------------------------
     * End configuration message
     * -------------------------------------------------------- */

    Visualizer_SendString(
        "\r\n"
    );
}


/* ============================================================
 * SEND SAMPLE
 * ============================================================ */

void Visualizer_Send(
    const int32_t values[],
    uint8_t number_of_values,
	uint32_t sample_time_ms
)
{
    if (visualizer_uart == NULL)
    {
        return;
    }

    if (values == NULL)
    {
        return;
    }


    /* --------------------------------------------------------
     * Make sure the number of values matches the
     * configured number of streams.
     * -------------------------------------------------------- */

    if (number_of_values !=
        visualizer_number_of_streams)
    {
        return;
    }

    /* --------------------------------------------------------
         * Send sample time in milliseconds
         * -------------------------------------------------------- */

        Visualizer_SendScaledInt(sample_time_ms);

        Visualizer_SendChar(',');

    /* --------------------------------------------------------
     * Send every value
     * -------------------------------------------------------- */

    for (uint8_t i = 0;
         i < number_of_values;
         i++)
    {
        if (i > 0)
        {
            Visualizer_SendChar(',');
        }


        Visualizer_SendScaledInt(
            values[i]
        );
    }


    /* --------------------------------------------------------
     * End sample
     * -------------------------------------------------------- */

    Visualizer_SendString(
        "\r\n"
    );
}
