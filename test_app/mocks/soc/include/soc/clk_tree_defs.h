/*
 * Linux SOC is missing some defs, we have to stub them with dummy values
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * Mock definitions for running on the host.
     */

    /**
     * @brief Supported clock sources for modules (CPU, peripherals, RTC, etc.)
     *
     * @note enum starts from 1, to save 0 for special purpose
     */
    typedef enum
    {
        SOC_MOD_CLK_APB = 1,
    } soc_module_clk_t;

    /**
     * @brief Type of SPI clock source.
     */
    typedef enum
    {
        SPI_CLK_SRC_DEFAULT = SOC_MOD_CLK_APB,
        SPI_CLK_SRC_APB = SOC_MOD_CLK_APB,
    } soc_periph_spi_clk_src_t;
    
    typedef enum
    {
        UART_SCLK_PLL_F80M = 1, /*!< UART source clock is PLL_F80M */
        UART_SCLK_RTC = 1,      /*!< UART source clock is RC_FAST */
        UART_SCLK_XTAL = 1,     /*!< UART source clock is XTAL */
        UART_SCLK_DEFAULT = 1,  /*!< UART source clock default choice is PLL_F80M */
    } soc_periph_uart_clk_src_legacy_t;

#ifdef __cplusplus
}
#endif
