MEMORY
{
    FLASH      : ORIGIN = 0x08000000, LENGTH =   60K /* BANK_1: general flash; .text, .data etc */
    APPCONFIG0 : ORIGIN = 0x0800f000, LENGTH =   1K  /* BANK_1: configuration variables sector 0 */
    APPCONFIG1 : ORIGIN = 0x0800f400, LENGTH =   1K  /* BANK_1: configuration variables sector 1 */
    APPCONFIG2 : ORIGIN = 0x0800f800, LENGTH =   1K  /* BANK_1: configuration variables sector 2 */
    APPCONFIG3 : ORIGIN = 0x0800fc00, LENGTH =   1K  /* BANK_1: configuration variables sector 3 */
    RAM        : ORIGIN = 0x20000000, LENGTH =   20K /* SRAM */
}

__appconfig0_start = ORIGIN(APPCONFIG0) - ORIGIN(FLASH);
__appconfig1_start = ORIGIN(APPCONFIG1) - ORIGIN(FLASH);
__appconfig2_start = ORIGIN(APPCONFIG2) - ORIGIN(FLASH);
__appconfig3_start = ORIGIN(APPCONFIG3) - ORIGIN(FLASH);
