MEMORY
{
    FLASH     : ORIGIN = 0x08000000, LENGTH =   63K /* BANK_1: general flash; .text, .data etc */
    APPCONFIG : ORIGIN = 0x0800fc00, LENGTH =   1K  /* BANK_1: configuration variables */
    RAM       : ORIGIN = 0x20000000, LENGTH =   20K /* SRAM */
}

SECTIONS
{
  .appconfig ORIGIN(APPCONFIG) :
  {
    *(.appconfig .appconfig.*);
  } > APPCONFIG
}
