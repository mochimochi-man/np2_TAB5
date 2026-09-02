One file only, and its name is not decoration: M5Burner takes the flash
address from it and writes every file in this directory. np2_TAB5_0x0.bin
is the whole flash image - bootloader, partition table and application -
so writing it at 0x0 is all that is needed.

The same image and the four separate parts are in ../bin/ for flashing by
hand with esptool. Do not copy those here: M5Burner would write them all
and they overlap this one.
