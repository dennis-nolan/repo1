ECHO OFF


"C:\Program Files (X86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI" -c SWD -ME  -P "sw068-00_23.bin" 0x08000000 -V -P %1 0x08003000 -V -P %1 0x08041800 -V -Rst -Run  


PAUSE
