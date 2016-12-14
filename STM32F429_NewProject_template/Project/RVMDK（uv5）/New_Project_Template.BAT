SET PATH=D:\Keil_v5_for_ARM\ARM\ARMCC\Bin;C:\ProgramData\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Windows\system32\wbem;d:\Program Files (x86)\IVT Corporation\BlueSoleil\Mobile;D:\Program Files\Git\cmd
SET CPU_TYPE=STM32F429ZITx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=New_Project_Template
SET CPU_CLOCK=0x00B71B00
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmAsm" --Via "..\..\output\startup_stm32f429_439xx._ia"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmCC" --Via "..\..\output\system_stm32f4xx.__i"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmCC" --Via "..\..\output\main.__i"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmCC" --Via "..\..\output\stm32f4xx_it.__i"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmCC" --Via "..\..\output\bsp_clkconfig.__i"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmCC" --Via "..\..\output\bsp_debug_usart.__i"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmCC" --Via "..\..\output\bsp_led.__i"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmCC" --Via "..\..\output\bsp_spi_flash.__i"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\ArmLink" --Via "..\..\Output\Template.lnp"
"D:\Keil_v5_for_ARM\ARM\ARMCC\Bin\fromelf.exe" "..\..\Output\Template.axf" --i32combined --output "..\..\Output\Template.hex"
