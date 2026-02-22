# Simple-STM32-Bootloader
Minimal STM32 custom bootloader that jumps to an application stored at a different flash address after a 5-second delay. Demonstrates VTOR remapping, MSP reconfiguration, and clean interrupt handover.

This project demonstrates a custom STM32 bootloader built using STM32 HAL.  
The bootloader initializes basic peripherals, provides UART debug output and LED indication, and then transfers execution to an application firmware located at a different flash memory address.

---

## 📌 Flash Memory Layout

Bootloader Start   : 0x08000000  
Application Header : 0x08008000  
Application Start  : 0x0800C000  

| Region               | Address       | Description |
|----------------------|--------------|-------------|
| Bootloader           | 0x08000000   | Executes after reset |
| Application Header   | 0x08008000   | Optional metadata region |
| Application Firmware | 0x0800C000   | Main user application |

---

## ⚙️ Bootloader Features

- STM32 HAL-based implementation  
- UART debug output (115200 baud)  
- LED status indication (PC13)  
- Clean interrupt shutdown before jump  
- SysTick reset before application transfer  
- MSP (Main Stack Pointer) reconfiguration  
- Direct jump to application Reset_Handler  

---

## 🔁 Bootloader Execution Flow

1. HAL_Init()
2. System Clock Configuration (HSI)
3. GPIO Initialization
4. UART Initialization
5. Print: "Inside Bootloader!!"
6. Blink LED
7. Execute JumpToApplication()

---
