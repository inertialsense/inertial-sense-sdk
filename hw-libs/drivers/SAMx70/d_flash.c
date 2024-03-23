/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include <stdlib.h>
#include "globals.h"
#include "bootloaderShared.h"
#include "rtos.h"
#include "d_flash.h"
#include "ISBoards.h"

static int s_flashWriteInProgress;

uint32_t flash_update_block(uint32_t address, const void* newData, int dataSize, int noPageErase)
{
	uint32_t result;
			
	if (address < BOOTLOADER_FLASH_USER_DATA_START_ADDRESS || 
        address >= BOOTLOADER_FLASH_USER_DATA_END_ADDRESS || 
        (noPageErase==0 && address % BOOTLOADER_FLASH_BLOCK_SIZE != 0) ||
        dataSize > (int)BOOTLOADER_FLASH_BLOCK_SIZE )
	{
		result = FLASH_RC_INVALID;
	}
	else if (memcmp((const void*)address, newData, dataSize) == 0)
	{
		// memory is identical, do not write
		result = FLASH_RC_OK;
	}
	else // if contents are different, perform the flash write
	{
		// assume failure unless we succeed	
		result = FLASH_RC_ERROR;
		
		// http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11242-32-bit-Cortex-M7-Microcontroller-SAM-S70Q-SAM-S70N-SAM-S70J_Datasheet.pdf
		// page 45

		// unlock the block
		s_flashWriteInProgress = 1;
		flash_unlock(address, address + dataSize - 1, 0, 0);

		// try up to 10 times to write flash, recommended by Atmel
		for (int i = 0; i < 10; i++)
		{       
            if(!noPageErase)
            {   // erase flash - Only works for 16 pages (8192 bytes) and NOT less.
                result = flash_erase_page(address, IFLASH_ERASE_PAGES_16);
            }
            else
            {   // skip flash erase
                result = FLASH_RC_OK;
            }
                 
			// erase flash - Only works for 16 pages (8192 bytes) and NOT less.
			if (result == FLASH_RC_OK)
			{
				// write new flash
				result = flash_write(address, newData, dataSize, 0);
				
				if (result == FLASH_RC_OK && memcmp((const void*)address, newData, dataSize) == 0)
				{
					result = FLASH_RC_OK;
					break;
				}
			}
			time_delay_msec(2);
		}
					
		// lock the block
		flash_lock(address, address + dataSize - 1, 0, 0);
		
		s_flashWriteInProgress = 0;
	}
		
	return result;
}

uint32_t flash_erase_block(uint32_t address)
{
	if (address < BOOTLOADER_FLASH_USER_DATA_START_ADDRESS || address >= BOOTLOADER_FLASH_USER_DATA_END_ADDRESS || address % BOOTLOADER_FLASH_BLOCK_SIZE != 0)
	{
		return FLASH_RC_INVALID;
	}
	
#ifdef ENABLE_WDT
	WDT->WDT_CR = 0xA5000000 | WDT_CR_WDRSTT;	// restart watchdog
#endif

	flash_unlock(address, address + BOOTLOADER_FLASH_BLOCK_SIZE - 1, 0, 0);
	return flash_erase_page(address, IFLASH_ERASE_PAGES_16);
}

#ifdef ENABLE_WDT
#define UPDATE_WATCHDOG() { WDT->WDT_CR = 0xA5000000 | WDT_CR_WDRSTT; }
#else
#define UPDATE_WATCHDOG() {}
#endif

extern uint32_t efc_perform_fcr(Efc *p_efc, uint32_t ul_fcr); 
__no_inline RAMFUNC void flash_reboot_down(int chip_erase)
{
	LED_COLOR_RED();

	cpu_irq_disable();	
	UPDATE_WATCHDOG();
		
	// Make sure all blocks are unlocked
	flash_unlock(IFLASH_ADDR, IFLASH_ADDR + IFLASH_SIZE, NULL, NULL);
	
	if (chip_erase)
	{	// Issue erase - After this we cannot access any functions in flash as it will be gone.
		EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD(EFC_FCMD_EA);
		while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY)
			UPDATE_WATCHDOG();
	}

	//Clear GPNVM Bits
	EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG(0) | EEFC_FCR_FCMD(EFC_FCMD_CGPB);		//Protect bit
	while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY)
		UPDATE_WATCHDOG();
	
	EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG(1) | EEFC_FCR_FCMD(EFC_FCMD_CGPB);		//Enter SAM-BA
	while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY)
		UPDATE_WATCHDOG();
	
	EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG(7) | EEFC_FCR_FCMD(EFC_FCMD_CGPB);		//TCM config
	while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY)
		UPDATE_WATCHDOG();
	
	EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG(8) | EEFC_FCR_FCMD(EFC_FCMD_CGPB);		//TCM config
	while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY)
		UPDATE_WATCHDOG();
			
	LEDS_ALL_OFF();

	//Reset Device
	RSTC->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST;
	while(1);
}

extern uint32_t efc_perform_fcr(Efc *p_efc, uint32_t ul_fcr); 
__no_inline RAMFUNC void flash_rom_bootloader(void)
{
	// Reboot into ROM bootloader mode
	flash_reboot_down(0);	
}

extern uint32_t efc_perform_fcr(Efc *p_efc, uint32_t ul_fcr); 
__no_inline RAMFUNC void flash_erase_chip(void)
{
	// chip erase and reboot into ROM bootloader mode
	flash_reboot_down(1);	
}

uint32_t flash_get_user_signature(volatile void* ptr, uint32_t size)
{
	BEGIN_CRITICAL_SECTION
	
	uint32_t ret = flash_read_user_signature((uint32_t*)ptr, size / sizeof(uint32_t));

	END_CRITICAL_SECTION

	return ret;
}

uint32_t flash_set_user_signature(const volatile void *ptr, uint32_t size)
{
	// get copy of user signature in critical section
	BEGIN_CRITICAL_SECTION
	
	uint32_t* ptrCopy = (uint32_t*)pvPortMalloc(size);
	memcpy(ptrCopy, (const void*)ptr, size);
	
	END_CRITICAL_SECTION

	// erase user signature first
	flash_erase_user_signature();
	
	// perform flash write with copy of data outside critical section, size for this function is
	//  number of 32 bit words.
	uint32_t result = flash_write_user_signature(ptrCopy, size / sizeof(uint32_t));
	vPortFree(ptrCopy);
	return result;
}

int flash_write_in_progress(void)
{
	return s_flashWriteInProgress;
}
