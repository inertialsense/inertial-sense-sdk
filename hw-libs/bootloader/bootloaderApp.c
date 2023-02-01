/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#if !defined(IMX_5) && !defined(GPX_1)
#include <asf.h>
#else
#include "ISBoards.h"
#endif

#include "data_sets.h"
#include "bootloaderApp.h"

#include <string.h>

void unlockUserFlash(void)
{
#ifndef IMX_5
    // unlock 64K of config data at end in event that downgrade of firmware is happening, old firmware did not attempt to unlock before flash writes
    for (uint32_t flashUnlockStart = BOOTLOADER_FLASH_USER_DATA_START_ADDRESS; flashUnlockStart < BOOTLOADER_FLASH_USER_DATA_END_ADDRESS; flashUnlockStart += BOOTLOADER_FLASH_BLOCK_SIZE)
    {
        flash_unlock(flashUnlockStart, flashUnlockStart + BOOTLOADER_FLASH_BLOCK_SIZE - 1, 0, 0); // unlock is inclusive
    }
#else
    // IMX-5 can currently only lock/unlock all of memory
    // TODO: Use write protect to protect user area and bootloader from erase.
#endif
}


static void soft_reset_internal(void)
{
	__disable_irq();
	__DMB();

#if defined(IMX_5) || defined(GPX_1)
    NVIC_SystemReset();
#else
#if !defined(PLATFORM_IS_EVB_2)
    usart_reset((Usart*)SERIAL0);
    usart_reset((Usart*)SERIAL1);
    usart_reset((Usart*)SERIAL2);
#endif    
    RSTC->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST;
#endif

    while(1);
}

void soft_reset_no_backup_register(void)
{
    soft_reset_internal();
}

void soft_reset_backup_register(uint32_t sysFaultStatus)
{
#if !defined(IMX_5) && !defined(GPX_1)
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= sysFaultStatus;    // Report cause of reset
#else
    BKUP_PERIPH->BKP0R |= sysFaultStatus;    // Report cause of reset
#endif

    soft_reset_internal();
}

void enable_bootloader(int pHandle)
{	
    // update the bootloader header jump signature to indicate we want to go to bootloader
    bootloader_header_t header;
    memcpy(&header, (void*)BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS, BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE);
    strncpy(header.data.jumpSignature, BOOTLOADER_JUMP_SIGNATURE_STAY_IN_BOOTLOADER, sizeof(header.data.jumpSignature));


#if !defined(IMX_5) && !defined(GPX_1)
    // unlock bootloader header 
	flash_unlock(BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS, BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS + BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE - 1, 0, 0);

	// this flash write is allowed to erase and write a 512 byte page because it is in the small sector, last param of 1 does this
	flash_write(BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS, &header, BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE, 1);

	// unlock flash in case of firmware downgrade
	unlockUserFlash();
#else
	flash_write(BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS, &header, BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE, 0);
#endif
    
	// Let the bootloader know which port to use for the firmware update.  Set key and port number.
#if !defined(IMX_5) && !defined(GPX_1)
    GPBR->SYS_GPBR[3] = PORT_SEL_KEY_SYS_GPBR_3;
	GPBR->SYS_GPBR[4] = PORT_SEL_KEY_SYS_GPBR_4;
	GPBR->SYS_GPBR[5] = PORT_SEL_KEY_SYS_GPBR_5;
	GPBR->SYS_GPBR[6] = PORT_SEL_KEY_SYS_GPBR_6;
	GPBR->SYS_GPBR[7] = pHandle;
#else
    BKUP_PERIPH->BKP3R = PORT_SEL_KEY_SYS_GPBR_3;
    BKUP_PERIPH->BKP4R = PORT_SEL_KEY_SYS_GPBR_4;
    BKUP_PERIPH->BKP5R = PORT_SEL_KEY_SYS_GPBR_5;
    BKUP_PERIPH->BKP6R = PORT_SEL_KEY_SYS_GPBR_6;
    BKUP_PERIPH->BKP7R = pHandle;
#endif

    // reset processor
    soft_reset_backup_register(SYS_FAULT_STATUS_ENABLE_BOOTLOADER);
}


void enable_bootloader_assistant(void)
{
    unlockUserFlash();

#if !defined(IMX_5) && !defined(GPX_1)
    //this enables SAM-BA
    flash_clear_gpnvm(1);
#endif

    soft_reset_backup_register(SYS_FAULT_STATUS_ENABLE_BOOTLOADER);
}


