/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IMX_5
#include <asf.h>
#else
#include "stm32l4xx.h"
#include "d_flash.h"
#endif

#include <string.h>
#include "../../src/data_sets.h"
#include "../../hw-libs/misc/rtos.h"
#include "bootloaderApp.h"


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



#ifndef IMX_5
#if defined(PLATFORM_IS_EVB_2)
#else
    usart_reset((Usart*)SERIAL0);
    usart_reset((Usart*)SERIAL1);
    usart_reset((Usart*)SERIAL2);
#endif    

    set_reset_pin_enabled(1);
    RSTC->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST;
#else
    __NVIC_SystemReset();
#endif

    while(1);
}

void soft_reset_no_backup_register(void)
{
    soft_reset_internal();
}

void soft_reset_backup_register(uint32_t sysFaultStatus)
{
#ifndef IMX_5
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= sysFaultStatus;    // Report cause of reset
#else
    RTC->BKP0R |= sysFaultStatus;    // Report cause of reset
#endif

    soft_reset_internal();
}

void set_reset_pin_enabled(int enabled)
{
    // *** WARNING *** Disabling the reset pin will require a chip erase via jtag to deploy new firmware
    // *** WARNING *** Provide a way to re-enable the reset pin via message or other mechanism to avoid this

#if 0

    if (enabled)
    {
        uint32_t mode = RSTC->RSTC_MR;
        mode &= ~RSTC_MR_KEY_Msk;
        mode |= (RSTC_MR_URSTEN | RSTC_MR_KEY_PASSWD);
        RSTC->RSTC_MR = mode;
    }
    else
    {
        uint32_t mode = RSTC->RSTC_MR;
        mode &= ~(RSTC_MR_URSTEN | RSTC_MR_KEY_Msk);
        mode |= RSTC_MR_KEY_PASSWD;
        RSTC->RSTC_MR = mode;
    }

#endif

}

void write_bootloader_signature_stay_in_bootloader_mode(void)
{	
    // update the bootloader header jump signature to indicate we want to go to bootloader
    bootloader_header_t header;
    memcpy(&header, (void*)BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS, BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE);
    strncpy(header.data.jumpSignature, BOOTLOADER_JUMP_SIGNATURE_STAY_IN_BOOTLOADER, sizeof(header.data.jumpSignature));

#ifndef IMX_5
    // unlock bootloader header 
	flash_unlock(BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS, BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS + BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE - 1, 0, 0);

	// this flash write is allowed to erase and write a 512 byte page because it is in the small sector, last param of 1 does this
	flash_write(BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS, &header, BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE, 1);

	// unlock flash in case of firmware downgrade
	unlockUserFlash();
#else
	flash_write(BOOTLOADER_FLASH_BOOTLOADER_HEADER_ADDRESS, &header, BOOTLOADER_FLASH_BOOTLOADER_HEADER_SIZE, 0);
#endif
}

void enable_bootloader(int pHandle)
{	
    write_bootloader_signature_stay_in_bootloader_mode();

	// Let the bootloader know which port to use for the firmware update.  Set key and port number.
#ifndef IMX_5
    GPBR->SYS_GPBR[3] = PORT_SEL_KEY_SYS_GPBR_3;
	GPBR->SYS_GPBR[4] = PORT_SEL_KEY_SYS_GPBR_4;
	GPBR->SYS_GPBR[5] = PORT_SEL_KEY_SYS_GPBR_5;
	GPBR->SYS_GPBR[6] = PORT_SEL_KEY_SYS_GPBR_6;
	GPBR->SYS_GPBR[7] = pHandle;
#else
    RTC->BKP3R = PORT_SEL_KEY_SYS_GPBR_3;
    RTC->BKP4R = PORT_SEL_KEY_SYS_GPBR_4;
    RTC->BKP5R = PORT_SEL_KEY_SYS_GPBR_5;
    RTC->BKP6R = PORT_SEL_KEY_SYS_GPBR_6;
    RTC->BKP7R = pHandle;
#endif

    // reset processor
    soft_reset_backup_register(SYS_FAULT_STATUS_ENABLE_BOOTLOADER);
}


void enable_rom_bootloader(void)
{
    unlockUserFlash();

#ifndef IMX_5
    //this enables SAM-BA
    flash_clear_gpnvm(1);
#endif

    soft_reset_backup_register(SYS_FAULT_STATUS_ENABLE_BOOTLOADER);
}

