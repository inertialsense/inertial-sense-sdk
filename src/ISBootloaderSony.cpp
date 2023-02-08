#include "ISBootloaderSony.h"

using namespace ISBootloader;

enum
{
	CXD_SET_STATUS = 0x00,
	CXD_PROGRAM_CODE_INJECTION = 0x01,
	CXD_PROGRAM_EXECUTION = 0x02,
	CXD_UART_SETTING = 0x03,
	CXD_I2C_SETTING = 0x04,
	CXD_GET_FIRMWARE_REV = 0x06,
	CXD_BINARY_DATA_INJECTION = 0x07,
	CXD_BINARY_DATA_OUTPUT = 0x08,
	CXD_WRITE_PROGRAM = 0x09,
};

typedef struct
{
	uint8_t sync;
	uint16_t oplen;
	uint8_t opcode;
} cxd5610_hdr_t;

typedef struct
{
	cxd5610_hdr_t hdr;
	uint8_t cksum;
} cxd5610_cmd_t;

is_operation_result cISBootloaderSONY::match_test(void* param)
{
    const char* serial_name = (const char*)param;

    if(strnlen(serial_name, 100) != 0 && strncmp(serial_name, m_port->port, 100) == 0)
    {
        return IS_OP_OK;
    }

    return IS_OP_ERROR;
}

uint8_t cISBootloaderSONY::check_is_compatible(uint32_t imgSign)
{
    int count = 0;

    // TODO: Do we need to erase the chip first?

    // Restart chip in special mode
    uint8_t msg[2] = { 0xFF, 0x00 };
    if(send_msg(CXD_SET_STATUS, msg, 1U, 5000U) != 0 || m_data[0] != 0x00)
    {
        m_info_callback(this, "Could not restart CXD chip into bootloader mode", IS_LOG_LEVEL_ERROR);
        return IS_IMAGE_SIGN_NONE;
    }

    return IS_IMAGE_SIGN_SONY_CXD5610;
}

is_operation_result cISBootloaderSONY::download_image(void)
{
    uint8_t isUpdater = 0;
    uint8_t type = 0xFF;
    uint8_t extension = 0xFF;

    if(strstr(image.c_str(), "sdk."))
    {
        type = 0x00;
    }
    else if(strstr(image.c_str(), "app."))
    {
        type = 0x01;
    }
    else if(strstr(image.c_str(), "lib."))
    {
        type = 0x02;
    }
    else if(strstr(image.c_str(), ".cfg"))
    {
        type = 0x03;
    }
    else if(strstr(image.c_str(), "updater."))
    {
        isUpdater = 1;  // Updater uses separate process for loading
    }
    else
    {
        return IS_OP_INCOMPATIBLE;
    }

    if(strstr(image.c_str(), ".efpk"))
    {
        extension = 0x00;
    }
    else if(strstr(image.c_str(), ".fpk") || strstr(image.c_str(), ".cfg"))
    {
        extension = 0x01;
    }
    else 
    {
        return IS_OP_INCOMPATIBLE;
    }

    // Open the image and get its length
    FILE* fw = 0;
    int fileSize;
#ifdef _MSC_VER
    fopen_s(&fw, image.c_str(), "rb");
#else
    fw = fopen(image.c_str(), "rb");
#endif
    fseek(fw, 0, SEEK_END);
    fileSize = ftell(fw);
    fseek(fw, 0, SEEK_SET);

    uint16_t nFrames = (fileSize + 4085) / 4086;    // Round up
    uint16_t frameSn = 0U;
    int len;

    do {
        uint8_t buf[2 + 2 + 4086 + 1] = { 0 };  // Frame SN (2) plus frame count (2) plus max payload size (4086) plus checksum (1)
        
        memcpy(&buf[0], &nFrames, 2);
        memcpy(&buf[2], &frameSn, 2);
        
        int len = read_bytes(fw, &buf[4], &fileSize) + 4;   // Add 4 for frame count and SN at start

        if(len > 0 && send_msg(CXD_PROGRAM_CODE_INJECTION, buf, len, 5000U) != 0)
        {
            m_info_callback(this, "Problem with CXD program injection", IS_LOG_LEVEL_ERROR);
            return IS_OP_ERROR;
        }

        frameSn++;  // Keep track of the frame we are on
    } while(len == 4086);

    uint8_t buf[4] = { type, extension, 0x00, 0x00 };   // 3 byte OPR tells chip where to put firmware, 1 byte for checksum added by send_msg
    if(send_msg(CXD_WRITE_PROGRAM, buf, 3, 5000) != 0)
    {
        m_info_callback(this, "Problem with CXD write program", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }
}

int cISBootloaderSONY::read_bytes(FILE* file, uint8_t line[4086], int *bytesLeft)
{
    uint8_t* currentPtr = line;
    uint8_t* endPtr = currentPtr + 4085;

    while ((currentPtr != endPtr) && bytesLeft)
    {
        *currentPtr++ = (uint8_t)fgetc(file);
        *bytesLeft--;
    }

    return (int)(currentPtr - line);
}


int cISBootloaderSONY::send_msg(uint8_t opcode, uint8_t* data, uint16_t len, uint16_t timeoutMs)
{
	cxd5610_cmd_t head;

	head.hdr.sync = 0x7F;
	head.hdr.opcode = opcode;
	head.hdr.oplen = len;
	head.cksum = checksum((uint8_t*)&head, 4U);
	serialPortWrite(m_port, (const uint8_t*)&head, sizeof(head));

	if(len && data)
	{
		data[len] = checksum(data, len);	// Add checksum to end of message
		len++;
		serialPortWrite(m_port, data, len);
	}

    int responseLen = sizeof(cxd5610_cmd_t) + 2;
    uint8_t buf[7];

    int readLen = serialPortReadTimeout(m_port, buf, responseLen, timeoutMs);

    if (readLen == responseLen &&   // Read right amount of data
        read_header(buf) == 0 &&    // Header is valid
        opcode == m_opcode)         // Opcode matches
    {
        return 0;
    }

    return -1;
}

int cISBootloaderSONY::read_header(uint8_t* buf)
{
    cxd5610_cmd_t* cmd = (cxd5610_cmd_t*)buf;

	if(cmd->hdr.sync != 0x7F) return -1;
	if(cmd->cksum != checksum(buf, 4)) return -1;

	uint16_t oplen = *((uint16_t*)(&buf[1]));
	if(cmd->hdr.oplen > 4086) return -1;     // 4086 is not a typo, max length including header is < 4096

	m_oplen = cmd->hdr.oplen;
	m_opcode = cmd->hdr.opcode;
	memcpy(m_data, &buf[5], m_oplen);

	return 0;
}

uint8_t cISBootloaderSONY::checksum(uint8_t* buf, uint16_t len)
{
	uint8_t checksum = 0x00;
	for (size_t i = 0; i < len; i++) checksum += buf[i];
	return checksum;
}
