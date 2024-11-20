#include <gtest/gtest.h>
#include "gtest_helpers.h"


#include "test_serial_utils.h"

#include "ISBFirmwareUpdater.h"

std::vector<std::string> hex_lines = {
        ":020000040800F2",
        ":1060000000800220ED700308ED3E03082D05030813",
        ":10601000C5040308DD040308F504030800000000BC",
        ":1060200000000000000000000000000081F50208F0",
        ":10603000417103080000000011F6020875F602081D",
        ":10604000417103084171030841710308417103085C",
        ":10605000714F0308B53E03089D4A0308A54A03088B",
        ":10608000554903086D49030841710308C13A0308E3",
        ":10609000417103084171030841710308C54A0308AF",
        ":1060A00041710308417103084171030841710308FC",
        ":1060B000AD6403084171030800000000417103084A",
        ":1060C00041710308417103084171030841710308DC",
        ":1060D00041710308D96A0308096B0308396B030887",
        ":1060E000CD4A0308417103080000000000000000D1",
        ":1060F00000000000000000000000000000000000A0",
        ":106100000000000041710308000000004171030815",
        ":10611000696B0308000000004171030800000000E3",
        ":106120008549030899490308AD490308C54903088F",
        ":020000040801F1",
        ":1000000032AFDFED0B7AF0EE676A46E7AFF30080C0",
        ":1000100000000000D0126341CDCCCC3D6F12833A7A",
        ":1000200017B7D138F0FF0F0000408F402C51060861",
        ":10003000000000007B14AE47E17A843F48AFBC9AD1",
        ":10004000F2D77A3E9A9999999999B93FFCA9F1D238",
        ":100050004D62503F0AD7233CCDCCCC3D6F12833A42",
        ":100060009AF9794468916D3E012940D022F48012BA",
        ":100070000229B2603DD142F48002B260E07E10F00D",
        ":10008000010739D0D5F81CC0BCF1000F06D1A86912",
        ":10009000249FAAEB0000C7F8AC00E07E00F0C0008F",
        ":1000A000402800F0D280802800F0B88022F08062E2",
        ":1000B000B2600027EA6EFFF7C5BB229B002B3FF41E",
        ":1000C000E2ABAC6A636E23B1716806F10800984731",
        ":04011000FF000000EC",
        ":04000005080370ED8F",
        ":00000001FF"

};

std::string portReadAsString(port_handle_t port, int numBytes, int& bytesRead) {
    uint8_t buffer[1024];
    bytesRead = portRead(port, buffer, numBytes);
    EXPECT_EQ(bytesRead, numBytes);
    return std::string((const char*)buffer, bytesRead);
}

void validate_ihex_record(int lineNum, const std::string& record, uint8_t len, uint16_t addr, uint8_t type) {
    uint8_t recLen = std::stoul(record.substr(1, 2), 0, 16);
    uint16_t recAddr = std::stoul(record.substr(3, 4), 0, 16);
    uint8_t recType = std::stoul(record.substr(7, 2), 0, 16);

    int chksum = IhexTransformer::checksum(const_cast<std::string &>(record));
    EXPECT_EQ(recLen, len) << "LineNum " << lineNum << " : Data size of transformed packet is incorrect";
    EXPECT_EQ(recAddr, addr) << "LineNum " << lineNum << " : Record address of transformed packet is incorrect";
    EXPECT_EQ(recType, type) << "LineNum " << lineNum << " : Record type of transformed packet is incorrect";
    EXPECT_EQ(0, chksum) << "LineNum " << lineNum << " : Record failed to validation checksum (checksum() should have returned 0)";
}

TEST(ISBootloader, ihexTransformer)
{
    IhexTransformer packet;
    int lineNum = 0, ackCount = 0;

    initTestPorts();

    portWrite(TEST3_PORT, (const uint8_t*)".\r\n", 3);  // pre-canned response to the first record (op 04)

    auto portEmitter = [](const std::string& record) {
        IhexTransformer::sendRecord(TEST2_PORT, record);
    };

    for (auto line : hex_lines) {
        auto result = packet.processRecord(line, portEmitter);    // we want to use a bridge port here, instead of a loopback, because some of the underlying functionality will perform a portRead()
        switch (result) {
            case IhexTransformer::IHEX_ERROR__FAILED_TO_ACK:         //! the packet was sent, but we failed to receive an ack response ('./r/n`)
                EXPECT_FALSE(true) << "LineNum " << lineNum << " : Failed to receive acknowledgement from remote";
                break;
            case IhexTransformer::IHEX_ERROR__FAILED_TO_SEND:        //! there was an error sending the data to the port
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : Failed to write data to port";
                break;
            case IhexTransformer::IHEX_ERROR__BUFLEN_EXCEEDED:       //! the record data exceeded the available buffer space
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : Unexpected Buffer length exceeded";
                break;
            case IhexTransformer::IHEX_ERROR__LINE_LEN_EXCEEDED:     //! the record data was not a multiple of 2
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : The transformed record length exceeded the protocol limit";
                break;
            case IhexTransformer::IHEX_ERROR__LINE_LEN_MOD:          //! the record data was not a multiple of 2
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : Parsed HEX line had odd number of bytes";
                break;
            case IhexTransformer::IHEX_ERROR__INVALID_CHECKSUM:      //! the record data checksum failed to match
                {
                    std::string sansChecksum = line.substr(0, line.length()-2);
                    int chksum = packet.checksum(sansChecksum);
                    EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : Record checksum mismatch: expected " << chksum;
                }
                break;
            case IhexTransformer::IHEX_ERROR__INVALID_START:         //! the record had an invalid start character ':'
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : An invalid start character was detected (or a valid start character was found)";
                break;
            case IhexTransformer::IHEX_OP_OK:
            case IhexTransformer::IHEX_DATA_READY_TO_SEND:
            default:
                EXPECT_EQ(result, IhexTransformer::IHEX_OP_OK);
                if (portAvailable(TEST2_PORT) == 0) {
                    portWrite(TEST3_PORT, (const uint8_t *) ".\r\n", 3);  // pre-canned response to the eventual sendRecord (op 00)
                    ackCount++;

                    if ((lineNum == 14) || (lineNum == 32)) {
                        portWrite(TEST3_PORT, (const uint8_t *) ".\r\n", 3);  // some operations will call sendPacket() twice, needing a second ACK waiting
                        ackCount++;
                    }
                }
                break;
        }
        lineNum++;
    }

    // now that everything has been sent, lets confirm what was actually sent.
    int bytesRead = 0;
    auto rxPort = COMM_PORT(TEST3_PORT);

    // validate PAGE select (SET EXTENDED ADDRESS)
    std::string record = portReadAsString(rxPort, 15, bytesRead);
    validate_ihex_record(lineNum, record, 0x02, 0x0000, 0x04);

    // validate first WRITE DATA
    record = portReadAsString(rxPort, 491, bytesRead);
    validate_ihex_record(lineNum, record, 0xF0, 0x6000, 0x00);

    // validate second WRITE DATA
    record = portReadAsString(rxPort, 139, bytesRead);
    validate_ihex_record(lineNum, record, 0x40, 0x60F0, 0x00);

    // validate second PAGE select (SET EXTENDED ADDRESS)
    record = portReadAsString(rxPort, 15, bytesRead);
    validate_ihex_record(lineNum, record, 0x02, 0x0000, 0x04);

    // validate third WRITE DATA
    record = portReadAsString(rxPort, 427, bytesRead);
    validate_ihex_record(lineNum, record, 0xD0, 0x0000, 0x00);

    // validate final WRITE DATA
    record = portReadAsString(rxPort, 19, bytesRead);
    validate_ihex_record(lineNum, record, 0x04, 0x0110, 0x00);

    // validate SET EXECUTION ADDR
    record = portReadAsString(rxPort, 19, bytesRead);
    validate_ihex_record(lineNum, record, 0x04, 0x0000, 0x05);

    // validate END OF FILE
    record = portReadAsString(rxPort, 11, bytesRead);
    validate_ihex_record(lineNum, record, 0x00, 0x0000, 0x01);

    // std::string transformedData(&buffer, )
    // packet.checksum()

    EXPECT_EQ(portAvailable(rxPort), 0) << "Expected 0 bytes remaining in RX queue, but bytes remained.";

    EXPECT_TRUE(true);
}

TEST(ISBootloader, ahexTransformer)
{
    int lineNum = 0, ackCount = 0;

    initTestPorts();

    portWrite(TEST3_PORT, (const uint8_t*)".\r\n", 3);  // pre-canned response to the first record (op 04)

    auto portEmitter = [](const std::string& record) {
        IhexTransformer::sendRecord(TEST2_PORT, record);
    };

    for (auto line : hex_lines) {
        auto result = packet.processRecord(line, portEmitter);    // we want to use a bridge port here, instead of a loopback, because some of the underlying functionality will perform a portRead()
        switch (result) {
            case IhexTransformer::IHEX_ERROR__FAILED_TO_ACK:         //! the packet was sent, but we failed to receive an ack response ('./r/n`)
                EXPECT_FALSE(true) << "LineNum " << lineNum << " : Failed to receive acknowledgement from remote";
                break;
            case IhexTransformer::IHEX_ERROR__FAILED_TO_SEND:        //! there was an error sending the data to the port
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : Failed to write data to port";
                break;
            case IhexTransformer::IHEX_ERROR__BUFLEN_EXCEEDED:       //! the record data exceeded the available buffer space
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : Unexpected Buffer length exceeded";
                break;
            case IhexTransformer::IHEX_ERROR__LINE_LEN_EXCEEDED:     //! the record data was not a multiple of 2
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : The transformed record length exceeded the protocol limit";
                break;
            case IhexTransformer::IHEX_ERROR__LINE_LEN_MOD:          //! the record data was not a multiple of 2
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : Parsed HEX line had odd number of bytes";
                break;
            case IhexTransformer::IHEX_ERROR__INVALID_CHECKSUM:      //! the record data checksum failed to match
            {
                std::string sansChecksum = line.substr(0, line.length()-2);
                int chksum = packet.checksum(sansChecksum);
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : Record checksum mismatch: expected " << chksum;
            }
                break;
            case IhexTransformer::IHEX_ERROR__INVALID_START:         //! the record had an invalid start character ':'
                EXPECT_FALSE(true) << "LineNum \" << lineNum << \" : An invalid start character was detected (or a valid start character was found)";
                break;
            case IhexTransformer::IHEX_OP_OK:
            case IhexTransformer::IHEX_DATA_READY_TO_SEND:
            default:
                EXPECT_EQ(result, IhexTransformer::IHEX_OP_OK);
                if (portAvailable(TEST2_PORT) == 0) {
                    portWrite(TEST3_PORT, (const uint8_t *) ".\r\n", 3);  // pre-canned response to the eventual sendRecord (op 00)
                    ackCount++;

                    if ((lineNum == 14) || (lineNum == 32)) {
                        portWrite(TEST3_PORT, (const uint8_t *) ".\r\n", 3);  // some operations will call sendPacket() twice, needing a second ACK waiting
                        ackCount++;
                    }
                }
                break;
        }
        lineNum++;
    }

    // now that everything has been sent, lets confirm what was actually sent.
    int bytesRead = 0;
    auto rxPort = COMM_PORT(TEST3_PORT);

    // validate PAGE select (SET EXTENDED ADDRESS)
    std::string record = portReadAsString(rxPort, 15, bytesRead);
    validate_ihex_record(lineNum, record, 0x02, 0x0000, 0x04);

    // validate first WRITE DATA
    record = portReadAsString(rxPort, 491, bytesRead);
    validate_ihex_record(lineNum, record, 0xF0, 0x6000, 0x00);

    // validate second WRITE DATA
    record = portReadAsString(rxPort, 139, bytesRead);
    validate_ihex_record(lineNum, record, 0x40, 0x60F0, 0x00);

    // validate second PAGE select (SET EXTENDED ADDRESS)
    record = portReadAsString(rxPort, 15, bytesRead);
    validate_ihex_record(lineNum, record, 0x02, 0x0000, 0x04);

    // validate third WRITE DATA
    record = portReadAsString(rxPort, 427, bytesRead);
    validate_ihex_record(lineNum, record, 0xD0, 0x0000, 0x00);

    // validate final WRITE DATA
    record = portReadAsString(rxPort, 19, bytesRead);
    validate_ihex_record(lineNum, record, 0x04, 0x0110, 0x00);

    // validate SET EXECUTION ADDR
    record = portReadAsString(rxPort, 19, bytesRead);
    validate_ihex_record(lineNum, record, 0x04, 0x0000, 0x05);

    // validate END OF FILE
    record = portReadAsString(rxPort, 11, bytesRead);
    validate_ihex_record(lineNum, record, 0x00, 0x0000, 0x01);

    // std::string transformedData(&buffer, )
    // packet.checksum()

    EXPECT_EQ(portAvailable(rxPort), 0) << "Expected 0 bytes remaining in RX queue, but bytes remained.";

    EXPECT_TRUE(true);
}


