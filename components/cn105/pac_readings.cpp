#include "cn105.h"

using namespace esphome;

/**
 * Seek the byte pointer to the beginning of the array
 * Initializes few variables
*/
void CN105Climate::initBytePointerPac() {
    this->foundStartPac = false;
    this->bytesReadPac = 0;
    this->dataLengthPac = -1;
    this->commandPac = 0;
}

/**
 *
* The total size of a frame is made up of several elements:
  * Header size: The header has a fixed length of 5 bytes (INFOHEADER_LEN).
  * Data Length: The data length is variable and is specified by the fourth byte of the header (header[4]).
  * Checksum: There is 1 checksum byte at the end of the frame.
  *
  * The total size of a frame is therefore the sum of these elements: header size (5 bytes) + data length (variable) + checksum (1 byte).
  * To calculate the total size, we can use the formula:
  * Total size = 5 (header) + Data length + 1 (checksum)
  *The total size depends on the specific data length for each individual frame.
 */
void CN105Climate::parsePac(uint8_t inputData) {

    ESP_LOGV("Decoder", "--> %02X [nb: %d]", inputData, this->bytesReadPac);

    if (!this->foundStartPac) {                // no packet yet
        if (inputData == HEADER[0]) {
            this->foundStartPac = true;
            this->bytesReadPac = 0;
            storedInputDataPac[this->bytesReadPac++] = inputData;
        } else {
            // unknown bytes
        }
    } else {                                // we are getting a packet
        if (this->bytesReadPac >= (MAX_DATA_BYTES - 1)) {
            ESP_LOGW("Decoder", "buffer overflow preventive reset (bytesRead=%d)", this->bytesReadPac);
            this->initBytePointerPac();
            return;
        }
        storedInputDataPac[this->bytesReadPac] = inputData;

        checkHeaderPac(inputData);

        if (this->dataLengthPac != -1) {       // is header complete ?
            if ((this->dataLengthPac + 6) > MAX_DATA_BYTES) {
                ESP_LOGW("Decoder", "declared data length %d too large, resetting parser", this->dataLengthPac);
                this->initBytePointerPac();
                return;
            }

            if ((this->bytesReadPac) == this->dataLengthPac + 5) {

                this->processDataPacketPac();
                this->initBytePointerPac();
            } else {                                        // packet is still filling
                this->bytesReadPac++;                          // more data to come
            }
        } else {
            ESP_LOGV("Decoder", "data length toujours pas connu");
            // header is not complete yet
            this->bytesReadPac++;
        }
    }

}

bool CN105Climate::checkSumPac() {
    // TODO: use the CN105Climate::checkSum(byte bytes[], int len) function

    uint8_t packetCheckSum = storedInputDataPac[this->bytesReadPac];
    uint8_t processedCS = 0;

    ESP_LOGV("chkSum", "controling chkSum should be: %02X ", packetCheckSum);

    for (int i = 0;i < this->dataLengthPac + 5;i++) {
        ESP_LOGV("chkSum", "adding %02X to %03X --> %X", this->storedInputDataPac[i], processedCS, processedCS + this->storedInputDataPac[i]);
        processedCS += this->storedInputDataPac[i];
    }

    processedCS = (0xfc - processedCS) & 0xff;

    if (packetCheckSum == processedCS) {
        ESP_LOGD("chkSum", "OK-> %02X=%02X ", processedCS, packetCheckSum);
    } else {
        ESP_LOGW("chkSum", "KO-> %02X!=%02X ", processedCS, packetCheckSum);
    }

    return (packetCheckSum == processedCS);
}

void CN105Climate::checkHeaderPac(uint8_t inputData) {
    if (this->bytesReadPac == 4) {
        if (storedInputDataPac[2] == HEADER[2] && storedInputDataPac[3] == HEADER[3]) {
            ESP_LOGV("Header", "header matches HEADER");
            ESP_LOGV("Header", "[%02X] (%02X) %02X %02X [%02X]<-- header", storedInputDataPac[0], storedInputDataPac[1], storedInputDataPac[2], storedInputDataPac[3], storedInputDataPac[4]);
            ESP_LOGD("Header", "command: (%02X) data length: [%02X]<-- header", storedInputDataPac[1], storedInputDataPac[4]);
            this->commandPac = storedInputDataPac[1];
        }
        this->dataLengthPac = storedInputDataPac[4];
    }
}

bool CN105Climate::processInputPac(void) {
    bool processed = false;
    while (this->get_pac_uart_()->available()) {
        processed = true;
        uint8_t inputData;
        if (this->get_pac_uart_()->read_byte(&inputData)) {
            parsePac(inputData);
        }

    }
    return processed;
}

void CN105Climate::processDataPacketPac() {

    ESP_LOGV(TAG, "processing data packet...");

    this->dataPac = &this->storedInputDataPac[5];

    this->hpPacketDebug(this->storedInputDataPac, this->bytesReadPac + 1, "PAC_READ");

    if (this->checkSumPac()) {
        // checkPoint of a heatpump response
        this->lastResponseMsPac = CUSTOM_MILLIS;    //esphome::CUSTOM_MILLIS;

        // processing the specific command
        // processCommandPac();
    }
}
