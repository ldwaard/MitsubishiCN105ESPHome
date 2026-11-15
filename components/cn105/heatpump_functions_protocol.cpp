#include "heatpumpFunctions.h"
#include <cstring>

heatpumpFunctions::heatpumpFunctions() {
    clear();
}

bool heatpumpFunctions::isValid() const {
    return _isValid1 && _isValid2;
}

void heatpumpFunctions::setData1(const uint8_t* data) {
    memcpy(raw, data, 15);
    _isValid1 = true;
}

void heatpumpFunctions::setData2(const uint8_t* data) {
    memcpy(raw + 15, data, 15);
    _isValid2 = true;
}

void heatpumpFunctions::getData1(uint8_t* data) const {
    memcpy(data, raw, 15);
}

void heatpumpFunctions::getData2(uint8_t* data) const {
    memcpy(data, raw + 15, 15);
}

void heatpumpFunctions::clear() {
    memset(raw, 0, sizeof(raw));
    _isValid1 = false;
    _isValid2 = false;
}

int heatpumpFunctions::getCode(uint8_t b) {
    return ((b >> 2) & 0xff) + 100;
}

int heatpumpFunctions::getValue(uint8_t b) {
    return b & 3;
}

int heatpumpFunctions::getValue(int code) {
    if (code > 128 || code < 101)
        return 0;

    for (int i = 0; i < MAX_FUNCTION_CODE_COUNT; ++i) {
        if (getCode(raw[i]) == code)
            return getValue(raw[i]);
    }

    return 0;
}

bool heatpumpFunctions::setValue(int code, int value) {
    if (code > 128 || code < 101)
        return false;

    if (value < 1 || value > 3)
        return false;

    // Clear all values for a sparse write
    for (int i = 0; i < MAX_FUNCTION_CODE_COUNT; ++i) {
        raw[i] = 0;
    }

    // Mappings based on PAC code order
    if (101 == code && 0 == raw[0]) { raw[0] = ((code - 100) << 2) + value; return true; }
    if (102 == code && 0 == raw[1]) { raw[1] = ((code - 100) << 2) + value; return true; }
    if (103 == code && 0 == raw[2]) { raw[2] = ((code - 100) << 2) + value; return true; }
    if (104 == code && 0 == raw[3]) { raw[3] = ((code - 100) << 2) + value; return true; }
    if (105 == code && 0 == raw[4]) { raw[4] = ((code - 100) << 2) + value; return true; }
    if (106 == code && 0 == raw[5]) { raw[5] = ((code - 100) << 2) + value; return true; }
    if (107 == code && 0 == raw[15]) { raw[15] = ((code - 100) << 2) + value; return true; }
    if (108 == code && 0 == raw[16]) { raw[16] = ((code - 100) << 2) + value; return true; }
    if (109 == code && 0 == raw[17]) { raw[17] = ((code - 100) << 2) + value; return true; }
    if (110 == code && 0 == raw[18]) { raw[18] = ((code - 100) << 2) + value; return true; }
    if (111 == code && 0 == raw[19]) { raw[19] = ((code - 100) << 2) + value; return true; }
    if (112 == code && 0 == raw[20]) { raw[20] = ((code - 100) << 2) + value; return true; }
    if (113 == code && 0 == raw[21]) { raw[21] = ((code - 100) << 2) + value; return true; }
    if (114 == code && 0 == raw[22]) { raw[22] = ((code - 100) << 2) + value; return true; }
    if (115 == code && 0 == raw[6]) { raw[6] = ((code - 100) << 2) + value; return true; }
    if (116 == code && 0 == raw[7]) { raw[7] = ((code - 100) << 2) + value; return true; }
    if (117 == code && 0 == raw[8]) { raw[8] = ((code - 100) << 2) + value; return true; }
    if (118 == code && 0 == raw[9]) { raw[9] = ((code - 100) << 2) + value; return true; }
    if (119 == code && 0 == raw[10]) { raw[10] = ((code - 100) << 2) + value; return true; }
    if (120 == code && 0 == raw[11]) { raw[11] = ((code - 100) << 2) + value; return true; }
    if (121 == code && 0 == raw[12]) { raw[12] = ((code - 100) << 2) + value; return true; }
    if (122 == code && 0 == raw[13]) { raw[13] = ((code - 100) << 2) + value; return true; }
    if (123 == code && 0 == raw[23]) { raw[23] = ((code - 100) << 2) + value; return true; }
    if (124 == code && 0 == raw[24]) { raw[24] = ((code - 100) << 2) + value; return true; }
    if (125 == code && 0 == raw[25]) { raw[25] = ((code - 100) << 2) + value; return true; }
    if (126 == code && 0 == raw[26]) { raw[26] = ((code - 100) << 2) + value; return true; }
    if (127 == code && 0 == raw[27]) { raw[27] = ((code - 100) << 2) + value; return true; }
    if (128 == code && 0 == raw[28]) { raw[28] = ((code - 100) << 2) + value; return true; }

    return false;
}

heatpumpFunctionCodes heatpumpFunctions::getAllCodes() {
    heatpumpFunctionCodes result;
    for (int i = 0; i < MAX_FUNCTION_CODE_COUNT; ++i) {
        int code = getCode(raw[i]);
        result.code[i] = code;
        result.valid[i] = (code >= 101 && code <= 128);
    }

    return result;
}

bool heatpumpFunctions::operator==(const heatpumpFunctions& rhs) {
    return this->isValid() == rhs.isValid() && memcmp(this->raw, rhs.raw, sizeof(this->raw)) == 0;
}

bool heatpumpFunctions::operator!=(const heatpumpFunctions& rhs) {
    return !(*this == rhs);
}
//#endregion heatpump_functions
