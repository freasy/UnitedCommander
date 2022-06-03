#pragma once

#define FLASH_TARGET_OFFSET (1 * 1024 * 1024)

#include <cstring>
#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"

enum eRxType { CRSF, SBUS };

struct uc_config {
    char mapping[4];
    eRxType rxType;
};

class config {
public:
    uc_config getConfig() {
        uc_config l_config;
        memcpy((void*)&l_config.mapping, (void*)&m_channel_mapping, 4 * sizeof(char));
        l_config.rxType = m_rx_type;
        return l_config;
    }

    void saveConfig();

    void readConfig();
private:
    char m_channel_mapping[4];
    eRxType m_rx_type;
};
