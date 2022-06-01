#include "config.h"

config::config() {
    m_rx_type = CRSF;
    m_channel_mapping[0] = 'A';
    m_channel_mapping[1] = 'E';
    m_channel_mapping[2] = 'T';
    m_channel_mapping[3] = 'R';

    stdio_init_all();
}

void print_buf(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        printf("%02x", buf[i]);
        if (i % 16 == 15)
            printf("\n");
        else
            printf(" ");
    }
}
 

config::saveConfig() {
    uint32_t ints = save_and_disable_interrupts();
    const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

    print_buf(flash_target_contents, FLASH_PAGE_SIZE);

    restore_interrupts (ints);
}