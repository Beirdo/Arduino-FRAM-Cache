#ifndef FRAM_CACHE_H__
#define FRAM_CACHE_H__

#include <inttypes.h>
#include <Adafruit_FRAM_SPI.h>

class FRAM_Cache;

typedef enum {
    SET_BITS,
    CLEAR_BITS,
    TOGGLE_BITS,
} oper_t;

class Cache_Segment {
    public:
        Cache_Segment(Adafruit_FRAM_SPI *fram, uint16_t start_addr,
                      uint16_t cache_size, uint16_t buffer_size,
                      uint16_t page_size, uint8_t *buffer_ = NULL,
                      bool circular = false);

        uint8_t read(uint16_t addr);
        void write(uint16_t addr, uint8_t value);
        void oper(uint16_t addr, oper_t oper_, uint8_t value);
        void setWriteProtect(bool enable) { m_write_protected = enable; };
        void clear(void);
        uint8_t *buffer(void) { return m_buffer; };
        bool initialized(void) { return m_initialized; };
        void flushCacheLine(void);

        uint16_t circularReadAvailable(void);
        uint16_t circularRead(uint8_t *buffer_, uint16_t maxlen,
                              bool terminate = true);
        uint16_t circularFind(const char *findstr);
        uint16_t circularWriteAvailable(void);
        uint16_t circularWrite(uint8_t *buffer_, uint16_t len);

    protected:
        Adafruit_FRAM_SPI *m_fram;
        bool m_initialized;
        bool m_circular;

        uint16_t m_device_size;
        uint16_t m_start_addr;
        uint16_t m_cache_size;
        uint16_t m_cache_mask;
        uint16_t m_buffer_size;
        uint16_t m_buffer_mask;
        uint16_t m_page_size;
        bool m_write_protected;
        uint8_t *m_buffer;

        uint32_t m_empty;
        bool m_clean;
        uint16_t m_curr_addr;

        uint16_t m_head;        // Where to write if circular
        uint16_t m_tail;        // Where to read if circular

        uint32_t getPageBit(uint16_t addr);
        void getCacheLine(uint16_t line_addr);
};

#endif
// vim:ts=4:sw=4:ai:et:si:sts=4

