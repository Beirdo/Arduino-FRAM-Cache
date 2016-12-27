#include "Arduino.h"
#include <Adafruit_FRAM_SPI.h>
#include "FRAM_Cache.h"
#include <string.h>

typedef struct {
    uint8_t manufID;
    uint16_t prodID;
    uint32_t size;
} fram_type_t;

fram_type_t fram_types[] = {
    { 0x04, 0x0302, 8192 },
    { 0x00, 0x0000, 0 },
};

Cache_Segment::Cache_Segment(Adafruit_FRAM_SPI *fram, uint16_t start_addr,
                             uint16_t cache_size, uint16_t buffer_size,
                             uint16_t page_size, uint8_t *buffer_,
                             bool circular)
{
    m_fram = fram;

    m_initialized = false;
    m_device_size = 0;
    uint8_t manufID;
    uint16_t prodID;
    fram->getDeviceID(&manufID, &prodID);
    for (uint8_t i = 0; fram_types[i].size != 0; i++) {
        if (fram_types[i].manufID == manufID &&
            fram_types[i].prodID == prodID) {
            m_device_size = fram_types[i].size;
            break;
        }
    }

    if (m_device_size == 0) {
        return;
    }

    if (start_addr + cache_size > m_device_size) {
        return;
    }

    m_start_addr = start_addr;
    m_cache_size = cache_size;
    m_cache_mask = cache_size - 1;

    m_write_protected = false;      // Start off writable

    if (buffer_size > cache_size) {
        return;
    }

    if (page_size > buffer_size) {
        return;
    }

    m_page_size = page_size;
    m_buffer_size = buffer_size;
    if (buffer_) {
        m_buffer = buffer_;
    } else {
        m_buffer = new uint8_t[buffer_size];
    }
    m_buffer_mask = buffer_size - 1;

    m_circular = circular;
    m_head = 0;
    m_tail = 0;

    m_empty = 0;
    m_clean = true;
    m_curr_addr = 0xFFFF;
    m_initialized = true;
}

uint8_t Cache_Segment::read(uint16_t addr)
{
    if (!m_initialized) {
        return 0;
    }

    uint16_t line_addr = addr & ~m_buffer_mask;
    if (line_addr != m_curr_addr) {
        getCacheLine(line_addr);
    }

    uint16_t offset = addr & m_buffer_mask;
    return m_buffer[offset];
}

void Cache_Segment::write(uint16_t addr, uint8_t value)
{
    if (!m_initialized || m_write_protected) {
        return 0;
    }

    uint16_t line_addr = addr & ~m_buffer_mask;
    if (line_addr != m_curr_addr) {
        getCacheLine(line_addr);
    }

    uint16_t offset = addr & m_buffer_mask;
    m_buffer[offset] = value;
    m_clean = false;

    uint32_t page_bit = getPageBit(addr);
    m_empty &= ~page_bit;
}

void Cache_Segment::oper(uint16_t addr, oper_t oper_, uint8_t value)
{
    uint8_t data = read(addr);

    switch (oper_) {
        case SET_BITS:
            data |= value;
            break;
        case CLEAR_BITS:
            data &= ~value;
            break;
        case TOGGLE_BITS:
            data ^= value;
            break;
        default:
            break;
    }

    write(addr, data);
}

uint32_t Cache_Segment::getPageBit(uint16_t addr)
{
    uint16_t bitNum = addr / m_page_size;
    return (uint32_t)(1L << (bitNum & 0x1F));
}

void Cache_Segment::clear(void)
{
    m_empty = 0xFFFFFFFFL;
    m_clean = true;
    m_curr_addr = 0xFFFF;
    m_head = 0;
    m_tail = 0;
}

void Cache_Segment::flushCacheLine(void)
{
    if (m_write_protected || m_clean) {
        return;
    }

    m_fram->writeEnable(true);
    m_fram->write(m_start_addr + m_curr_addr, m_buffer, m_buffer_size);
    m_clean = true;
}

void Cache_Segment::getCacheLine(uint16_t line_addr)
{
    // Need to load the cache from the FRAM, flushing first if needed
    if (!m_clean) {
        flushCacheLine();
    }

    // Do this in page-size chunks as we track cleared buffer by pages
    uint16_t last_page = line_addr + m_buffer_size;
    for (uint16_t page_addr = line_addr; page_addr < last_page;
         page_addr += m_page_size) {
        uint32_t page_bit = getPageBit(page_addr);
        bool empty = !(!(m_empty & page_bit));
        uint16_t buff_addr = page_addr & m_buffer_mask;

        if (!empty) {
            m_fram->read(m_start_addr + page_addr, &m_buffer[buff_addr],
                         m_page_size);
        } else {
            memset(&m_buffer[buff_addr], 0x00, m_page_size);
        }
    }

    m_clean = true;
    m_curr_addr = line_addr;
}

uint16_t Cache_Segment::circularReadAvailable(void)
{
    return (m_tail + m_cache_size - m_head) & m_cache_mask;
}

uint16_t Cache_Segment::circularRead(uint8_t *buffer_, uint16_t maxlen,
                                     bool terminate)
{
    if (!m_circular)
    {
        return 0;
    }

    if (terminate) {
        maxlen--;
    }

    uint16_t avail = circularReadAvailable();
    avail = min(avail, maxlen);
    uint16_t len = 0;

    for (uint16_t i = 0; i < avail; i += len) {
        len = min(avail - i, m_buffer_size);
        len -= (m_tail & m_buffer_mask);

        getCacheLine(m_tail & ~m_buffer_mask);

        memcpy(&buffer_[i], m_buffer, len);
        m_tail += len;
        m_tail &= m_cache_mask;
    }

    if (terminate) {
        buffer_[avail] = '\0';
    }

    return avail;
}

uint16_t Cache_Segment::circularWriteAvailable(void)
{
    return (m_head + m_cache_size - 1 - m_tail) & m_cache_mask;
}

uint16_t Cache_Segment::circularWrite(uint8_t *buffer_, uint16_t len)
{
    if (!m_circular)
    {
        return 0;
    }

    int16_t wrlen = (int16_t)circularWriteAvailable();
    if (len > wrlen) {
        return 0;
    }

    for (uint16_t i = 0; i < len; i++) {
        write(m_head, buffer_[i]);
        m_head += 1;
        m_head &= m_cache_mask;
    }

    flushCacheLine();
    return len;
}

bool Cache_Segment::circularFind(const char *findstr)
{
    if (!m_circular)
    {
        return false;
    }

    int16_t total = (int16_t)circularReadAvailable();
    int16_t len = 0;
    int16_t findlen = strlen(findstr);

    char *searchstr = findstr;
    int16_t searchlen = findlen;
    uint16_t loc = 0;

    for (int16_t i = 0; i < total; i += len) {
        len = min(total - i, m_buffer_size);
        len -= (m_tail & m_buffer_mask);

        getCacheLine((m_tail + i) & ~m_buffer_mask);

        char *found = memmem(m_buffer, len, searchstr, searchlen)
        if (found) {
            loc = (m_tail + i + m_buffer + m_cache_size - found) & m_cache_mask;
            len = (loc + m_cache_size - m_tail + findlen) & m_cache_mask;
            return len
        }

        char *searchsubstr = searchstr;
        char *substr = memchr(m_buffer, *searchsubstr++, len);
        while (substr) {
            uint8_t matched = 0;
            uint8_t remlen = len - (substr - m_buffer);
            uint8_t sublen = remlen;
            char *searchsub = substr;
            do {
                matched++;
                searchsub++;
                if (remlen == 1) {
                    searchlen -= matched;
                    break;
                }
                
                if (*searchsub != *searchsubstr++) {
                    searchsubstr = searchstr;
                    searchlen = findlen;
                    substr = memchr(&substr[1], *searchsubstr++, sublen - 1);
                    remlen = len - (substr - m_buffer);
                    sublen = remlen;
                    break;
                }

                remlen--;
            } while (remlen);

            if (remlen == 1) {
                break;
            }
        }

        if (!substr) {
            searchstr = findstr;
            searchlen = findlen;
        }
    }

    return 0;
}

// vim:ts=4:sw=4:ai:et:si:sts=4
