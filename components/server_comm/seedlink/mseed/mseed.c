/*---------------------------------------------------------------------------------------------------------------
 * mseed.c
 *
 * Created on: July 22. 2025
 * By: Ravi Tamrakar
 *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_check.h"

#include "mseed.h"

/*---------------------------------------------------------------------------------------------------------------
 * MACROS AND DEFINES
 *--------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------
 * EXTERN PUBLIC VARIABLES
 *--------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------
 * EXTERN PUBLIC FUNCTION DECLARATIONS
 *--------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE STRUCTURES
 *--------------------------------------------------------------------------------------------------------------*/

typedef struct BLOCKETTE1000
{
    uint16_t type;
    uint16_t next_blockette;
    uint8_t encoding;
    uint8_t byte_order;
    uint8_t rec_len;
    uint8_t reserved;
} blockette1000_t;

typedef struct MSEED_HEADER // MiniSEED 2.4
{
    char sequence_number[6];
    char data_quality;
    char reserved;
    char station[5];
    char location[2];
    char channel[3];
    char network[2];
    uint16_t year;
    uint16_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t unused;
    uint16_t fract;
    uint16_t numsamples;
    int16_t samprate_fact;
    int16_t samprate_mult;
    uint8_t act_flags;
    uint8_t io_flags;
    uint8_t dq_flags;
    uint8_t numblockettes;
    int32_t time_correct;
    uint16_t data_offset;
    uint16_t blockette_offset;
} mseed_header_t;

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE FUNCTION DECLARATIONS
 *--------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE VARIABLES
 *--------------------------------------------------------------------------------------------------------------*/
#if 0
static const char *LOG_TAG = "[mseed]";
#endif
uint8_t g_mseed_pkt[512] = {0};
uint8_t g_dataLink_pkt[584] = {0};

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE INLINE FUNCTION DEFINITIONS
 *--------------------------------------------------------------------------------------------------------------*/

static int16_t _BEi16(int16_t arg)
{
    uint16_t u = (uint16_t)arg;

    u = (uint16_t)(((u & 0x00FFu) << 8) | ((u & 0xFF00u) >> 8));
    return (int16_t)u;
}

static int32_t _BEi32(int32_t arg)
{
    uint32_t u = (uint32_t)arg;

    u = ((u & 0x000000FFu) << 24) |
        ((u & 0x0000FF00u) << 8) |
        ((u & 0x00FF0000u) >> 8) |
        ((u & 0xFF000000u) >> 24);

    return (int32_t)u;
}

static uint16_t _BEu16(uint16_t arg)
{
    return (uint16_t)(((arg & 0x00FFu) << 8) | ((arg & 0xFF00u) >> 8));
}

#if 0
static uint32_t __attribute__((unused)) _BEu32(uint32_t arg)
{
    return ((arg & 0x000000FFu) << 24) |
           ((arg & 0x0000FF00u) << 8) |
           ((arg & 0x00FF0000u) >> 8) |
           ((arg & 0xFF000000u) >> 24);
}

static uint32_t __attribute__((unused)) _BEf32(float param)
{
    uint32_t u = 0;

    memcpy(&u, &param, sizeof(u));
    return _BEu32(u);
}
#endif

/*---------------------------------------------------------------------------------------------------------------
 * PUBLIC FUNCTION DEFINITIONS
 *--------------------------------------------------------------------------------------------------------------*/

uint8_t *mseed_record(float *p_data, mseed_config_t *sp_cfg)
{
    const uint16_t c_sample_count = sp_cfg->sample_count;
    const uint16_t c_data_size = sp_cfg->sample_count * 4; // sizeof(uint32_t);

    uint64_t start_time = sp_cfg->start_time;
    uint16_t time_fract = sp_cfg->time_fract;

    char sequence[7];
    sprintf(sequence, "%06ld", sp_cfg->sequence_number);

    struct tm s_tm;
    memcpy(&s_tm, gmtime((const time_t *)&start_time), sizeof(struct tm)); // must use utc time to send data in server

    mseed_header_t s_header =
        {
            {'0', '0', '0', '0', '0', '0'}, // sequence_number
            'D',                            // data_quality
            ' ',                            // reserved
            {' ', ' ', ' ', ' ', ' '},      // station
            {' ', ' '},                     // location
            {' ', ' ', ' '},                // channel
            {' ', ' '},                     // network
            _BEu16(s_tm.tm_year + 1900),    // year
            _BEu16(s_tm.tm_yday + 1),       // day
            s_tm.tm_hour,                   // hour
            s_tm.tm_min,                    // min
            s_tm.tm_sec,                    // sec
            0,                              // unused
            _BEu16(time_fract),             // fract
            _BEu16(c_sample_count),         // numsamples
            _BEi16(100),                    // samprate_fact
            _BEi16(1),                      // samprate_mult
            0,                              // act_flags
            0,                              // io_flags
            0,                              // dq_flags
            1,                              // numblockettes
            _BEi32(0),                      // time_correct
            _BEu16(0x38),                   // data_offset;
            _BEu16(0x30),                   // blockette_offset
        };

    blockette1000_t blk1000 =
        {
            _BEu16(1000), // type
            _BEu16(0),    // next_blockette
            4,            // encoding
            0,            // byte_order 0: little endian; 1: big endian
            9,            // rec_len
            0             // reserved
        };

    memcpy(s_header.sequence_number, sequence, 6);
    memcpy(s_header.station, sp_cfg->station, 5);
    memcpy(s_header.location, sp_cfg->location, 2);
    memcpy(s_header.channel, sp_cfg->channel, 3);
    memcpy(s_header.network, sp_cfg->network, 2);

    memset(g_mseed_pkt, 0, sizeof(g_mseed_pkt));
    memcpy(g_mseed_pkt, (uint8_t *)&s_header, sizeof(mseed_header_t));
    memcpy(&g_mseed_pkt[48], (uint8_t *)&blk1000, sizeof(blockette1000_t));
    memcpy(&g_mseed_pkt[56], (uint8_t *)p_data, c_data_size);

    return g_mseed_pkt;
}

uint8_t *mseed_datalink(uint8_t *p_record, mseed_config_t *sp_cfg)
{
    char header[64] = {0};
    // SeedLink Header Format:
    // NP_RTS01_00_HNZ/MSEED 1753350232000000 1753350233000000 N 512
    const char *format = " %-2.2s_%-5.5s_%-2.2s_%-3.3s/MSEED %llu000000 %llu000000 N 512 ";

    uint8_t preHeader[] = {68, 76, (5 + 63), 87, 82, 73, 84, 69};
    uint64_t start_time = sp_cfg->start_time;

    sprintf(header, format,
            sp_cfg->network,
            sp_cfg->station,
            sp_cfg->location,
            sp_cfg->channel,
            start_time, start_time + 1);

    memset(g_dataLink_pkt, 0, sizeof(g_dataLink_pkt));
    memcpy(g_dataLink_pkt, preHeader, 8);
    memcpy(&g_dataLink_pkt[8], header, 63);
    memcpy(&g_dataLink_pkt[8 + 63], p_record, 512);
#if 0
    ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, g_dataLink_pkt, 8 + 63 + 512, ESP_LOG_INFO);
    ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, p_record, 512, ESP_LOG_INFO);
#endif
    return g_dataLink_pkt;
}

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE FUNCTION DEFINITIONS
 *--------------------------------------------------------------------------------------------------------------*/