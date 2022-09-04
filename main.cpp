#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <SPI.h>


//#define DEBUG_PRINT_ENCODE
//#define DEBUG_PRINT_DECODE

#define CC1101_GDO0        (26)
#define CC1101_GDO2        (25)

#define HSPI_CS            (15)
#define HSPI_SCLK        (14) // for doc
#define HSPI_MISO        (12) // for doc
#define HSPI_MOSI        (13) // for doc


#define TEST_INPUT        (17)

#define SIGNAL_CLEAR_CHANNEL_ASSESMENT    CC1101_GDO0
#define SYNCWORD_DET_SENT_TX_SENT        CC1101_GDO2

#define NUM_OF_REMOTES              (3)
#define REMOTE_ADDR_LEN             (3)
#define NUM_OF_BLINDS_PER_REMOTE    (5) // they have 5 ch
#define BLIND_ADDR_LEN              (3)

static const int spiClk = 500000;
byte address = 0x00;
SPIClass * hspi = NULL;

static const uint8_t flash_table_encode[] = {0x08, 0x02, 0x0d, 0x01, 0x0f, 0x0e, 0x07, 0x05, 0x09, 0x0c, 0x00, 0x0a, 0x03, 0x04, 0x0b, 0x06};
static const uint8_t flash_table_decode[] = {0x0a, 0x03, 0x01, 0x0c, 0x0d, 0x07, 0x0f, 0x06, 0x00, 0x08, 0x0b, 0x0e, 0x09, 0x02, 0x05, 0x04};

static uint8_t  gIndex[3] = {1, 1, 1};

static uint8_t remote_addr[NUM_OF_REMOTES][REMOTE_ADDR_LEN] = {
        {0x62, 0x0B, 0x0D}, // idx0 = NAPPALI
        {0x1A, 0x01, 0x0D}, // idx1 = GYSZOBA
        {0x00, 0x00, 0x00}, // idx2 = HALO
};

static uint8_t remote_blind_id[NUM_OF_REMOTES][NUM_OF_BLINDS_PER_REMOTE][BLIND_ADDR_LEN] = {
        { {0xB0, 0xA4, 0x35}, {0xBE, 0xA3, 0x35}, {0x9F, 0xA4, 0x35}, {0xC3, 0xA3, 0x35}, {0x00, 0x00, 0x00} },
        { {0xC2, 0xA3, 0x35}, {0xDF, 0xAC, 0x2A}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00} },
        { {0xC6, 0xA3, 0x35}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00} },
};

static uint8_t msg_buffer[64];
static uint8_t remotes[32][3];

uint8_t scan_check_if_addr_remote(uint8_t* msg)
{
    uint8_t all_match = 1;

    if( memcmp(&msg[6], &msg[9], 3) )
    {
        all_match = 0;
    }

    if( memcmp(&msg[6], &msg[12], 3) )
    {
        all_match = 0;
    }

    if( memcmp(&msg[9], &msg[12], 3) )
    {
        all_match = 0;
    }

    return all_match;
}

void scan_remote_add(uint8_t* msg)
{
    uint32_t i;
    uint8_t empty[3] = {0, 0, 0};

    for( i = 0; i < 32; i++ )
    {
        if( (memcmp(&remotes[i][0], empty, 3) == 0) && (memcmp(&remotes[i][0], empty, 3) == 0) && (memcmp(&remotes[i][0], empty, 3) == 0) )
        {
            break;
        }

        if( memcmp(&remotes[i][0], &msg[6], 3) == 0 )
        {
            i = 60;
            break;
        }
    }

    if( i < 32 )
    {
        memcpy(&remotes[i][0], &msg[6], 3);
    }
}

static void print_msg(uint8_t* msg)
{
    uint8_t i;

    for( i = 0; i < 8 ; i++ )
    {
        Serial.printf("0x%02X ", msg[i]);
    }
}

static uint8_t count_bits(uint8_t byte)
{
    uint8_t i;
    uint8_t ones = 0;
    uint8_t mask = 1;

    for( i = 0; i < 8; i++ )
    {
        if( mask & byte )
        {
            ones += 1;
        }

        mask <<= 1;
    }

    return ones & 0x01;
}

static void calc_parity(uint8_t* msg)
{
    uint8_t i;
    uint8_t p = 0;

    for( i = 0; i < 4; i++ )
    {
        uint8_t a = count_bits( msg[0 + i*2] );
        uint8_t b = count_bits( msg[1 + i*2] );

        p |= a ^ b;
        p <<= 1;
    }

    msg[7] = (p << 3);
}

void add_r20_to_nibbles(uint8_t* msg, uint8_t r20, uint8_t start, uint8_t length)
{
    uint8_t i;

    for( i = 0; i < 8; i++ )
    {
        uint8_t d = msg[i];

        uint8_t ln = (d + r20) & 0x0F;
        uint8_t hn = ((d & 0xF0) + (r20 & 0xF0)) & 0xFF;

        msg[i] = hn | ln;

        r20 = (r20 - 0x22) & 0xFF;
    }
}

void sub_r20_from_nibbles(uint8_t* msg, uint8_t r20, uint8_t start, uint8_t length)
{
    uint8_t i;

    for(i = start; i < length; i++)
    {
        uint8_t d = msg[i];

        uint8_t ln = (d - r20) & 0x0F;
        uint8_t hn = ((d & 0xF0) - (r20 & 0xF0)) & 0xFF;

        msg[i] = hn | ln;

        r20 = (r20 - 0x22) & 0xFF;
    }
}

void xor_2byte_in_array_encode(uint8_t* msg, uint8_t xor0, uint8_t xor1)
{
    uint8_t i;

    for( i = 1; i < 4; i++ )
    {
        msg[i*2 + 0] = msg[i*2 + 0] ^ xor0;
        msg[i*2 + 1] = msg[i*2 + 1] ^ xor1;
    }
}

void xor_2byte_in_array_decode(uint8_t* msg, uint8_t xor0, uint8_t xor1)
{
    uint8_t i;

    for( i = 0; i < 4; i++ )
    {
        msg[i*2 + 0] = msg[i*2 + 0] ^ xor0;
        msg[i*2 + 1] = msg[i*2 + 1] ^ xor1;
    }
}

void encode_nibbles(uint8_t* msg)
{
    uint8_t i;

    for( i = 0; i < 8; i++ )
    {
        uint8_t nh = (msg[i] >> 4) & 0x0F;
        uint8_t nl = msg[i] & 0x0F;

        uint8_t dh = flash_table_encode[nh];
        uint8_t dl = flash_table_encode[nl];

        msg[i] = ((dh << 4) & 0xFF) | ((dl) & 0xFF);
    }
}

void decode_nibbles(uint8_t* msg, uint8_t len)
{
    uint8_t i;

    for( i = 0; i < len; i++ )
    {
        uint8_t nh = (msg[i] >> 4) & 0x0F;
        uint8_t nl = msg[i] & 0x0F;

        uint8_t dh = flash_table_decode[nh];
        uint8_t dl = flash_table_decode[nl];

        msg[i] = ((dh << 4) & 0xFF) | ((dl) & 0xFF);
    }
}

uint8_t calc_exp_parity(uint16_t cnt, uint8_t* msg)
{
    uint16_t num;
    uint8_t input_arr[8];

    num = (0x00 - (cnt * 0x708F)) & 0xFFFF;

    // copy message bytes to a buffer
    memcpy(input_arr, msg, 8);

    // clear parity position
    input_arr[7] = 0;

    // overwrite first 2 with the calculated magic from msg cnt/index
    input_arr[0] = num >> 8;
    input_arr[1] = num & 0xFF;

    calc_parity(input_arr);

    return input_arr[7];
}

void msg_decode(uint8_t* msg)
{
#ifdef DEBUG_PRINT_DECODE
    Serial.print("decode_nibbles: ");
#endif

    decode_nibbles(msg, 8);

#ifdef DEBUG_PRINT_DECODE
    print_msg(msg);
    Serial.println();
    Serial.print("sub_r20_from_nibbles: ");
#endif

    sub_r20_from_nibbles(msg, 0xFE, 0, 2);

#ifdef DEBUG_PRINT_DECODE
    print_msg(msg);
    Serial.println();
    Serial.printf("xor_2byte_in_array: %x %x", msg[0], msg[1]);
#endif

    xor_2byte_in_array_decode(msg, msg[0], msg[1]);

#ifdef DEBUG_PRINT_DECODE
    print_msg(msg);
    Serial.println();
    Serial.print("sub_r20_from_nibbles: ");
#endif

    sub_r20_from_nibbles(msg, 0xBA, 2, 8);

#ifdef DEBUG_PRINT_DECODE
    print_msg(msg);
    Serial.println();
#endif
}

void msg_encode(uint8_t* msg, uint8_t xor0, uint8_t xor1)
{
#ifdef DEBUG_PRINT_ENCODE
    Serial.print("encode: ");
    print_msg(msg);
    Serial.println();
#endif

    calc_parity(msg);

#ifdef DEBUG_PRINT_ENCODE
    Serial.print("parity: ");
    print_msg(msg);
    Serial.println();
#endif

    add_r20_to_nibbles(msg, 0xFE, 0, 8);

#ifdef DEBUG_PRINT_ENCODE
    Serial.print("r20 nibbles: ");
    print_msg(msg);
    Serial.println();
#endif

    xor_2byte_in_array_encode(msg, xor0, xor1);

#ifdef DEBUG_PRINT_ENCODE
    Serial.print("xor nibbles: ");
    print_msg(msg);
    Serial.println();
#endif

    encode_nibbles(msg);

#ifdef DEBUG_PRINT_ENCODE
    Serial.print("encode nibbles: ");
    print_msg(msg);
    Serial.println();
#endif
}

void spi_write_cmd(uint8_t cmd)
{
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);
    hspi->transfer(cmd);
    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();
}

void spi_write_reg(uint8_t addr, uint8_t value)
{
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);
    hspi->transfer(addr);
    hspi->transfer(value);
    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();
}

uint8_t spi_read_reg(uint8_t addr)
{
    uint8_t reg;
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);
    hspi->transfer(addr);
    reg = hspi->transfer(0x00);
    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();

    return reg;
}

void spi_read_burst(uint8_t addr, uint8_t* data, uint8_t len)
{
    uint8_t i;

    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);

    hspi->transfer(addr);

    for( i = 0; i < len; i++ )
    {
        data[i] = hspi->transfer(0x00);
    }

    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();
}

void spi_write_burst(uint8_t addr, uint8_t* data, uint8_t len)
{
    uint8_t i;

    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);

    hspi->transfer(addr);

    for( i = 0; i < len; i++ )
    {
        hspi->transfer(data[i]);
    }

    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();
}

static void cc1100_tx(uint8_t* msg, uint8_t len)
{
    uint8_t i;
    uint8_t marcstate;
    uint32_t ts;
    uint32_t timeout = 1000;

    spi_write_cmd(0x36); // idle for flushing
    spi_write_cmd(0x3A); // flush RX fifo
    spi_write_cmd(0x3B); // flush TX fifo
    spi_write_cmd(0x34); // back to RX for listening clear channel
    
    Serial.print("cc1100_tx...");
    Serial.print("waiting for clear channel...");
    ts = millis();

    //while( (digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW) )
    while( (digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW) && ((millis()-ts) < timeout) );
    {
        delay(5);
    }

    if( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) != LOW )
    {
        Serial.printf("cleared %d\r\n", millis()-ts);
    }
    else
    {
        Serial.printf("TIMEOUT!! %d\r\n", millis()-ts);
        //elero_cc1100_init();
    }

    Serial.println("spi transmit start");
    spi_write_burst(0x7F, msg, len);
    Serial.println("spi transmit end");

    delayMicroseconds(10);

    Serial.println("cc1100_tx.tx");
    spi_write_cmd(0x35);

    Serial.println("cc1100_tx.waiting tx state");


    ts = millis();    do
    {
        marcstate = spi_read_reg(0xF5);
        delay(5);
    }
   //while( marcstate != 0x13 );
    while( (marcstate != 0x13) && ((millis()-ts) < timeout) );

    Serial.printf("cc1100_tx.waiting non tx state %x\r\n", marcstate);


    ts = millis();    do
    {
        marcstate = spi_read_reg(0xF5);
        delay(5);
    }
    //while( (marcstate == 0x13) );
    while( (marcstate == 0x13) && ((millis()-ts) < timeout) );

    Serial.printf("done %x\r\n", marcstate);
}


static void generate_msg_down(uint8_t* msg, uint8_t* addr, uint8_t index, uint8_t channel, uint8_t button_pressed)
{
    uint16_t code;

    memset(msg, 0, 28);

    msg[ 0] = 0x1B;      // msg_len
    msg[ 1] = index;      // pck cnt
    msg[ 2] = 0x44;      // pck_info = STOP
    msg[ 3] = 0x10;      // pck_inf2 = STOP
    msg[ 4] = 0x00;      // hop_info = 0
    msg[ 5] = 0x01;      // sys_addr = 1
    msg[ 6] = (channel == 1)?(0x11):(channel);      // source_group = 0x11

    msg[ 7] = addr[0];   // source addr[0]
    msg[ 8] = addr[1];   // source addr[1]
    msg[ 9] = addr[2];   // source addr[2]

    msg[10] = addr[0];   // backward addr[0]
    msg[11] = addr[1];   // backward addr[1]
    msg[12] = addr[2];   // backward addr[2]

    msg[13] = addr[0];   // forward addr[0]
    msg[14] = addr[1];   // forward addr[1]
    msg[15] = addr[2];   // forward addr[2]

    msg[16] = 0x01;        // dest_count = 1
    msg[17] = (channel == 1)?(0x11):(channel);      // dest = TODO        (ch1=0x11(?) ch2=0x02 ch3=0x03 ch4=0x03)
    msg[18] = 0x00;
    msg[19] = 0x03;

    code = (0x00 - (index * 0x708F)) & 0xFFFF;

    msg[20] = (code >> 8) & 0xFF;
    msg[21] = code & 0xFF;
    msg[22] = (button_pressed) ? (0x40) : (0x00);

    msg_encode(&msg[20], (code >> 8) & 0xFF, code & 0xFF);
}

static void generate_msg_stop(uint8_t* msg, uint8_t* addr, uint8_t index, uint8_t channel, uint8_t* blind_id)
{
    uint16_t code;

    memset(msg, 0, 30);

    msg[ 0] = 0x1D;        // msg_len
    msg[ 1] = index;     // pck cnt
    msg[ 2] = 0x6A;        // pck_info = STOP
    msg[ 3] = 0x10;        // pck_inf2 = STOP
    msg[ 4] = 0x00;        // hop_info = 0
    msg[ 5] = 0x01;        // sys_addr = 1
    msg[ 6] = (channel == 1)?(0x11):(channel);      // source_group = 0x11

    msg[ 7] = addr[0];   // source addr[0]
    msg[ 8] = addr[1];   // source addr[1]
    msg[ 9] = addr[2];   // source addr[2]

    msg[10] = addr[0];   // backward addr[0]
    msg[11] = addr[1];   // backward addr[1]
    msg[12] = addr[2];   // backward addr[2]

    msg[13] = addr[0];   // forward addr[0]
    msg[14] = addr[1];   // forward addr[1]
    msg[15] = addr[2];   // forward addr[2]

    msg[16] = 0x01;        // dest_count = 1
    msg[17] = blind_id[0];        // dest = TODO
    msg[18] = blind_id[1];
    msg[19] = blind_id[2];

    msg[20] = 0x00;
    msg[21] = 0x03;

    code = (0x00 - (index * 0x708F)) & 0xFFFF;

    msg[22] = (code >> 8) & 0xFF;
    msg[23] = code & 0xFF;
    msg[24] = 0x10;

    msg_encode(&msg[22], (code >> 8) & 0xFF, code & 0xFF);
}

static void generate_msg_up(uint8_t* msg, uint8_t* addr, uint8_t index, uint8_t channel, uint8_t button_pressed)
{
    uint16_t code;

    memset(msg, 0, 30);

    msg[ 0] = 0x1B;      // msg_len
    msg[ 1] = index;     // pck cnt
    msg[ 2] = 0x44;      // pck_info
    msg[ 3] = 0x10;  //(index == 0)?(0x12):(0x10);      // pck_inf2
    msg[ 4] = 0x00;      // hop_info = 0
    msg[ 5] = 0x01;      // sys_addr = 1
    msg[ 6] = (channel == 1)?(0x11):(channel);      // source_group = 0x11

    msg[ 7] = addr[0];   // source addr[0]
    msg[ 8] = addr[1];   // source addr[1]
    msg[ 9] = addr[2];   // source addr[2]

    msg[10] = addr[0];   // backward addr[0]
    msg[11] = addr[1];   // backward addr[1]
    msg[12] = addr[2];   // backward addr[2]

    msg[13] = addr[0];   // forward addr[0]
    msg[14] = addr[1];   // forward addr[1]
    msg[15] = addr[2];   // forward addr[2]

    msg[16] = 0x01;        // dest_count = 1
    msg[17] = (channel == 1)?(0x11):(channel);// dest = TODO
    msg[18] = 0x00;
    msg[19] = 0x03;

    code = (0x00 - (index * 0x708F)) & 0xFFFF;

    msg[20] = (code >> 8) & 0xFF;
    msg[21] = code & 0xFF;
    msg[22] = (button_pressed) ? (0x20) : (0x00);

    msg_encode(&msg[20], (code >> 8) & 0xFF, code & 0xFF);
}

void elero_send_msg_down(uint8_t remote_index, uint8_t channel)
{
    uint8_t i;
    uint8_t* r_addr = remote_addr[remote_index];

    generate_msg_down(msg_buffer, r_addr, gIndex[remote_index], channel, 1);
    gIndex[remote_index]++;

    Serial.println("BUTTON DOWN -- PRESS msg generated:");
    for( i = 0; i < 28; i++ )
    {
        Serial.printf("0x%02X ", msg_buffer[i]);
    }
    Serial.println();

    for( i = 0; i < 3; i++ )
    {
        cc1100_tx(msg_buffer, 28);
        delay(10);
    }

    delay(100);

    generate_msg_down(msg_buffer, r_addr, gIndex[remote_index], channel, 0);

    gIndex[remote_index]++;

    Serial.println("BUTTON DOWN -- RELEASE msg generated:");
    for( i = 0; i < 28; i++ )
    {
        Serial.printf("0x%02X ", msg_buffer[i]);
    }
    Serial.println();

    for( i = 0; i < 3; i++ )
    {
        cc1100_tx(msg_buffer, 28);
        delay(10);
    }
}

void elero_send_msg_stop(uint8_t remote_index, uint8_t channel)
{
    uint8_t i;
    uint8_t* r_addr = remote_addr[remote_index];
    uint8_t* blind_id = remote_blind_id[remote_index][channel-1];

    generate_msg_stop(msg_buffer, r_addr, gIndex[remote_index], channel, blind_id);

    gIndex[remote_index]++;

    Serial.println("BUTTON STOP PRESS msg generated:");
    for( i = 0; i < 30; i++ )
    {
        Serial.printf("0x%02X ", msg_buffer[i]);
    }
    Serial.println();

    cc1100_tx(msg_buffer, 30);
    delay(10);

}
void elero_send_msg_up(uint8_t remote_index, uint8_t channel)
{
    uint8_t i;
    uint8_t* r_addr = remote_addr[remote_index];

    generate_msg_up(msg_buffer, r_addr, gIndex[remote_index], channel, 1);

    gIndex[remote_index]++;

    Serial.println("UP: button pressed");
    for( i = 0; i < 28; i++ )
    {
        Serial.printf("0x%02X ", msg_buffer[i]);
    }
    Serial.println();

    for( i = 0; i < 3; i++ )
    {
        cc1100_tx(msg_buffer, 28);
        delay(10);
    }

    delay(100);

    generate_msg_up(msg_buffer, r_addr, gIndex[remote_index], channel, 0);

    gIndex[remote_index]++;

    Serial.println("UP: button release");
    for( i = 0; i < 28; i++ )
    {
        Serial.printf("0x%02X ", msg_buffer[i]);
    }
    Serial.println();

    for( i = 0; i < 3; i++ )
    {
        cc1100_tx(msg_buffer, 28);
        delay(10);
    }
}

static void elero_spi_init(void)
{
    pinMode(HSPI_CS, OUTPUT);
    digitalWrite(HSPI_CS, HIGH);

    pinMode(CC1101_GDO0, INPUT);
    pinMode(CC1101_GDO2, INPUT);

    hspi = new SPIClass(HSPI);
    hspi->begin();

    delay(10);
}

static void elero_cc1100_init()
{
    Serial.printf("[cc1100] 0xf0 = %x\r\n", spi_read_reg(0xF0));
    Serial.printf("[cc1100] 0xf1 = %x\r\n", spi_read_reg(0xF1));


    spi_write_cmd(0x30); delayMicroseconds(50);
    spi_write_cmd(0x36); delayMicroseconds(50);

    spi_write_reg(0x0B, 0x08); delayMicroseconds(15);
    spi_write_reg(0x0C, 0x00); delayMicroseconds(15);
    spi_write_reg(0x0D, 0x21); delayMicroseconds(15);
    spi_write_reg(0x0E, 0x71); delayMicroseconds(15);
    spi_write_reg(0x0F, 0x7A); delayMicroseconds(15);
    spi_write_reg(0x10, 0x7B); delayMicroseconds(15);
    spi_write_reg(0x11, 0x83); delayMicroseconds(15);
    spi_write_reg(0x12, 0x13); delayMicroseconds(15);
    spi_write_reg(0x13, 0x52); delayMicroseconds(15);
    spi_write_reg(0x14, 0xF8); delayMicroseconds(15);
    spi_write_reg(0x0A, 0x00); delayMicroseconds(15);
    spi_write_reg(0x15, 0x43); delayMicroseconds(15);
    spi_write_reg(0x21, 0xB6); delayMicroseconds(15);
    spi_write_reg(0x22, 0x10); delayMicroseconds(15);
    spi_write_reg(0x18, 0x18); delayMicroseconds(15);
    spi_write_reg(0x17, 0x3F); delayMicroseconds(15);
    spi_write_reg(0x19, 0x1D); delayMicroseconds(15);
    spi_write_reg(0x1A, 0x1C); delayMicroseconds(15);
    spi_write_reg(0x1B, 0xC7); delayMicroseconds(15);
    spi_write_reg(0x1C, 0x00); delayMicroseconds(15);
    spi_write_reg(0x1D, 0xB2); delayMicroseconds(15);
    spi_write_reg(0x23, 0xEA); delayMicroseconds(15);
    spi_write_reg(0x24, 0x2A); delayMicroseconds(15);
    spi_write_reg(0x25, 0x00); delayMicroseconds(15);
    spi_write_reg(0x26, 0x1F); delayMicroseconds(15);
    spi_write_reg(0x29, 0x59); delayMicroseconds(15);
    spi_write_reg(0x2C, 0x81); delayMicroseconds(15);
    spi_write_reg(0x2D, 0x35); delayMicroseconds(15);
    spi_write_reg(0x2E, 0x09); delayMicroseconds(15);
    spi_write_reg(0x00, 0x06); delayMicroseconds(15);
    spi_write_reg(0x02, 0x09); delayMicroseconds(15);
    spi_write_reg(0x07, 0x8C); delayMicroseconds(15);
    spi_write_reg(0x08, 0x45); delayMicroseconds(15);
    spi_write_reg(0x09, 0x00); delayMicroseconds(15);
    spi_write_reg(0x06, 0x3C); delayMicroseconds(15);
    spi_write_reg(0x04, 0xD3); delayMicroseconds(15);
    spi_write_reg(0x05, 0x91); delayMicroseconds(15);
    spi_write_reg(0x7E, 0xC2);

    delayMicroseconds(40);

    spi_write_cmd(0x34);

    Serial.print("Waiting for clear channel...");
    while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );
    Serial.println("channel cleared");
}

void setup()
{

    Serial.begin(921600);

    delay(100);


    pinMode(TEST_INPUT, INPUT_PULLUP);

    elero_spi_init();
    elero_cc1100_init();
    Serial.println("[elero] inited");
}

uint8_t sync_det_prev = 0;
uint8_t rx_fifo[256];
uint32_t tx;
uint8_t state;

//#define ELERO_MSG_JUST_MY_REMOTES

void loop()
{
    uint8_t sync_det = digitalRead(SYNCWORD_DET_SENT_TX_SENT);
    uint8_t bytes_in_fifo;
    uint8_t pck_len;
    uint8_t i;

    //if( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW )
    {
        if( (sync_det_prev != LOW) && (sync_det == LOW) )
        {
            delayMicroseconds(50);

            bytes_in_fifo = spi_read_reg(0xFB);

            if(bytes_in_fifo)
            {
                uint8_t calc_par;

                pck_len = spi_read_reg(0xFF);
                bytes_in_fifo--,

                spi_read_burst(0xFF, rx_fifo, bytes_in_fifo);
                
                spi_write_cmd(0x36); // idle for flushing
                spi_write_cmd(0x3A); // flush RX fifo
                spi_write_cmd(0x3B); // flush TX fifo
                spi_write_cmd(0x34); // back to RX for listening clear channel

                if( scan_check_if_addr_remote(rx_fifo) )
                {
                    scan_remote_add(rx_fifo);
                }

#ifdef ELERO_MSG_JUST_MY_REMOTES
                uint8_t myremote = 0;

                for( i = 0; i < 2; i++ )
                {

                    if( (memcmp(&rx_fifo[6], &my_remotes[i][0], 3) == 0) && (memcmp(&rx_fifo[9], &my_remotes[i][0], 3) == 0) && (memcmp(&rx_fifo[12], &my_remotes[i][0], 3) == 0) )
                    {
                        myremote = 1;
                    }
                }

                if( myremote )
                {
#endif
                Serial.printf("[%7d] len=%2d ", millis(), pck_len);

                Serial.printf("cnt=%3d ", rx_fifo[0]);
                Serial.printf("0x%02X ", rx_fifo[1]);              // pck_info
                Serial.printf("0x%02X ", rx_fifo[2]);              // pck_info2
                Serial.printf("0x%02X ", rx_fifo[3]);              // hop
                Serial.printf("0x%02X ", rx_fifo[4]);              // addr_sys
                Serial.printf("0x%02X ", rx_fifo[5]);              // src_grp
                Serial.printf("src=[%02X%02X%02X] ", rx_fifo[6],  rx_fifo[7],  rx_fifo[8]);  // source addr
                Serial.printf("bwd=[%02X%02X%02X] ", rx_fifo[9],  rx_fifo[10], rx_fifo[11]); // backward addr
                Serial.printf("fwd=[%02X%02X%02X] ", rx_fifo[12], rx_fifo[13], rx_fifo[14]); // fwd addr
                Serial.printf("[0x%02X ", rx_fifo[15]);             // destination count
                Serial.printf("0x%02X] ", rx_fifo[16]);          // destination

                Serial.printf("payl={");
                for( i = 0; i < (pck_len-17); i++ )
                {
                    Serial.printf("0x%02X ", rx_fifo[17+i]);
                }
                Serial.printf("} ");

                //Serial.printf("| CRC=%d LQI=%x RSSI=%d | ", (rx_fifo[bytes_in_fifo-2] & (0x80)) == 0x80, rx_fifo[bytes_in_fifo-2] & (~0x80), rx_fifo[bytes_in_fifo-1]);

                if( pck_len == 0x1B )   // len=27
                {
                    msg_decode(&rx_fifo[19]);

                    Serial.printf("| payl_dec=");

                    // non-encrypted part of payload
                    Serial.printf("{0x%02X 0x%02X} ", rx_fifo[17], rx_fifo[18]);

                    // always 0 (contains the key that gets eliminated during decrypt) - printing for checking the encrypt process
                    Serial.printf("{0x%02X 0x%02X} ", rx_fifo[19], rx_fifo[20]);

                    // useful payload
                    for( i = 0; i < 5; i++ )
                    {
                        Serial.printf("0x%02X ", rx_fifo[21+i]);
                    }


                    calc_par = calc_exp_parity(rx_fifo[0], &rx_fifo[19]);

                    // + 1 last byte as parity
                    Serial.printf(" {0x%02X} %c {0x%02X}", rx_fifo[26], (rx_fifo[26] == calc_par)?('='):('?'), calc_par);
                }

                if( pck_len == 0x1C )   // len=28
                {
                    msg_decode(&rx_fifo[20]);

                    Serial.printf("| payl_dec=");

                    // non-encrypted part of payload
                    Serial.printf("{0x%02X 0x%02X 0x%02X} ", rx_fifo[17], rx_fifo[18], rx_fifo[19]);

                    // always 0 (contains the key that gets eliminated during decrypt) - printing for checking the encrypt process
                    Serial.printf("{0x%02X 0x%02X} ", rx_fifo[20], rx_fifo[21]);

                    // useful payload
                    for( i = 0; i < 5; i++ )
                    {
                        Serial.printf("0x%02X ", rx_fifo[22+i]);
                    }

                    // + 1 last byte as parity
                    Serial.printf(" {0x%02X} ", rx_fifo[27]);
                }

                if( pck_len == 0x1D )   // len=29
                {
                    msg_decode(&rx_fifo[21]);

                    Serial.printf("| payl_dec=");

                    // non-encrypted part of payload
                    Serial.printf("{0x%02X 0x%02X 0x%02X 0x%02X} ", rx_fifo[17], rx_fifo[18], rx_fifo[19], rx_fifo[20]);

                    // always 0 (contains the key that gets eliminated during decrypt) - printing for checking the encrypt process
                    Serial.printf("{0x%02X 0x%02X} ", rx_fifo[21], rx_fifo[22]);

                    // useful payload
                    for( i = 0; i < 5; i++ )
                    {
                        Serial.printf("0x%02X ", rx_fifo[23+i]);
                    }

                    // + 1 last byte as parity
                    Serial.printf(" {0x%02X} ", rx_fifo[28]);
                }

                if( pck_len == 0x1E )   // len=29
                {
                    msg_decode(&rx_fifo[22]);

                    Serial.printf("| payl_dec=");

                    // non-encrypted part of payload
                    Serial.printf("{0x%02X 0x%02X 0x%02X 0x%02X 0x%02X} ", rx_fifo[17], rx_fifo[18], rx_fifo[19], rx_fifo[20], rx_fifo[21]);

                    // always 0 (contains the key that gets eliminated during decrypt) - printing for checking the encrypt process
                    Serial.printf("{0x%02X 0x%02X} ", rx_fifo[22], rx_fifo[23]);

                    // useful payload
                    for( i = 0; i < 5; i++ )
                    {
                        Serial.printf("0x%02X ", rx_fifo[24+i]);
                    }

                    // + 1 last byte as parity
                    Serial.printf(" {0x%02X} ", rx_fifo[29]);
                }
                Serial.println();
#ifdef ELERO_MSG_JUST_MY_REMOTES
                }
#endif
            }
        }
    }

    sync_det_prev = sync_det;

    if( digitalRead(TEST_INPUT) == LOW )
    {
        if( (millis()-tx) > 2000 )
        {
            uint8_t remote = 0;
            uint8_t channel = 1;
            tx = millis();

            elero_send_msg_down(remote, channel);

            delay(2000);

            elero_send_msg_stop(remote, channel);
        }
    }
/*
    static uint32_t remote_disp_ts = 0;

    if( (millis()-remote_disp_ts) >= 2000 )
    {
        remote_disp_ts = millis();

        Serial.write(27);       // ESC command
        Serial.print("[2J");    // clear screen command
        Serial.write(27);
        Serial.print("[H");     // cursor to home command

        for( i = 0; i < 32; i++ )
        {
            Serial.printf("{0x%02X 0x%02X 0x%02X} \r\n", remotes[i][0], remotes[i][1], remotes[i][2]);
        }
    }
*/
}


