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

static const int spiClk = 500000;
byte address = 0x00;
SPIClass * hspi = NULL;

static const uint8_t flash_table_encode[] = {0x08, 0x02, 0x0d, 0x01, 0x0f, 0x0e, 0x07, 0x05, 0x09, 0x0c, 0x00, 0x0a, 0x03, 0x04, 0x0b, 0x06};
static const uint8_t flash_table_decode[] = {0x0a, 0x03, 0x01, 0x0c, 0x0d, 0x07, 0x0f, 0x06, 0x00, 0x08, 0x0b, 0x0e, 0x09, 0x02, 0x05, 0x04};

static uint8_t gIndex = 0x60;
static uint16_t gCode = (0x00 - (gIndex * 0x708F)) & 0xFFFF;

static uint8_t msg_buffer[64];

static const uint8_t my_remotes[2][3] = {
        {0x1A, 0x01, 0x0D},
        {0x62, 0x0B, 0x0D},
};

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

    Serial.print("cc1100_tx...");

    Serial.print("waiting for clear channel...");
    while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );
    Serial.println("cleared");

    Serial.println("spi transmit start");
    spi_write_burst(0x7F, msg, len);
    Serial.println("spi transmit end");

    delayMicroseconds(10);

    Serial.println("cc1100_tx.tx");
    spi_write_cmd(0x35);

    Serial.println("cc1100_tx.waiting tx state");

    do
    {
        marcstate = spi_read_reg(0xF5);
        delay(1);
    }
    while( marcstate != 0x13 );

    Serial.printf("cc1100_tx.waiting non tx state %x\r\n", marcstate);

    do
    {
        marcstate = spi_read_reg(0xF5);
        delay(1);
    }
    while( marcstate == 0x13 );

    Serial.printf("done %x\r\n", marcstate);
}


static void generate_msg_down(uint8_t* msg, uint8_t index, uint8_t button_pressed)
{
    uint32_t i;
    uint8_t* msg_data;

    memset(msg, 0, 28);

    msg[ 0] = 0x1B;      // msg_len
    msg[ 1] = index;      // pck cnt
    msg[ 2] = 0x44;      // pck_info = STOP
    msg[ 3] = 0x10;      // pck_inf2 = STOP
    msg[ 4] = 0x00;      // hop_info = 0
    msg[ 5] = 0x01;      // sys_addr = 1
    msg[ 6] = 0x11;      // source_group = 0x11

    msg[ 7] = 0x1A;        // source addr[0]
    msg[ 8] = 0x01;        // source addr[1]
    msg[ 9] = 0x0D;        // source addr[2]

    msg[10] = 0x1A;        // backward addr[0]
    msg[11] = 0x01;        // backward addr[1]
    msg[12] = 0x0D;        // backward addr[2]

    msg[13] = 0x1A;        // forward addr[0]
    msg[14] = 0x01;        // forward addr[1]
    msg[15] = 0x0D;        // forward addr[2]

    msg[16] = 0x01;        // dest_count = 1
    msg[17] = 0x11;        // dest = TODO        (ch1=0x11(?) ch2=0x02 ch3=0x03 ch4=0x03)
    msg[18] = 0x00;
    msg[19] = 0x03;

    gCode = (0x00 - (index * 0x708F)) & 0xFFFF;

    msg[20] = (gCode >> 8) & 0xFF;
    msg[21] = gCode & 0xFF;
    msg[22] = (button_pressed) ? (0x40) : (0x00);

    msg_encode(&msg[20], (gCode >> 8) & 0xFF, gCode & 0xFF);
}

static void generate_msg_stop(uint8_t* msg, uint8_t index, uint8_t button_pressed)
{
    uint32_t i;

    memset(msg, 0, 30);

    msg[ 0] = 0x1D;        // msg_len
    msg[ 1] = index;     // pck cnt
    msg[ 2] = 0x6A;        // pck_info = STOP
    msg[ 3] = 0x10;        // pck_inf2 = STOP
    msg[ 4] = 0x00;        // hop_info = 0
    msg[ 5] = 0x01;        // sys_addr = 1
    msg[ 6] = 0x11;        // source_group = 0x11

    msg[ 7] = 0x1A;        // source addr[0]
    msg[ 8] = 0x01;        // source addr[1]
    msg[ 9] = 0x0D;        // source addr[2]

    msg[10] = 0x1A;        // backward addr[0]
    msg[11] = 0x01;        // backward addr[1]
    msg[12] = 0x0D;        // backward addr[2]

    msg[13] = 0x1A;        // forward addr[0]
    msg[14] = 0x01;        // forward addr[1]
    msg[15] = 0x0D;        // forward addr[2]

    msg[16] = 0x01;        // dest_count = 1
    msg[17] = 0xC2;        // dest = TODO
    msg[18] = 0xA3;
    msg[19] = 0x35;
    msg[20] = 0x00;
    msg[21] = 0x03;

    gCode = (0x00 - (index * 0x708F)) & 0xFFFF;

    msg[22] = (gCode >> 8) & 0xFF;
    msg[23] = gCode & 0xFF;
    msg[24] = 0x10;

    msg_encode(&msg[22], (gCode >> 8) & 0xFF, gCode & 0xFF);
}

static void send_msg_down(void)
{
    uint8_t i;

    generate_msg_down(msg_buffer, gIndex, 1);

    gIndex++;

    Serial.println("BUTTON DOWN PRESS msg generated:");
    for( i = 0; i < 28; i++ )
    {
        Serial.printf("0x%02X ", msg_buffer[i]);
    }

    //for( i = 0; i < 3; i++ )
    {
        cc1100_tx(msg_buffer, 28);

        delay(10);
    }

    delay(100);

    generate_msg_down(msg_buffer, gIndex, 0);

    gIndex++;

    Serial.println("BUTTON DOWN -- RELEASE msg generated:");
    //for( i = 0; i < 28; i++ )
    {
        Serial.printf("0x%02X ", msg_buffer[i]);
    }

    for( i = 0; i < 3; i++ )
    {
        cc1100_tx(msg_buffer, 28);

        delay(10);
    }

    delay(100);
}

void send_msg_stop(void)
{
    uint8_t i;

    generate_msg_stop(msg_buffer, gIndex, 1);

    gIndex++;

    Serial.println("BUTTON STOP PRESS msg generated:");
    for( i = 0; i < 30; i++ )
    {
        Serial.printf("0x%02X ", msg_buffer[i]);
    }

    cc1100_tx(msg_buffer, 30);
    delay(10);

}

void send_msg_up(void)
{
    uint8_t i;
    uint8_t* msg_data;

    Serial.print("waiting for clear channel...");
    while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );
    Serial.println("cleared");

    Serial.println("UP: button pressed");

    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);

    hspi->transfer(0x7F);
    hspi->transfer(0x1B);

    hspi->transfer(gIndex); // pckt

    hspi->transfer(0x44);
    hspi->transfer(0x12); // reset packet
    hspi->transfer(0x00);
    hspi->transfer(0x01);
    hspi->transfer(0x11);

    hspi->transfer(0x1A);
    hspi->transfer(0x01);
    hspi->transfer(0x0D);
    hspi->transfer(0x1A);
    hspi->transfer(0x01);
    hspi->transfer(0x0D);
    hspi->transfer(0x1A);
    hspi->transfer(0x01);
    hspi->transfer(0x0D);

    hspi->transfer(0x01);
    hspi->transfer(0x11);
    hspi->transfer(0x00);
    hspi->transfer(0x03);

    for( i = 0; i < 8; i++ )
    {
        hspi->transfer(msg_data[i]);
    }

    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();

    delayMicroseconds(10);

    spi_write_cmd(0x35);

    while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);
    while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);


    delay(100);

    Serial.println("UP: button release");

    while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );

    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);

    hspi->transfer(0x7F);
    hspi->transfer(0x1B);

    hspi->transfer(0x02);

    hspi->transfer(0x44);
    hspi->transfer(0x10);
    hspi->transfer(0x00);
    hspi->transfer(0x01);
    hspi->transfer(0x11);

    hspi->transfer(0x1A);
    hspi->transfer(0x01);
    hspi->transfer(0x0D);
    hspi->transfer(0x1A);
    hspi->transfer(0x01);
    hspi->transfer(0x0D);
    hspi->transfer(0x1A);
    hspi->transfer(0x01);
    hspi->transfer(0x0D);

    hspi->transfer(0x01);
    hspi->transfer(0x11);
    hspi->transfer(0x00);
    hspi->transfer(0x03);

    hspi->transfer(0xAA);
    hspi->transfer(0x82);
    hspi->transfer(0x55);
    hspi->transfer(0x04);
    hspi->transfer(0xAA);
    hspi->transfer(0x72);
    hspi->transfer(0x66);
    hspi->transfer(0x0E);
/*
    hspi->transfer(0xEE);
    hspi->transfer(0x7A);
    hspi->transfer(0xC4);
    hspi->transfer(0x85);
    hspi->transfer(0x22);
    hspi->transfer(0x3A);
    hspi->transfer(0xEE);
    hspi->transfer(0xF6);
*/
    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();

    delayMicroseconds(10);

    spi_write_cmd(0x35);

    while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);
    while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);
}

void setup()
{
    Serial.begin(460800);

    delay(100);

    pinMode(HSPI_CS, OUTPUT);
    digitalWrite(HSPI_CS, HIGH);

    pinMode(CC1101_GDO0, INPUT);
    pinMode(CC1101_GDO2, INPUT);
    pinMode(TEST_INPUT, INPUT_PULLUP);

    hspi = new SPIClass(HSPI);
    hspi->begin();

    delay(10);

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

uint8_t sync_det_prev = 0;
uint8_t rx_fifo[256];
uint32_t tx;
uint8_t state;

//#define ELERO_MSG_JUST_MY_REMOTES
#define ELERO_MSG_ALL

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
                pck_len = spi_read_reg(0xFF);
                bytes_in_fifo--,

                spi_read_burst(0xFF, rx_fifo, bytes_in_fifo);

#ifdef ELERO_MSG_ALL
                Serial.printf("[%7d] plen=0x%02X ", millis(), pck_len);

                Serial.printf("cnt=%3d ", rx_fifo[0]);
                Serial.printf("pck_info=0x%02X ", rx_fifo[1]);
                Serial.printf("pck_info2=0x%02X ", rx_fifo[2]);
                Serial.printf("hop=0x%02X ", rx_fifo[3]);
                Serial.printf("addr_sys=0x%02X ", rx_fifo[4]);
                Serial.printf("src_grp=0x%02X ", rx_fifo[5]);
                Serial.printf("addr_scr=0x%02X:0x%02X:0x%02X ", rx_fifo[6],  rx_fifo[7],  rx_fifo[8]);
                Serial.printf("addr_bwd=0x%02X:0x%02X:0x%02X ", rx_fifo[9],  rx_fifo[10], rx_fifo[11]);
                Serial.printf("addr_fwd=0x%02X:0x%02X:0x%02X ", rx_fifo[12], rx_fifo[13], rx_fifo[14]);
                Serial.printf("dst_cnt=0x%02X ", rx_fifo[15]);
                Serial.printf("dst=0x%02X ", rx_fifo[16]);

                Serial.printf("payl=");
                for( i = 0; i < (pck_len-17); i++ )
                {
                    Serial.printf("0x%02X ", rx_fifo[17+i]);
                }

                //Serial.printf("| CRC=%d LQI=%x RSSI=%d | ", (rx_fifo[bytes_in_fifo-2] & (0x80)) == 0x80, rx_fifo[bytes_in_fifo-2] & (~0x80), rx_fifo[bytes_in_fifo-1]);

                if( pck_len == 0x1B )
                {
                    msg_decode(&rx_fifo[19]);

                    Serial.printf(" | payl_dec = ");

                    // un-encrypted part of payload
                    Serial.printf("{0x%02X 0x%02X} ", rx_fifo[17], rx_fifo[18]);

                    // always 0 (contains the key that gets eliminated during decrypt) - printing for checking the encrypt process
                    Serial.printf("{0x%02X 0x%02X} ", rx_fifo[19], rx_fifo[20]);

                    // useful payload
                    for( i = 0; i < 5; i++ )
                    {
                        Serial.printf("0x%02X ", rx_fifo[21+i]);
                    }

                    // + 1 last byte as parity
                    Serial.printf(" {0x%02X} ", rx_fifo[26]);
                }

                Serial.println("");
#endif

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
                    Serial.printf("[%7d] pck_len=0x%02X ------------------------------------------------------------------------------- \r\n ", millis(), pck_len);

                    Serial.printf("index/counter %d\r\n", rx_fifo[0]);
                    Serial.printf("packet info        0x%02X\r\n", rx_fifo[1]);
                    Serial.printf("packet info 2      0x%02X\r\n", rx_fifo[2]);
                    Serial.printf("hop information    0x%02X\r\n", rx_fifo[3]);
                    Serial.printf("sys address        0x%02X\r\n", rx_fifo[4]);
                    Serial.printf("source group       0x%02X\r\n", rx_fifo[5]);
                    Serial.printf("source address     0x%02X:0x%02X:0x%02X\r\n", rx_fifo[6],  rx_fifo[7],  rx_fifo[8]);
                    Serial.printf("backward address   0x%02X:0x%02X:0x%02X\r\n", rx_fifo[9],  rx_fifo[10], rx_fifo[11]);
                    Serial.printf("forward address    0x%02X:0x%02X:0x%02X\r\n", rx_fifo[12], rx_fifo[13], rx_fifo[14]);
                    Serial.printf("destination count  0x%02X\r\n", rx_fifo[15]);
                    Serial.printf("destination        0x%02X\r\n", rx_fifo[16]);


                    Serial.printf("payload_orig    = ");

                    for( i = 0; i < 10; i++ )
                    {
                        Serial.printf("0x%02X ", rx_fifo[17+i]);
                    }
                    Serial.println();


                    msg_decode(&rx_fifo[19]);

                    Serial.printf("payload_decoded = ");
                    Serial.printf("0x%02X ", rx_fifo[17]);
                    Serial.printf("0x%02X ", rx_fifo[18]);

                    for( i = 0; i < 8; i++ )
                    {
                        Serial.printf("0x%02X ", rx_fifo[19+i]);
                    }
                    Serial.println();

                    Serial.printf("payload_useful  = ");
                    Serial.printf("0x%02X ", rx_fifo[17]);
                    Serial.printf("0x%02X ", rx_fifo[18]);
                    //Serial.printf("0x%02X ", rx_fifo[19]);    // key0
                    //Serial.printf("0x%02X ", rx_fifo[20]);    // key1
                    Serial.printf("0x%02X ", rx_fifo[21]);
                    Serial.printf("0x%02X ", rx_fifo[22]);
                    Serial.printf("0x%02X ", rx_fifo[23]);
                    Serial.printf("0x%02X ", rx_fifo[24]);
                    Serial.printf("0x%02X ", rx_fifo[25]);
                    //Serial.printf("0x%02X ", rx_fifo[26]);    // parity
                    Serial.println();

                    Serial.printf("CRC=%d LQI=%x RSSI=%d ", (rx_fifo[bytes_in_fifo-2] & (0x80)) == 0x80, rx_fifo[bytes_in_fifo-2] & (~0x80), rx_fifo[bytes_in_fifo-1]);
                    Serial.println("");
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
            tx = millis();

            send_msg_down();

            delay(2000);

            send_msg_stop();
            /*

            switch(state)
            {
            case 0: send_msg_down(); state = 1; break;
            case 1: send_msg_stop(); state = 2; break;
            case 2: send_msg_up();   state = 0; break;
            default: break;
            }
            */
        }
    }
}


