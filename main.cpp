#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <SPI.h>

#define CC1101_GDO0		(26)
#define CC1101_GDO2		(25)

#define SIGNAL_CLEAR_CHANNEL_ASSESMENT	CC1101_GDO0
#define SYNCWORD_DET_SENT_TX_SENT		CC1101_GDO2

static const int spiClk = 4000000;
byte address = 0x00;
SPIClass * hspi = NULL;

void spi_write_cmd(uint8_t cmd)
{
	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);
	hspi->transfer(cmd);
	digitalWrite(15, HIGH);
	hspi->endTransaction();
}

void spi_write_reg(uint8_t addr, uint8_t value)
{
	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);
	hspi->transfer(addr);
	hspi->transfer(value);
	digitalWrite(15, HIGH);
	hspi->endTransaction();
}

uint8_t spi_read_reg(uint8_t addr)
{
	uint8_t reg;
	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);
	hspi->transfer(addr);
	reg = hspi->transfer(0x00);
	digitalWrite(15, HIGH);
	hspi->endTransaction();

	return reg;
}

void spi_read_burst(uint8_t addr, uint8_t* data, uint8_t len)
{
	uint8_t i;

	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);

	hspi->transfer(addr);

	for( i = 0; i < len; i++ )
	{
		data[i] = hspi->transfer(0x00);
	}

	digitalWrite(15, HIGH);
	hspi->endTransaction();
}

void send_msg_down(void)
{
	Serial.print("waiting for clear channel...");
	while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );
	Serial.println("cleared");

	Serial.println("button pressed");
	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);

	hspi->transfer(0x7F);
	hspi->transfer(0x1B);

	hspi->transfer(036); // pck cnt

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

	hspi->transfer(0x74;
	hspi->transfer(0x77);
	hspi->transfer(0x9E);
	hspi->transfer(0x8D);
	hspi->transfer(0x8C);
	hspi->transfer(0x3B);
	hspi->transfer(0xF4);
	hspi->transfer(0xF0);
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
	digitalWrite(15, HIGH);
	hspi->endTransaction();

	delayMicroseconds(10);

	spi_write_cmd(0x35);


	while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);
	while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);


	delay(100);
	while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );


	Serial.println("button released");

	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);

	hspi->transfer(0x7F);
	hspi->transfer(0x1B);

	hspi->transfer(0x37); // pck cnt

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

	hspi->transfer(0x64);
	hspi->transfer(0x45);
	hspi->transfer(0xAE);
	hspi->transfer(0xC1);
	hspi->transfer(0x5C);
	hspi->transfer(0xE6);
	hspi->transfer(0x14);
	hspi->transfer(0xCA);
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
	digitalWrite(15, HIGH);
	hspi->endTransaction();

	delayMicroseconds(10);

	spi_write_cmd(0x35);


	while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);
	while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);

}

void send_msg_stop(void)
{
	Serial.print("waiting for clear channel...");
	while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );
	Serial.println("cleared");

	Serial.println("command STOP!");
	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);

	hspi->transfer(0x7F);
	hspi->transfer(0x1B);
	hspi->transfer(0x1D);

	hspi->transfer(0x38); // pck cnt

	hspi->transfer(0x6A);
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
	hspi->transfer(0xC2);
	hspi->transfer(0xA3);
	hspi->transfer(0x35);

	hspi->transfer(0x00);
	hspi->transfer(0x03);
	hspi->transfer(0x93);
	hspi->transfer(0xF9);
	hspi->transfer(0xEF);
	hspi->transfer(0xBF);
	hspi->transfer(0xB9);
	hspi->transfer(0xD9);
	hspi->transfer(0x09);
	hspi->transfer(0xD3);

	digitalWrite(15, HIGH);
	hspi->endTransaction();

	delayMicroseconds(10);

	spi_write_cmd(0x35);

	while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);
	while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);
}

void send_msg_up(void)
{
	Serial.print("waiting for clear channel...");
	while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );
	Serial.println("cleared");

	Serial.println("UP: button pressed");

	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);

	hspi->transfer(0x7F);
	hspi->transfer(0x1B);

	hspi->transfer(0x01); // pckt

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

	hspi->transfer(0x54);
	hspi->transfer(0xF4);
	hspi->transfer(0xEE);
	hspi->transfer(0xBC);
	hspi->transfer(0x6C);
	hspi->transfer(0xDE);
	hspi->transfer(0xA4);
	hspi->transfer(0x02);

	digitalWrite(15, HIGH);
	hspi->endTransaction();

	delayMicroseconds(10);

	spi_write_cmd(0x35);

	while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);
	while( spi_read_reg(0xF5) != 0x0D ) delayMicroseconds(10);


	delay(100);

	Serial.println("UP: button release");

	while( digitalRead(SIGNAL_CLEAR_CHANNEL_ASSESMENT) == LOW );

	hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
	digitalWrite(15, LOW);

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
	digitalWrite(15, HIGH);
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
	pinMode(15, OUTPUT);
	digitalWrite(15, HIGH);

	pinMode(CC1101_GDO0, INPUT);
	pinMode(CC1101_GDO2, INPUT);
	pinMode(17, INPUT_PULLUP);

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

				Serial.printf("[%7d] %d pck_len=0x%02X ", millis(), bytes_in_fifo, pck_len);

				for(i = 0; i < (bytes_in_fifo-2); i++ )
				{
					Serial.printf("0x%02X ", rx_fifo[i]);
				}

				Serial.printf("CRC=%d LQI=%x RSSI=%d ", (rx_fifo[bytes_in_fifo-2] & (0x80)) == 0x80, rx_fifo[bytes_in_fifo-2] & (~0x80), rx_fifo[bytes_in_fifo-1]);

				Serial.println("");
			}
		}
	}

	sync_det_prev = sync_det;

	if( digitalRead(17) == LOW )
	{
		if( (millis()-tx) > 2000 )
		{
			tx = millis();

			switch(state)
			{
			case 0: send_msg_down(); state = 1; break;
			case 1: send_msg_stop(); state = 2; break;
			case 2: send_msg_up();   state = 0; break;
			default: break;
			}

		}
	}
}


