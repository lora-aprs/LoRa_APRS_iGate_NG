
#include <cstring>
#include "sx1287.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40


#define MAX_PKT_LENGTH           255


SX1287::SX1287(
	std::mutex& guard,
	gpio_num_t chip_select_pin,
	uint8_t spi_command_bits,
	uint8_t spi_address_bits,
	uint8_t bits_between_address_and_data_phase,
	uint8_t spi_mode,
	uint8_t spi_positive_duty_cycle,
	uint8_t spi_cs_ena_posttrans,
	int spi_clock_speed_hz,
	uint32_t spi_device_flags,
	int spi_queue_size,
	bool use_pre_transaction_callback,
	bool use_post_transaction_callback) :
		SPIDevice(
			guard,
			spi_command_bits,
			spi_address_bits,
			bits_between_address_and_data_phase,
			spi_mode,
			spi_positive_duty_cycle,
			spi_cs_ena_posttrans,
			spi_clock_speed_hz,
			spi_device_flags,
			spi_queue_size,
			use_pre_transaction_callback,
			use_post_transaction_callback),
		cs_pin(chip_select_pin)
{
}

bool SX1287::init(spi_host_device_t host)
{
	return initialize(host, cs_pin);
}

uint8_t SX1287::read_id()
{
	return readRegister(REG_VERSION);
}

bool SX1287::begin(long frequency)
{
	uint8_t id = read_id();
	if(id != 0x12)
	{
		return false;
	}

	sleep();

	setFrequency(frequency);

	writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

	writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

	writeRegister(REG_MODEM_CONFIG_3, 0x04);

	setTxPower(20);

	idle();

	return true;
}

void SX1287::end()
{
	sleep();
}

bool SX1287::beginPacket()
{
	if(isTransmitting())
	{
		return false;
	}

	idle();

	explicitHeaderMode();

	writeRegister(REG_FIFO_ADDR_PTR, 0);
	writeRegister(REG_PAYLOAD_LENGTH, 0);

	return true;
}

void SX1287::endPacket()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
	{
		// sleep? context switch?
	}
	writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}


size_t SX1287::write(uint8_t byte)
{
	return write(&byte, sizeof(byte));
}

size_t SX1287::write(const uint8_t *buffer, size_t size)
{
	int currentLength = readRegister(REG_PAYLOAD_LENGTH);

	// check size
	if ((currentLength + size) > MAX_PKT_LENGTH)
	{
		size = MAX_PKT_LENGTH - currentLength;
	}

	// write data
	for (size_t i = 0; i < size; i++)
	{
		writeRegister(REG_FIFO, buffer[i]);
	}

	// update length
	writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

	return size;
}

int SX1287::available()
{
	return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int SX1287::read()
{
	if (!available())
	{
		return -1;
	}

	_packetIndex++;

	return readRegister(REG_FIFO);
}

int SX1287::peek()
{
	if (!available())
	{
		return -1;
	}

	// store current FIFO address
	int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

	// read
	uint8_t b = readRegister(REG_FIFO);

	// restore FIFO address
	writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

	return b;
}

int SX1287::parsePacket(int size = 0)
{
	int packetLength = 0;
	int irqFlags = readRegister(REG_IRQ_FLAGS);

	if (size > 0)
	{
		implicitHeaderMode();

		writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	}
	else
	{
		explicitHeaderMode();
	}

	// clear IRQ's
	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
	{
		// received a packet
		_packetIndex = 0;

		// read packet length
		if (_implicitHeaderMode)
		{
			packetLength = readRegister(REG_PAYLOAD_LENGTH);
		}
		else
		{
			packetLength = readRegister(REG_RX_NB_BYTES);
		}

		// set FIFO address to current RX address
		writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

		// put in standby mode
		idle();
	}
	else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
	{
		// not currently in RX mode

		// reset FIFO address
		writeRegister(REG_FIFO_ADDR_PTR, 0);

		// put in single RX mode
		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}

	return packetLength;
}

int SX1287::packetRssi()
{
	return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float SX1287::packetSnr()
{
	return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long SX1287::packetFrequencyError()
{
	int32_t freqError = 0;
	freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 0b111);
	freqError <<= 8L;
	freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
	freqError <<= 8L;
	freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

	if (readRegister(REG_FREQ_ERROR_MSB) & 0b1000) // Sign bit is on
	{
		freqError -= 0x80000;
	}

	const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
	const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

	return static_cast<long>(fError);
}

void SX1287::idle()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void SX1287::sleep()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void SX1287::setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN)
{
	if (PA_OUTPUT_RFO_PIN == outputPin)
	{
		// RFO
		if (level < 0)
		{
			level = 0;
		} else if (level > 14)
		{
			level = 14;
		}

		writeRegister(REG_PA_CONFIG, 0x70 | level);
	}
	else
	{
		// PA BOOST
		if (level > 17)
		{
			if (level > 20)
			{
				level = 20;
			}

			// subtract 3 from level, so 18 - 20 maps to 15 - 17
			level -= 3;

			// High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
			writeRegister(REG_PA_DAC, 0x87);
			setOCP(140);
		}
		else
		{
			if (level < 2)
			{
				level = 2;
			}
			//Default value PA_HF/LF or +17dBm
			writeRegister(REG_PA_DAC, 0x84);
			setOCP(100);
		}

		writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
	}
}

void SX1287::setFrequency(long frequency)
{
	_frequency = frequency;

	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void SX1287::setSpreadingFactor(int sf)
{
	if (sf < 6)
	{
		sf = 6;
	}
	else if (sf > 12)
	{
		sf = 12;
	}

	if (sf == 6)
	{
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
	}
	else
	{
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
	setLdoFlag();
}

void SX1287::setSignalBandwidth(long sbw)
{
	int bw;

	if (sbw <= 7.8E3)
	{
		bw = 0;
	}
	else if (sbw <= 10.4E3)
	{
		bw = 1;
	}
	else if (sbw <= 15.6E3)
	{
		bw = 2;
	}
	else if (sbw <= 20.8E3)
	{
		bw = 3;
	}
	else if (sbw <= 31.25E3)
	{
		bw = 4;
	}
	else if (sbw <= 41.7E3)
	{
		bw = 5;
	}
	else if (sbw <= 62.5E3)
	{
		bw = 6;
	}
	else if (sbw <= 125E3)
	{
		bw = 7;
	}
	else if (sbw <= 250E3)
	{
		bw = 8;
	}
	else /*if (sbw <= 250E3)*/
	{
		bw = 9;
	}

	writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
	setLdoFlag();
}

void SX1287::setCodingRate4(int denominator)
{
	if (denominator < 5)
	{
		denominator = 5;
	}
	else if (denominator > 8)
	{
		denominator = 8;
	}

	int cr = denominator - 4;

	writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void SX1287::setPreambleLength(long length)
{
	writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
	writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void SX1287::setSyncWord(int sw)
{
	writeRegister(REG_SYNC_WORD, sw);
}

void SX1287::enableCrc()
{
	writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void SX1287::disableCrc()
{
	writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void SX1287::enableInvertIQ()
{
	writeRegister(REG_INVERTIQ,  0x66);
	writeRegister(REG_INVERTIQ2, 0x19);
}

void SX1287::disableInvertIQ()
{
	writeRegister(REG_INVERTIQ,  0x27);
	writeRegister(REG_INVERTIQ2, 0x1d);
}

void SX1287::setOCP(uint8_t mA)
{
	uint8_t ocpTrim = 27;

	if (mA <= 120)
	{
		ocpTrim = (mA - 45) / 5;
	}
	else if (mA <=240)
	{
		ocpTrim = (mA + 30) / 10;
	}

	writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void SX1287::explicitHeaderMode()
{
	_implicitHeaderMode = false;
	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void SX1287::implicitHeaderMode()
{
	_implicitHeaderMode = true;
	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void SX1287::handleDio0Rise()
{
	int irqFlags = readRegister(REG_IRQ_FLAGS);

	// clear IRQ's
	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
	{
		if ((irqFlags & IRQ_RX_DONE_MASK) != 0)
		{
			// received a packet
			_packetIndex = 0;

			// read packet length
			int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

			// set FIFO address to current RX address
			writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
		}
		else if ((irqFlags & IRQ_TX_DONE_MASK) != 0)
		{
		}
	}
}

bool SX1287::isTransmitting()
{
	if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX)
	{
		return true;
	}

	if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
	{
		// clear IRQ's
		writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}

	return false;
}

int SX1287::getSpreadingFactor()
{
	return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

long SX1287::getSignalBandwidth()
{
	uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

	switch (bw)
	{
	case 0:
		return 7.8E3;
	case 1:
		return 10.4E3;
	case 2:
		return 15.6E3;
	case 3:
		return 20.8E3;
	case 4:
		return 31.25E3;
	case 5:
		return 41.7E3;
	case 6:
		return 62.5E3;
	case 7:
		return 125E3;
	case 8:
		return 250E3;
	case 9:
		return 500E3;
	}

	return -1;
}

void SX1287::setLdoFlag()
{
	// Section 4.1.1.5
	long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

	// Section 4.1.1.6
	bool ldoOn = symbolDuration > 16;

	uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
	config3 |= (ldoOn << 3);
	writeRegister(REG_MODEM_CONFIG_3, config3);
}





bool SX1287::writeReg(const uint8_t* txdata, size_t length)
{
	std::lock_guard<std::mutex> lock(get_guard());
	spi_transaction_t trans;
	std::memset(&trans, 0, sizeof(trans));  //Zero out the transaction
	trans.rx_buffer = nullptr;
	trans.length = 8 * length;
	trans.tx_buffer = const_cast<uint8_t*>(txdata);

	return polling_write(trans);
}

bool SX1287::readReg(const uint8_t reg, uint8_t* rxdata, size_t length)
{
	std::lock_guard<std::mutex> lock(get_guard());
	spi_transaction_t trans;
	std::memset(&trans, 0, sizeof(trans));  //Zero out the transaction
	trans.rx_buffer = rxdata;
	trans.rxlength = 8 * length;
	trans.length = 8 * length;
	trans.tx_buffer = &reg;

	return polling_write(trans);
}

uint8_t SX1287::readRegister(const uint8_t reg)
{
	bool res = readReg(reg, rxdata.data(), 2);
	return res ? rxdata[1] : 0;
}

bool SX1287::writeRegister(const uint8_t reg, const uint8_t data)
{
	smooth::core::io::spi::SpiDmaFixedBuffer<uint8_t, 4> txdata;

	// for spi write bit 7 of register address must be zero
	// modify 0xE0 to 0x60;
	txdata[0] = reg & 0x7F;

	// magic number to the reset device
	txdata[1] = data;

	return writeReg(txdata.data(), 2);
}
