#ifndef SX1287_H_
#define SX1287_H_

#include "smooth/core/io/spi/Master.h"
#include "smooth/core/io/spi/SPIDevice.h"
#include "smooth/core/io/spi/SpiDmaFixedBuffer.h"

#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

class SX1287 : public smooth::core::io::spi::SPIDevice
{
public:
	SX1287(std::mutex& guard,
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
			bool use_post_transaction_callback);

	bool init(spi_host_device_t host);

	uint8_t read_id();

	bool begin(long frequency);
	void end();

	bool beginPacket();
	void endPacket();

	size_t write(uint8_t byte);
	size_t write(const uint8_t *buffer, size_t size);
	int available();
	int read();
	int peek();

	int parsePacket(int size = 0);
	int packetRssi();
	float packetSnr();
	long packetFrequencyError();

	void idle();
	void sleep();

	void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
	void setFrequency(long frequency);
	void setSpreadingFactor(int sf);
	void setSignalBandwidth(long sbw);
	void setCodingRate4(int denominator);
	void setPreambleLength(long length);
	void setSyncWord(int sw);
	void enableCrc();
	void disableCrc();
	void enableInvertIQ();
	void disableInvertIQ();

	void setOCP(uint8_t mA);

private:
	void explicitHeaderMode();
	void implicitHeaderMode();

	void handleDio0Rise();
	bool isTransmitting();

	int getSpreadingFactor();
	long getSignalBandwidth();

	void setLdoFlag();

	long _frequency;
	int _packetIndex;
	bool _implicitHeaderMode;

	bool writeReg(const uint8_t* txdata, size_t length);
	bool readReg(const uint8_t reg, uint8_t* rxdata, size_t length);

	uint8_t readRegister(const uint8_t reg);
	bool writeRegister(const uint8_t reg, const uint8_t data);

	gpio_num_t cs_pin;
	smooth::core::io::spi::SpiDmaFixedBuffer<uint8_t, 4> rxdata;
};

#endif
