#ifndef MAIN_H_
#define MAIN_H_

#include "smooth/core/Application.h"

#include "lora.h"
#include "hw/sx1287.h"

class App : public smooth::core::Application
{
	public:
		App();

		void init() override;

		void init_LoRa();

		void tick() override;

	private:
		spi_host_device_t spi_host;
		smooth::core::io::spi::Master spi_master;

		LoRaTask lora_task;
};

#endif
