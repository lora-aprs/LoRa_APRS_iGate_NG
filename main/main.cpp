
#include <chrono>

#include "smooth/core/logging/log.h"
#include "smooth/core/task_priorities.h"

#include "main.h"

static const char* TAG = "APP";

App::App() :
	Application(smooth::core::APPLICATION_BASE_PRIO, std::chrono::seconds(1)),
	spi_host(VSPI_HOST),            // Use VSPI as host
	spi_master(
		spi_host,                   // host VSPI
		smooth::core::io::spi::SPI_DMA_Channel::DMA_1, // use DMA
		GPIO_NUM_27,                // mosi gpio pin
		GPIO_NUM_19,                // miso gpio pin  (full duplex)
		GPIO_NUM_5,                 // clock gpio pin
		0)                          // max transfer size default of 4096
{
}

void App::init()
{
	Application::init();

	init_LoRa();

	lora_task.start();
}

void App::init_LoRa()
{
	std::unique_ptr<SX1287> sx1287 = spi_master.create_device<SX1287>(
					GPIO_NUM_18,            // chip select gpio pin
					0,                      // spi command_bits
					0,                      // spi address_bits,
					0,                      // bits_between_address_and_data_phase,
					0,                      // spi_mode = 0,
					128,                    // spi positive_duty_cycle,
					0,                      // spi cs_ena_posttrans,
					SPI_MASTER_FREQ_10M,    // spi-sck = 10MHz
					0,                      // full duplex (4-wire)
					2,                      // queue_size,
					false,                  // will not use pre-trans callback
					false);                 // will not use post-trans callback

	if(!sx1287->init(spi_host))
	{
		Log::error(TAG, "Initializing of SPI Device: SX1287 --- FAILED");
	}
	Log::info(TAG, "SX1287 initialized, ID: {}", sx1287->read_id());
	lora_task.setSXdevice(std::move(sx1287));
}

void App::tick()
{
	//std::cout << "Tick!" << std::endl;
}


extern "C" void app_main(void)
{
	App app;
	app.start();
}
