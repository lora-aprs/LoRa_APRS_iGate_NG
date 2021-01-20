
#include <chrono>
#include "lora.h"

LoRaTask::LoRaTask() :
	smooth::core::Task("LoRaTask", 4096, 10, std::chrono::seconds(1))
{
}

void LoRaTask::setSXdevice(std::unique_ptr<SX1287> sx1287)
{
	_sx1287 = std::move(sx1287);
}

void LoRaTask::tick()
{
	Log::info("LoRaTask", "SX1287 initialized, ID: {}", _sx1287->read_id());
}
