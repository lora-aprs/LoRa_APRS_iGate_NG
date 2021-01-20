#ifndef LORA_H_
#define LORA_H_

#include "smooth/core/Task.h"

#include "hw/sx1287.h"

class LoRaTask : public smooth::core::Task
{
public:
	explicit LoRaTask();

	void setSXdevice(std::unique_ptr<SX1287> sx1287);

	void tick() override;

private:
	std::unique_ptr<SX1287> _sx1287;
};

#endif
