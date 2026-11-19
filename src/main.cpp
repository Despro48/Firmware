#include <Arduino.h>
#include <Adafruit_MLX90640.h>
#include <SPI.h>
#include <LoRa.h>
#include <freertos/queue.h>
#include <string.h>

Adafruit_MLX90640 mlx;

#define I2C_SDA 21
#define I2C_SCL 22

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 5
#define LORA_RST 14
#define LORA_DIO0 26

#define LORA_FREQ 433E6
#define LORA_SYNCWORD 0x67
#define LORA_CODINGRATE 5
#define LORA_SPREADINGFACTOR 7
#define LORA_BANDWIDTH 500E3
#define LORA_POWER 20
#define LORA_PREAMBLE 6
#define LORA_GAIN 0

#define PIXELS (32 * 24)
#define PAYLOAD 252
#define FRAME_BYTES (PIXELS * 2)

float frame[PIXELS];
float txFrame[PIXELS];
uint8_t txBuffer[FRAME_BYTES];
QueueHandle_t frameQueue;

static uint16_t floatToHalf(float value) {
	uint32_t x;
	memcpy(&x, &value, sizeof(x));

	uint16_t sign = (uint16_t)((x >> 16) & 0x8000u);
	uint32_t expField = (x >> 23) & 0xFFu;
	uint32_t mantissa = x & 0x007FFFFFu;

	if (expField == 0xFFu) {
		return (uint16_t)(sign | (mantissa ? 0x7E00u : 0x7C00u));
	}

	int32_t halfExp = (int32_t)expField - 127 + 15;
	if (halfExp >= 0x1F) return (uint16_t)(sign | 0x7C00u);

	if (halfExp <= 0) {
		if (halfExp < -10) return sign;
		mantissa |= 0x00800000u;
		int shift = 14 - halfExp;
		uint32_t result = mantissa >> shift;
		uint32_t rem = mantissa & ((1u << shift) - 1u);
		uint32_t halfway = 1u << (shift - 1);
		if (rem > halfway || (rem == halfway && (result & 1u))) result++;
		return (uint16_t)(sign | result);
	}

	uint32_t result = mantissa >> 13;
	uint32_t rem = mantissa & 0x1FFFu;
	if (rem > 0x1000u || (rem == 0x1000u && (result & 1u))) {
		result++;
		if (result == 0x400u) {
			result = 0;
			halfExp++;
			if (halfExp >= 0x1F) return (uint16_t)(sign | 0x7C00u);
		}
	}

	return (uint16_t)(sign | ((uint16_t)halfExp << 10) | result);
}

void sendFrame(const float *frameData) {
	for (int i = 0; i < PIXELS; i++) {
		uint16_t packed = floatToHalf(frameData[i]);
		txBuffer[i * 2] = (uint8_t)(packed & 0xFF);
		txBuffer[i * 2 + 1] = (uint8_t)(packed >> 8);
	}

	uint8_t total = (FRAME_BYTES + PAYLOAD - 1) / PAYLOAD;
	for (uint8_t i = 1; i <= total; i++) {
		Serial.printf("Transmit %d/%d\n", i, total);
		size_t off = (size_t)(i - 1) * PAYLOAD;
		size_t n = FRAME_BYTES - off;
		if (n > PAYLOAD) n = PAYLOAD;
		LoRa.beginPacket();
		LoRa.write(i);
		LoRa.write(total);
		LoRa.write(txBuffer + off, n);
		LoRa.endPacket();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void mlxTask(void *parameter) {
	(void)parameter;
	for (;;) {
		if (mlx.getFrame(frame) == 0) {
			xQueueOverwrite(frameQueue, frame);
		} else {
			Serial.println("Error reading frame");
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void loraTask(void *parameter) {
	(void)parameter;
	for (;;) {
		if (xQueueReceive(frameQueue, txFrame, portMAX_DELAY) == pdTRUE) {
			sendFrame(txFrame);
		}
	}
}

void initializeLoRa() {
	SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
	LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

	if (LoRa.begin(LORA_FREQ) == 0) {
		Serial.println("LoRa init failed! Check connections");
		while (1) delay(10);
	}

	LoRa.setSyncWord(LORA_SYNCWORD);
	LoRa.setCodingRate4(LORA_CODINGRATE);
	LoRa.setSpreadingFactor(LORA_SPREADINGFACTOR);
	LoRa.setSignalBandwidth(LORA_BANDWIDTH);
	LoRa.setTxPower(LORA_POWER);
	LoRa.setPreambleLength(LORA_PREAMBLE);
	LoRa.setGain(LORA_GAIN);

	Serial.printf("LoRa ready @ %.0f MHz  CS=%d RST=%d DIO0=%d\n", LORA_FREQ / 1E6,
				LORA_CS, LORA_RST, LORA_DIO0);
}

void setup() {
	Serial.begin(115200);
	initializeLoRa();

	Wire.begin(I2C_SDA, I2C_SCL);
  
	if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
		Serial.println("MLX90640 not found! Check connections");
		while (1) delay(10);
	}

	Wire.setClock(1000000);

	mlx.setMode(MLX90640_CHESS);
	mlx.setResolution(MLX90640_ADC_18BIT);
	mlx.setRefreshRate(MLX90640_8_HZ);

	Serial.println("THERMAL CAM Initialized!");

	frameQueue = xQueueCreate(1, sizeof(frame));
	if (frameQueue == nullptr) {
		Serial.println("Failed to create frame queue");
		while (1) delay(10);
	}

	if (xTaskCreate(mlxTask, "mlx", 4096, nullptr, 2, nullptr) != pdPASS ||
		xTaskCreate(loraTask, "lora", 4096, nullptr, 1, nullptr) != pdPASS) {
		Serial.println("Failed to create RTOS tasks");
		while (1) delay(10);
	}
}

void loop() {
	delay(1000);
}