#include <Arduino.h>
#include <Adafruit_MLX90640.h>
#include <SPI.h>
#include <LoRa.h>
#include <freertos/queue.h>
#include <string.h>

#include "thermal_codec_a.h"

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
#define PACKET_GAP_MS 20

float frame[PIXELS];
float txFrame[PIXELS];
thermal_a::Packet txPackets[thermal_a::kMaxPackets];
QueueHandle_t frameQueue;

void sendFrame(const float *frameData, uint8_t frameId) {
	const int packetCount = thermal_a::encodeFrame(frameData, frameId, txPackets);
	if (packetCount == 0) {
		Serial.printf("Frame %u: profile-A encode failed\n", frameId);
		return;
	}

	for (int i = 0; i < packetCount; i++) {
		Serial.printf("TX A frame=%u strip=%u part=%u k=%u len=%u%s\n", frameId,
					txPackets[i].stripId, txPackets[i].part, txPackets[i].riceK,
					txPackets[i].length, txPackets[i].split ? " split" : "");
		LoRa.beginPacket();
		LoRa.write(txPackets[i].data, txPackets[i].length);
		if (LoRa.endPacket() == 0) {
			Serial.println("LoRa TX failed");
		}
		vTaskDelay(pdMS_TO_TICKS(PACKET_GAP_MS));
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
	uint8_t frameId = 0;
	for (;;) {
		if (xQueueReceive(frameQueue, txFrame, portMAX_DELAY) == pdTRUE) {
			sendFrame(txFrame, frameId);
			frameId++;
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