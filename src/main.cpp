#include <Arduino.h>
#include <Adafruit_MLX90640.h>
#include <SPI.h>
#include <LoRa.h>

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

#define PIXELS (32 * 24)
#define PAYLOAD 253

float frame[PIXELS];
char line[PIXELS * 10];

void sendFrame() {
	size_t len = 0;
	for (int i = 0; i < PIXELS; i++) {
		len += snprintf(
			line + len, 
			sizeof(line) - len, 
			"%0.2f%c", frame[i],
			i < PIXELS - 1 ? ',' : '\n'
		);
	}

	uint8_t total = (len + PAYLOAD - 1) / PAYLOAD;
	for (uint8_t i = 0; i < total; i++) {
		Serial.printf("Transmitting: %d/%d\n", i, total);

		size_t off = i * PAYLOAD;
		size_t n = (len - off < PAYLOAD) ? len - off : PAYLOAD;
		LoRa.beginPacket();
		LoRa.write(i);
		LoRa.write(total);
		LoRa.write((uint8_t *)line + off, n);
		LoRa.endPacket();
		delay(5);
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

	if (LoRa.begin(LORA_FREQ) == 0) {
		Serial.println("LoRa init failed! Check connections");
		while (1) {
			delay(10);
		}
	}

	Serial.printf("LoRa ready @ %.0f MHz  CS=%d RST=%d DIO0=%d\n", LORA_FREQ / 1E6,
				LORA_CS, LORA_RST, LORA_DIO0);
}

void setup() {
	Serial.begin(115200);
	initializeLoRa();

	Wire.begin(I2C_SDA, I2C_SCL);
	Wire.setClock(1000000);
  
	if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
		Serial.println("MLX90640 not found! Check connections");
		while (1) delay(10);
	}

	mlx.setResolution(MLX90640_ADC_18BIT);
	mlx.setRefreshRate(MLX90640_16_HZ);
	mlx.setMode(MLX90640_CHESS);

	Serial.println("THERMAL CAM Initialized!");
}

void loop() {
	if (mlx.getFrame(frame) == 0) {
		sendFrame();
	} else {
		Serial.println("Error reading frame");
	}
}