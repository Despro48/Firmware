#include <Arduino.h>
#include <Adafruit_MLX90640.h>

Adafruit_MLX90640 mlx;

#define I2C_SDA 21
#define I2C_SCL 22

float frame[32 * 24];

void setup() {
	Serial.begin(921600);
	Serial.println("- THERMAL CAM READER -");

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
		for (int i = 0; i < 32 * 24 - 1; i++) {
			Serial.print(frame[i]);
			Serial.print(",");
		}
		Serial.println(frame[32 * 24 - 1]);
	} else {
		Serial.println("Error reading frame");
	}
}