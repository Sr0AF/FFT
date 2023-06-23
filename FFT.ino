#include <Arduino.h>
#include <arduinoFFT.h>

#define SAMPLES 1024              // Tamaño de la muestra (potencia de 2)
#define SAMPLING_FREQUENCY 10000  // Frecuencia de muestreo en Hz

arduinoFFT FFT = arduinoFFT();    // Crear una instancia de la clase arduinoFFT

double vReal[SAMPLES];
double vImag[SAMPLES];

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);       // Configurar la resolución de lectura analógica en 12 bits
}

void loop() {
  // Leer la señal analógica
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(34);    // Leer el valor analógico y almacenarlo en el vector vReal
    vImag[i] = 0;                 // Configurar los valores imaginarios en 0
    delayMicroseconds(100);       // Esperar un breve tiempo antes de leer la siguiente muestra
  }

  // Aplicar la FFT
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);     // Calcular la FFT
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);       // Convertir la salida a amplitud

  // Encontrar el índice de la frecuencia fundamental (el máximo en el espectro)
  double maxValue = 0;
  unsigned int index = 0;
  for (int i = 0; i < (SAMPLES / 2); i++) {
    if (vReal[i] > maxValue) {
      maxValue = vReal[i];
      index = i;
    }
  }

  // Calcular la frecuencia fundamental estimada
  double frecuencia_fundamental_estimada = (index * SAMPLING_FREQUENCY) / SAMPLES;

  Serial.print("Frecuencia fundamental estimada: ");
  Serial.print(frecuencia_fundamental_estimada);
  Serial.println(" Hz");

  delay(1000);                    // Esperar un segundo antes de realizar la siguiente medición
}
