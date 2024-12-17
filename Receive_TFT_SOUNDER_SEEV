/*Nome ALUNO A- Guilherme Miguel Filipe Inacio
Nome ALUNO B- Tiago Silva Bastos
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
EAU- Licenciatura em Engenharia Automovel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos*/

#include "Arduino.h"
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <esp32_can.h>
#include "Audio.h"
#include "SD.h"
#include "FS.h"
#include <TFT_eSPI.h> // Biblioteca para o display TFT

// Variáveis globais para CAN
volatile int velocidade = 0;
volatile bool ignicao = false;
volatile int marcha = 0;
volatile bool audioPlaying = false; // Nova variável para controlar o estado do áudio

// Mutex para proteger o acesso ao barramento SPI e variáveis globais
SemaphoreHandle_t xSPIMutex;
SemaphoreHandle_t xDataMutex;
SemaphoreHandle_t xAudioDisplayMutex;

// TFT connections
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4

// microSD Card Reader connections
#define SD_CS     17
#define SPI_MOSI  23
#define SPI_MISO  19
#define SPI_SCK   18

// I2S Connections
#define I2S_DOUT  33
#define I2S_BCLK  14
#define I2S_LRC   27

// Objeto de áudio
Audio audio;
String arquivoAtual = "";

// Objeto para o display TFT
TFT_eSPI tft = TFT_eSPI();

// Inicializa o barramento SPI
SPIClass vspi(VSPI);

// Função para inicializar o CAN
void initCAN() {
  CAN0.setCANPins(GPIO_NUM_25, GPIO_NUM_26); // RX, TX
  CAN0.begin(500000);
  CAN0.watchFor(0x100); // Filtra mensagens com ID 0x100
  Serial.println("CAN inicializado com sucesso.");
}

// Função para inicializar o SD
void initSD() {
  if (!SD.begin(SD_CS, vspi)) {
    Serial.println("Erro ao acessar o cartao microSD!");
    while (true);
  }
  Serial.println("Cartao microSD inicializado com sucesso.");
}

// Função para inicializar o I2S e o áudio
void initAudio() {
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(10); // Volume inicial
  Serial.println("I2S e audio configurados.");
}

// Função para inicializar o display TFT
void initTFT() {
  tft.init();
  tft.setRotation(1);         // Define a orientação (0-3)
  tft.fillScreen(TFT_BLACK);  // Preenche a tela com preto
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  Serial.println("Display TFT inicializado.");
}

// Função para processar mensagens CAN
void processaCANFrame(CAN_FRAME *frame) {
  if (frame->id == 0x100) {
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
      velocidade = frame->data.uint8[0];
      ignicao = frame->data.uint8[1] == 1;
      marcha = frame->data.uint8[2];
      xSemaphoreGive(xDataMutex);
    }

    // Debug
    Serial.printf("CAN: Velocidade=%d, Ignicao=%s, Marcha=%d\n",
                  velocidade, ignicao ? "ON" : "OFF", marcha);
  }
}

// Tarefa para processar mensagens CAN
void taskProcessaCAN(void *parameter) {
  CAN_FRAME frame;
  while (true) {
    if (CAN0.read(frame)) {
      processaCANFrame(&frame);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Tarefa para gerenciar o áudio
void taskAudio(void *parameter) {
  initSD();
  initAudio();

  if (!audio.connecttoFS(SD, "Ignition.mp3")) {
    Serial.println("Erro ao abrir arquivo de inicializacao!");
    while (true);
  }
  arquivoAtual = "Ignition.mp3";
  Serial.println("Audio inicial carregado.");

  while (true) {
    if (xSemaphoreTake(xAudioDisplayMutex, portMAX_DELAY)) {
      String novoArquivo;

      // Escolha de arquivo baseada nos estados
      if (ignicao) {
        novoArquivo = "Ignition.mp3";
      } else {
        if (marcha == 0) novoArquivo = "Neutral.mp3";
        else if (marcha == 1) novoArquivo = "Drive.mp3";
        else if (marcha == 2) novoArquivo = "Reverse.mp3";
      }

      // Verifica se o arquivo precisa ser alterado
      if (novoArquivo != arquivoAtual) {
        if (!audio.connecttoFS(SD, novoArquivo.c_str())) {
          Serial.printf("Erro ao carregar arquivo: %s\n", novoArquivo.c_str());
        } else {
          arquivoAtual = novoArquivo;
          Serial.printf("Tocando: %s\n", novoArquivo.c_str());
        }
      }

      audioPlaying = true; // Indica que o áudio está tocando
      xSemaphoreGive(xAudioDisplayMutex);

      while (audio.isRunning()) {
        audio.loop(); // Garante a reprodução contínua
      }
      audioPlaying = false; // Indica que o áudio parou de tocar
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Tarefa para atualizar o display
void taskAtualizaDisplay(void *parameter) {
  initTFT();
  while (true) {
    if (xSemaphoreTake(xAudioDisplayMutex, portMAX_DELAY)) {
      if (xSemaphoreTake(xSPIMutex, portMAX_DELAY)) {
        int velocidadeLocal;
        bool ignicaoLocal;
        int marchaLocal;

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
          velocidadeLocal = velocidade;
          ignicaoLocal = ignicao;
          marchaLocal = marcha;
          xSemaphoreGive(xDataMutex);
        }

        tft.fillScreen(TFT_BLACK);

        // Exibe a velocidade
        String textoVelocidade = "Velocidade: " + String(velocidadeLocal) + " km/h";
        tft.setTextColor(TFT_WHITE);
        tft.setCursor((128 - tft.textWidth(textoVelocidade)) / 2, 20);
        tft.print(textoVelocidade);

        // Exibe o estado da ignição
        String estadoIgnicao = "Igni: " + String(ignicaoLocal ? "ON" : "OFF");
        tft.setCursor((128 - tft.textWidth(estadoIgnicao)) / 2, 60);
        tft.print(estadoIgnicao);

        // Exibe a marcha
        String estadoMarcha;
        if (marchaLocal == 1) estadoMarcha = "Drive";
        else if (marchaLocal == 2) estadoMarcha = "Reverse";
        else estadoMarcha = "Neutral";

        String textoMarcha = "Marcha: " + estadoMarcha;
        tft.setCursor((128 - tft.textWidth(textoMarcha)) / 2, 100);
        tft.print(textoMarcha);

        xSemaphoreGive(xSPIMutex);
      }
      xSemaphoreGive(xAudioDisplayMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Atualiza a cada 200ms
  }
}

// Configuração inicial
void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando sistema...");

  // Criar os mutexes para proteger os dados e o barramento SPI
  xSPIMutex = xSemaphoreCreateMutex();
  xDataMutex = xSemaphoreCreateMutex();
  xAudioDisplayMutex = xSemaphoreCreateMutex();

  if (!xSPIMutex || !xDataMutex || !xAudioDisplayMutex) {
    Serial.println("Erro ao criar os Mutexes!");
    while (true);
  }

  // Inicializar CAN
  initCAN();

  // Inicializar o barramento SPI
  vspi.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Criar tarefas
  xTaskCreate(taskProcessaCAN, "TaskProcessaCAN", 4096, NULL, 3, NULL);
  xTaskCreate(taskAudio, "TaskAudio", 4096, NULL, 1, NULL);
  xTaskCreate(taskAtualizaDisplay, "TaskAtualizaDisplay", 4096, NULL, 2, NULL);

  Serial.println("Sistema pronto!");
}

// Loop principal vazio, pois usamos FreeRTOS
void loop() {
  vTaskDelete(NULL); // Loop principal não é usado em aplicações FreeRTOS
}
