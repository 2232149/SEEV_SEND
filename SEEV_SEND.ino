/*Nome ALUNO A- Guilherme Miguel Filipe Inacio
Nome ALUNO B- Tiago Silva Bastos
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
EAU- Licenciatura em Engenharia Automovel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos*/
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <esp32_can.h>

#define SENDER_POT 0
#define SENDER_BOT 1

// Configuração dos pinos
const int POT_PIN = 34;     // Potenciômetro no GPIO 34 (entrada analógica)
const int BTN_IGN_PIN = 14; // Botão de ignição (ON/OFF) no GPIO 13
const int BTN_DRV_PIN = 12; // Botão de Drive no GPIO 12
const int BTN_REV_PIN = 13; // Botão de Reverse no GPIO 14

// Variáveis globais
volatile bool ignicao = false;
volatile int marcha = 1;
QueueHandle_t canQueue;

// Estrutura de mensagem CAN
CAN_FRAME txFrame;

// Estrutura para dados
struct CANData {
    int sender; // 0 POT, 1 BOT
    int velocidade;
    bool ignicao;
    int marcha;  // 1 = Drive, 0 = Neutral, 2 = Reverse
};

// Declaração das funções
void taskPotenciometro(void *parameter);
void taskEnvioCAN(void *parameter);
void IRAM_ATTR isrIgnicao();
void IRAM_ATTR isrDrive();
void IRAM_ATTR isrReverse();

void setup() {
    // Set loopTask max priority before deletion
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    Serial.begin(115200);

    // Configuração dos pinos
    pinMode(POT_PIN, INPUT);
    pinMode(BTN_IGN_PIN, INPUT_PULLUP);
    pinMode(BTN_DRV_PIN, INPUT_PULLUP);
    pinMode(BTN_REV_PIN, INPUT_PULLUP);

    // Inicialização do CAN
    CAN0.setCANPins(GPIO_NUM_25, GPIO_NUM_26); // rx, tx
    CAN0.enable();
    CAN0.begin(500000);

    // Criação da fila
    canQueue = xQueueCreate(10, sizeof(CANData));

    // Criação de tarefas
    xTaskCreate(taskPotenciometro, "Potenciometro", 2048, NULL, 1, NULL);
    xTaskCreate(taskEnvioCAN, "EnvioCAN", 2048, NULL, 2, NULL);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(BTN_IGN_PIN), isrIgnicao, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BTN_DRV_PIN), isrDrive, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BTN_REV_PIN), isrReverse, CHANGE);
}

// Tarefa para ler o potenciômetro
void taskPotenciometro(void *parameter) {
    int velocidade = 0;
    static int velocidadeAnterior = -1;
    while (1) {
        int leitura = analogRead(POT_PIN);
        velocidade = map(leitura, 0, 4095, 0, 150); // Mapeia para 0-150 km/h

        if (velocidade != velocidadeAnterior) {
            CANData data = { SENDER_POT, velocidade, 0, 0 };
            xQueueSend(canQueue, &data, portMAX_DELAY);
            velocidadeAnterior = velocidade;
            Serial.println("POT");
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Atualiza a cada 500 ms
    }
}

// Interrupt Service Routines (ISRs)
void IRAM_ATTR isrIgnicao() {
    ignicao = digitalRead(BTN_IGN_PIN) == LOW;
    CANData data = { SENDER_BOT, 0, ignicao, marcha };
    xQueueSendFromISR(canQueue, &data, NULL);
}

void IRAM_ATTR isrDrive() {
    if (digitalRead(BTN_DRV_PIN) == LOW) {
        marcha = 1;
    } else {
        marcha = 0;
    }
    CANData data = { SENDER_BOT, 0, ignicao, marcha };
    xQueueSendFromISR(canQueue, &data, NULL);
}

void IRAM_ATTR isrReverse() {
    if (digitalRead(BTN_REV_PIN) == LOW) {
        marcha = 2;
    } else {
        marcha = 0;
    }
    CANData data = { SENDER_BOT, 0, ignicao, marcha };
    xQueueSendFromISR(canQueue, &data, NULL);
}

// Tarefa para enviar os dados via CAN
void taskEnvioCAN(void *parameter) {
    CANData tmp_data, data;
    while (1) {
        // Aguarda novos dados na fila
        if (xQueueReceive(canQueue, &tmp_data, portMAX_DELAY)) {
            if (tmp_data.sender == SENDER_POT) {
                data.velocidade = tmp_data.velocidade;
            } else if (tmp_data.sender == SENDER_BOT) {
                data.ignicao = tmp_data.ignicao;
                data.marcha = tmp_data.marcha;
            }
            // Prepara a mensagem CAN
            txFrame.rtr = 0;
            txFrame.id = 0x100; // ID arbitrário da mensagem
            txFrame.extended = false;
            txFrame.length = 3;
            txFrame.data.uint8[0] = (uint8_t) data.velocidade; // Velocidade
            txFrame.data.uint8[1] = (uint8_t) data.ignicao ? 1 : 0;
            txFrame.data.uint8[2] = (uint8_t) data.marcha;

            // Envia a mensagem
            CAN0.sendFrame(txFrame);

            // Log para debug
            Serial.printf("Enviado: Velocidade=%d, Ignicao=%d, Marcha=%d\n", data.velocidade, data.ignicao, data.marcha);
        }
    }
}

void loop() {
    // O loop principal está vazio pois usamos FreeRTOS
    vTaskDelete(NULL);
}
