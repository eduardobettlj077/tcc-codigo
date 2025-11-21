// ======================================================================
//        Sistema de Alimentação Automatizada - RFID + Modbus + Motores
// ======================================================================

// ----- Bibliotecas -----
#include <SPI.h>
#include <MFRC522.h>
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

// ----- Definição dos pinos do RFID -----
#define SS_PIN 53
#define RST_PIN 40

// ----- Pinos do Motor 1 -----
#define MOTOR1_IN1 22
#define MOTOR1_IN2 23
#define MOTOR1_IN3 24
#define MOTOR1_IN4 25

// ----- Pinos do Motor 2 -----
#define MOTOR2_IN1 28
#define MOTOR2_IN2 29
#define MOTOR2_IN3 30
#define MOTOR2_IN4 31

// ----- Pinos Modbus -----
#define LED_PIN 13
#define DIGITAL_INPUT_PIN 2

// ----- Constantes -----
const unsigned long TIMEOUT_ESPERA = 30000;
const unsigned long MAX_TEMPO_MOTOR = 300000;
const unsigned long MAX_INTERVALO = 60000;

const float FATOR_CONVERSAO_MOTOR1 = 15.0;
const float FATOR_CONVERSAO_MOTOR2 = 15.0;

int velocidadeMotor1 = 20;
int velocidadeMotor2 = 20;

// ----- Tags RFID válidas -----
byte uid1[] = {0x36, 0x5E, 0x81, 0x8D};
byte uid2[] = {0x26, 0xEC, 0xFE, 0x93};
byte uid3[] = {0xB6, 0x52, 0x85, 0x8D};

// ----- Objetos principais -----
MFRC522 mfrc522(SS_PIN, RST_PIN);
modbusDevice regBank;
modbusSlave slave;

// ----- Variáveis de controle -----
bool aguardandoComando = false;
unsigned long tempoInicioEspera = 0;

// ----- Status dos motores em tempo real -----
bool motor1Ligado = false;
bool motor2Ligado = false;

// ======================================================================
//                              SETUP
// ======================================================================
void setup() {
  configurarPinos();
  configurarModbus();
  configurarRFID();
}

// ======================================================================
//                              LOOP
// ======================================================================
void loop() {
  slave.run();
  atualizarModbusIO();

  if (aguardandoComando) {
    processarComandoModbus();
  } else {
    processarLeituraRFID();
  }
}

// ======================================================================
//                          CONFIGURAÇÕES
// ======================================================================
void configurarPinos() {
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_IN3, OUTPUT);
  pinMode(MOTOR1_IN4, OUTPUT);

  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);

  pinMode(DIGITAL_INPUT_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  desligarMotores();
}

void configurarModbus() {
  regBank.setId(1);
  regBank.add(1);          // LED
  regBank.add(10002);      // Input digital
  regBank.add(30001);      // UID byte 0
  regBank.add(30002);      // UID byte 1
  regBank.add(30003);      // UID byte 2
  regBank.add(30004);      // UID byte 3
  regBank.add(30006);      // NovaTag detectada
  regBank.add(40001);      // Peso granulado
  regBank.add(40002);      // Peso sal
  regBank.add(40003);      // Motor1 ligado/desligado
  regBank.add(40004);      // Motor2 ligado/desligado
  regBank.add(40015);      // Intervalo
  
  // ----- STATUS DOS MOTORES (Coils) -----
  regBank.add(2);          // Motor1 ligado (coil 00002)
  regBank.add(3);          // Motor2 ligado (coil 00003)
  
  // ----- TIMESTAMP DA ÚLTIMA TAG LIDA -----
  regBank.add(40010);      // Timestamp da última tag (holding register)

  slave._device = &regBank;
  slave.setBaud(9600);

  // Inicializar registradores
  regBank.set(30001, 0);
  regBank.set(30002, 0);
  regBank.set(30003, 0);
  regBank.set(30004, 0);
  regBank.set(30006, 0);
  regBank.set(40001, 0);
  regBank.set(40002, 0);
  regBank.set(40003, 0);   // Motor1 = OFF
  regBank.set(40004, 0);   // Motor2 = OFF
  regBank.set(40015, 10);
  regBank.set(40010, 0);
  
  // Inicializar motores como desligados
  regBank.set(2, 0);       // Motor1 = OFF
  regBank.set(3, 0);       // Motor2 = OFF
}

void configurarRFID() {
  SPI.begin();
  mfrc522.PCD_Init();
  delay(100);

  byte versao = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  if (versao == 0x92 || versao == 0x91)
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
}

// ======================================================================
//                     FUNÇÕES MODBUS / RFID / MOTORES
// ======================================================================
void atualizarModbusIO() {
  // Atualizar LED
  byte ledStatus = regBank.get(1);
  digitalWrite(LED_PIN, ledStatus ? HIGH : LOW);

  // Atualizar entrada digital
  byte inputStatus = digitalRead(DIGITAL_INPUT_PIN);
  regBank.set(10002, inputStatus ? 1 : 0);
  
  // Manter status dos motores sempre atualizado
  regBank.set(2, motor1Ligado ? 1 : 0);
  regBank.set(3, motor2Ligado ? 1 : 0);
  regBank.set(40003, motor1Ligado ? 1 : 0);
  regBank.set(40004, motor2Ligado ? 1 : 0);
}

void processarComandoModbus() {
  word peso1 = regBank.get(40001);
  word peso2 = regBank.get(40002);

  if (peso1 > 0 && peso2 > 0) {
    executarSequencia(peso1, peso2);
    resetarComando();
    delay(2000);
    return;
  }

  if (millis() - tempoInicioEspera > TIMEOUT_ESPERA)
    resetarComando();
}

void processarLeituraRFID() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    delay(1000);
    return;
  }

  if (!mfrc522.PICC_ReadCardSerial()) {
    delay(1000);
    return;
  }

  // Armazenar UID da tag lida
  armazenarUID();

  if (validarTag()) {
    // Tag válida detectada - sinalizar ao supervisório
    aguardandoComando = true;
    tempoInicioEspera = millis();
    regBank.set(30006, 1);      // NovaTag detectada
    regBank.set(40010, 1);      // Timestamp da detecção
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  delay(500);
}

void armazenarUID() {
  regBank.set(30001, mfrc522.uid.uidByte[0]);
  regBank.set(30002, mfrc522.uid.uidByte[1]);
  regBank.set(30003, mfrc522.uid.uidByte[2]);
  regBank.set(30004, mfrc522.uid.uidByte[3]);
}

bool validarTag() {
  byte* uid = mfrc522.uid.uidByte;
  byte size = mfrc522.uid.size;

  return compararUID(uid, uid1, size) ||
         compararUID(uid, uid2, size) ||
         compararUID(uid, uid3, size);
}

bool compararUID(byte *u1, byte *u2, byte size) {
  for (byte i = 0; i < size; i++) {
    if (u1[i] != u2[i]) return false;
  }
  return true;
}

// ======================================================================
//                          MOTORES
// ======================================================================
void executarSequencia(word peso1, word peso2) {
  word intervalo = regBank.get(40015);
  unsigned long tempo1 = (peso1 / FATOR_CONVERSAO_MOTOR1) * 1000UL;
  unsigned long tempo2 = (peso2 / FATOR_CONVERSAO_MOTOR2) * 1000UL;
  unsigned long intervaloMs = (intervalo * 1000UL);

  acionarMotor1(tempo1);
  aguardarIntervalo(intervaloMs);
  acionarMotor2(tempo2);
}

void acionarMotor1(unsigned long duracao) {
  motor1Ligado = true;      // Sinalizar que motor1 está ligado
  regBank.set(2, 1);        // Atualizar Modbus imediatamente
  regBank.set(40003, 1);    // Registrador 40003
  
  unsigned long inicio = millis();
  while (millis() - inicio < duracao) {
    executarPassoMotor1();
    slave.run();             // Manter comunicação Modbus durante execução
  }
  
  desligarMotor1();
  motor1Ligado = false;      // Sinalizar que motor1 está desligado
  regBank.set(2, 0);        // Atualizar Modbus
  regBank.set(40003, 0);    // Registrador 40003
}

void acionarMotor2(unsigned long duracao) {
  motor2Ligado = true;      // Sinalizar que motor2 está ligado
  regBank.set(3, 1);        // Atualizar Modbus imediatamente
  regBank.set(40004, 1);    // Registrador 40004
  
  unsigned long inicio = millis();
  while (millis() - inicio < duracao) {
    executarPassoMotor2();
    slave.run();             // Manter comunicação Modbus durante execução
  }
  
  desligarMotor2();
  motor2Ligado = false;      // Sinalizar que motor2 está desligado
  regBank.set(3, 0);        // Atualizar Modbus
  regBank.set(40004, 0);    // Registrador 40004
}

void aguardarIntervalo(unsigned long intervalo) {
  for (unsigned long i = 0; i < intervalo; i += 1000) {
    delay(1000);
    slave.run();
  }
}

void resetarComando() {
  aguardandoComando = false;
  regBank.set(40001, 0);
  regBank.set(40002, 0);
  regBank.set(30006, 0);      // Resetar NovaTag
  regBank.set(40010, 0);      // Resetar timestamp
}

// ======================================================================
//                      PASSOS DOS MOTORES
// ======================================================================
void executarPassoMotor1() {
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR1_IN3, HIGH);
  digitalWrite(MOTOR1_IN4, LOW);
  delay(velocidadeMotor1);

  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  digitalWrite(MOTOR1_IN3, HIGH);
  digitalWrite(MOTOR1_IN4, LOW);
  delay(velocidadeMotor1);

  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  digitalWrite(MOTOR1_IN3, LOW);
  digitalWrite(MOTOR1_IN4, HIGH);
  delay(velocidadeMotor1);

  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR1_IN3, LOW);
  digitalWrite(MOTOR1_IN4, HIGH);
  delay(velocidadeMotor1);
}

void executarPassoMotor2() {
  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  delay(velocidadeMotor2);

  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, HIGH);
  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  delay(velocidadeMotor2);

  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, HIGH);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, HIGH);
  delay(velocidadeMotor2);

  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, HIGH);
  delay(velocidadeMotor2);
}

// ======================================================================
//                          DESLIGAMENTO
// ======================================================================
void desligarMotor1() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR1_IN3, LOW);
  digitalWrite(MOTOR1_IN4, LOW);
}

void desligarMotor2() {
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
}

void desligarMotores() {
  desligarMotor1();
  desligarMotor2();
  motor1Ligado = false;
  motor2Ligado = false;
  
  // Garantir que o supervisório mostre motores desligados
  regBank.set(2, 0);
  regBank.set(3, 0);
}