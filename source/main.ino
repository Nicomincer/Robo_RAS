#include <Servo.h>

// ================= PINOS =================
const int PIN_TRIG = 7;
const int PIN_ECHO = 6;
const int PIN_BASE = 5;  // Servo 1 - giro da base
const int PIN_OMBRO = 2;  // Servo 2
const int PIN_COTOVELO = 3; // Servo 3
const int PIN_PUNHO = 4;  // Servo 4
const int PIN_GARRA = 8;  // Servo 5

// ================= CONFIGURAÇÕES =================
const float DIST_MIN_CM     = 3.0;   // HC-SR04 não lê bem abaixo de ~2 cm
const float DIST_ALCANCE_CM = 40.0;  // objeto a até 40 cm: o braço tenta pegar
const float DIST_AGARRAR_CM = 12.0;  // a esta distância a garra fecha (CALIBRE)
const int   LEITURAS_CONFIRMA = 3;   // leituras seguidas para confirmar objeto

// Poses (CALIBRE com o braço montado)
const int POSE_BUSCA[4]   = {90, 90, 45, 0};   // ombro, cotovelo, punho, garra
const int POSE_ALCANCE[4] = {60, 60, 90, 0};   // braço estendido, garra aberta
const int POSE_CARGA[4]   = {90, 90, 45, 90};  // braço recolhido, garra fechada
const int GARRA_ABERTA  = 0;
const int GARRA_FECHADA = 90;

Servo sBase, sOmbro, sCotovelo, sPunho, sGarra;

int angBase = 0, dirBase = 1;
int angOmbro = 90, angCotovelo = 90, angPunho = 45, angGarra = 0;

enum Estado { BUSCA, ALCANCE, AGARRA, RECOLHE, FIM };
Estado estado = BUSCA;
int confirmacoes = 0;

// ================= SENSOR =================
float lerDistanciaCmUnica() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duracao = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (duracao == 0) return -1;
  return duracao * 0.0343 / 2.0;
}

float maior(float a, float b, float c){
    float m = a;
    if(b > m){
      m = b;
    }
    if(c > m){
      m = c;
    }

    return m;
}

float menor(float a, float b, float c){
    float m = a;
    if(b < m){
      m = b;
    }
    if(c < m){
      m = c;
    }
    return m;
}

// Mediana de 3 leituras para reduzir ruído
float lerDistanciaCm() {
  float a = lerDistanciaCmUnica(); delay(30);
  float b = lerDistanciaCmUnica(); delay(30);
  float c = lerDistanciaCmUnica();
  float Maior = maior(a, b, c);
  float Menor = menor(a, b, c);
  float mediana = (a+b+c)-Maior-Menor;
  return mediana;
}

// ================= MOVIMENTO SUAVE =================
void moverSuave(Servo &s, int &atual, int alvo, int passoMs = 15) {
  while (atual != alvo) {
    atual += (alvo > atual) ? 1 : -1;
    s.write(atual);
    delay(passoMs);
  }
}

void irParaPose(const int pose[4]) {
  // Move uma articulação por vez (menos corrente de pico)
  moverSuave(sOmbro, angOmbro, pose[0]);
  moverSuave(sCotovelo, angCotovelo, pose[1]);
  moverSuave(sPunho, angPunho, pose[2]);
  moverSuave(sGarra, angGarra, pose[3], 8);
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  sBase.attach(PIN_BASE);       sBase.write(angBase);
  sOmbro.attach(PIN_OMBRO);     sOmbro.write(angOmbro);
  sCotovelo.attach(PIN_COTOVELO); sCotovelo.write(angCotovelo);
  sPunho.attach(PIN_PUNHO);     sPunho.write(angPunho);
  sGarra.attach(PIN_GARRA);     sGarra.write(angGarra);

  Serial.println("Sistema iniciado");
}

// ================= LOOP =================
void loop() {
  switch (estado) {

  case BUSCA: {
  float d = lerDistanciaCm(); // mede ANTES de mover
  Serial.println(d);
  bool visto = (d >= DIST_MIN_CM && d <= DIST_ALCANCE_CM);

  if (visto) {
    // Base parada: confirma o objeto sem se mexer
    confirmacoes++;
    if (confirmacoes >= LEITURAS_CONFIRMA) {
      Serial.print("OBJETO CONFIRMADO - ");
      Serial.print(d);
      Serial.println(" cm");
      estado = ALCANCE;
    }
  } else {
    // Nada à vista: continua a varredura
    confirmacoes = 0;
    angBase += dirBase * 2;
    if (angBase >= 180) { angBase = 180; dirBase = -1; }
    if (angBase <= 0)   { angBase = 0;   dirBase = 1; }
    sBase.write(angBase);
  	}
  	break;
	}

    case ALCANCE: {
      // Estende o braço com a garra aberta
      irParaPose(POSE_ALCANCE);
      estado = AGARRA;
      break;
    }

    case AGARRA: {
      float d = lerDistanciaCm();
      Serial.print("Dist. para agarrar: ");
      Serial.println(d);
      // Fecha se o objeto está perto o bastante OU se o sensor não
      // enxerga mais (objeto entre as garras, perto demais do sensor)
      if ((d >= 0 && d <= DIST_AGARRAR_CM) || d < 0) {
        moverSuave(sGarra, angGarra, GARRA_FECHADA, 10);
        delay(500);
        estado = RECOLHE;
      }
      break;
    }

    case RECOLHE: {
      irParaPose(POSE_CARGA);
      Serial.println("OBJETO CAPTURADO");
      estado = FIM;
      break;
    }

    case FIM:
      // Fica parado segurando o objeto. Para reiniciar: reset do Arduino
      // (ou troque por um botão que volte para BUSCA com a garra aberta).
      break;
  }
}