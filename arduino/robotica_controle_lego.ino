#include <Arduino.h>

// ======================================================================================
// Pinos
// ======================================================================================
int primeiroMotorPin1 = 2;   // Motor 1 = roda ESQUERDA
int primeiroMotorPin2 = 3;

int segundoMotorPin1 = 4;    // Motor 2 = roda DIREITA
int segundoMotorPin2 = 5;

int motorElevarPin1 = 6;
int motorElevarPin2 = 7;

int primeiroMotorEncoder = 18;
int segundoMotorEncoder = 19;

// ======================================================================================
// Estado geral do robô
// ======================================================================================
int id_tag = -1;             // Tag que estamos procurando. -1 = nenhuma (fica parado)
bool modo_manual = false;

// --- Máquina de estados do modo automático ---
enum EstadoRobo { IDLE, PROCURANDO, APROXIMANDO, ALVO_ALCANCADO };
EstadoRobo estadoAtual = IDLE;

// --- Dados mais recentes vindos da visão (Raspberry) ---
float visao_dist = 0.0;      // distância (m) até a tag alvo  -> vem do "tz"
float visao_ang  = 0.0;      // ângulo (graus) até a tag alvo -> vem do "bearing" (>0 = à direita)
bool  visao_valida = false;  // true = estamos enxergando a tag alvo agora
unsigned long tempoUltimaVisao = 0;              // millis() do último VIS_COMP com o id correto
const unsigned long TIMEOUT_VISAO = 500;         // ms sem ver a tag => consideramos "perdida"

// ======================================================================================
// Parâmetros de controle (AJUSTE ESTES TESTANDO NO CHÃO)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ======================================================================================

// --- Critérios de chegada ---
const float DIST_ALVO = 0.25;   // (m) para de aproximar quando fica mais perto que isso
const float ANG_ALVO  = 8.0;    // (graus) tolerância angular final para considerar "chegou"
const float ANG_MORTO = 3.0;    // (graus) zona morta: abaixo disso não corrige ângulo

// --- Velocidades base (RPM) ---
const float V_APROX     = 90.0;   // avanço na aproximação
const float V_PROCURA   = 55.0;   // avanço lento durante a varredura de busca
const float GIRO_PROCURA = 45.0;  // amplitude do "olhar de um lado pro outro" na busca
const unsigned long PERIODO_SWEEP = 3000; // (ms) período de uma varredura completa

// --- Ganho do controle de ângulo na aproximação ---
const float K_ANG    = 1.5;     // RPM de giro por grau de erro angular
const float GIRO_MAX = 80.0;    // saturação do termo de giro (RPM)

// ======================================================================================
// Variáveis do Encoder e PID
// ======================================================================================
volatile long pulsosMotor1 = 0;
volatile long pulsosMotor2 = 0;

const float FUROS_POR_VOLTA = 20.0;

// Direção lógica comandada de cada roda (+1 frente / -1 ré).
// Como o encoder é de canal único (não mede sentido), usamos o sentido
// que MANDAMOS para "assinar" o RPM medido. Sem isso o PID não gira o robô.
volatile int sentidoM1 = 1;
volatile int sentidoM2 = 1;

// Sinal físico de cada motor. Se a roda girar ao contrário do esperado,
// troque o +1 por -1 (calibração de fiação/montagem).
const int SENTIDO_M1 = 1;
const int SENTIDO_M2 = 1;

float setpoint_RPM_M1 = 0.0;   // alvo de velocidade da roda esquerda (+ = frente)
float setpoint_RPM_M2 = 0.0;   // alvo de velocidade da roda direita  (+ = frente)

unsigned long tempoAnteriorPID = 0;
float erroAcumulado_M1 = 0, erroAnterior_M1 = 0;
float erroAcumulado_M2 = 0, erroAnterior_M2 = 0;

float Kp = 2.0;
float Ki = 5.0;
float Kd = 0.1;
const float LIMITE_INTEGRAL = 150.0;  // anti-windup

// ======================================================================================
// Controle Manual do Robô
// ======================================================================================
void irParaTras() {
  digitalWrite(primeiroMotorPin1, HIGH); digitalWrite(primeiroMotorPin2, LOW);
  digitalWrite(segundoMotorPin1, HIGH);  digitalWrite(segundoMotorPin2, LOW);
}
void irParaFrente() {
  digitalWrite(primeiroMotorPin1, LOW);  digitalWrite(primeiroMotorPin2, HIGH);
  digitalWrite(segundoMotorPin1, LOW);   digitalWrite(segundoMotorPin2, HIGH);
}
void irParaEsquerda() {
  digitalWrite(primeiroMotorPin1, HIGH); digitalWrite(primeiroMotorPin2, LOW);
  digitalWrite(segundoMotorPin1, LOW);   digitalWrite(segundoMotorPin2, HIGH);
}
void irParaDireita() {
  digitalWrite(primeiroMotorPin1, LOW);  digitalWrite(primeiroMotorPin2, HIGH);
  digitalWrite(segundoMotorPin1, HIGH);  digitalWrite(segundoMotorPin2, LOW);
}
void parar() {
  digitalWrite(primeiroMotorPin1, LOW);  digitalWrite(primeiroMotorPin2, LOW);
  digitalWrite(segundoMotorPin1, LOW);   digitalWrite(segundoMotorPin2, LOW);
}
void subir() {
  digitalWrite(motorElevarPin1, HIGH); digitalWrite(motorElevarPin2, LOW);
}
void descer() {
  digitalWrite(motorElevarPin1, LOW);  digitalWrite(motorElevarPin2, HIGH);
}
void pararElevacao() {
  digitalWrite(motorElevarPin1, LOW);  digitalWrite(motorElevarPin2, LOW);
}

// ======================================================================================
// Baixo nível: encoders + PID de velocidade
// ======================================================================================
void lerPrimeiroMotor() { pulsosMotor1++; }
void lerSegundoMotor()  { pulsosMotor2++; }

// Atuador: recebe PWM LÓGICO (+ = frente) e aplica na ponte H.
void controlaMotoresPWM(int pwm1, int pwm2) {
  // --- NOVO: Trava de Segurança (Saturação) ---
  // Garante que o Arduino nunca sofra overflow enviando valores > 255 ou < -255
  pwm1 = constrain(pwm1, -255, 255);
  pwm2 = constrain(pwm2, -255, 255);

  // Guarda o sentido lógico para "assinar" o RPM medido depois.
  sentidoM1 = (pwm1 >= 0) ? 1 : -1;
  sentidoM2 = (pwm2 >= 0) ? 1 : -1;

  // Aplica o sinal físico de cada motor (calibração de montagem).
  int p1 = SENTIDO_M1 * pwm1;
  int p2 = SENTIDO_M2 * pwm2;

  // Motor 1 (Esquerda) - FRENTE é Pin1 LOW e Pin2 PWM
  if (p1 >= 0) { 
    digitalWrite(primeiroMotorPin1, LOW); 
    analogWrite(primeiroMotorPin2, p1); 
  } else { 
    analogWrite(primeiroMotorPin1, abs(p1)); 
    digitalWrite(primeiroMotorPin2, LOW); 
  }

  // Motor 2 (Direita) - FRENTE é Pin1 LOW e Pin2 PWM
  if (p2 >= 0) { 
    digitalWrite(segundoMotorPin1, LOW); 
    analogWrite(segundoMotorPin2, p2); 
  } else { 
    analogWrite(segundoMotorPin1, abs(p2)); 
    digitalWrite(segundoMotorPin2, LOW); 
  }
}

// Roda a cada 100 ms: fecha a malha de velocidade das duas rodas.
void atualizaVelocidadeRodas() {
  unsigned long tempoAtual = millis();
  float dt = (tempoAtual - tempoAnteriorPID) / 1000.0;

  if (dt >= 0.1) {
    // Copia e zera os contadores com as interrupções desligadas (evita corrida).
    noInterrupts();
    long p1 = pulsosMotor1;
    long p2 = pulsosMotor2;
    pulsosMotor1 = 0;
    pulsosMotor2 = 0;
    interrupts();
    tempoAnteriorPID = tempoAtual;

    // RPM ASSINADO: magnitude medida * sentido que mandamos.
    float rpm_M1 = sentidoM1 * (p1 / FUROS_POR_VOLTA) / dt * 60.0;
    float rpm_M2 = sentidoM2 * (p2 / FUROS_POR_VOLTA) / dt * 60.0;

    // PID Motor 1
    float erro_M1 = setpoint_RPM_M1 - rpm_M1;
    erroAcumulado_M1 += erro_M1 * dt;
    erroAcumulado_M1 = constrain(erroAcumulado_M1, -LIMITE_INTEGRAL, LIMITE_INTEGRAL);
    float derivativo_M1 = (erro_M1 - erroAnterior_M1) / dt;
    int pwm_M1 = (Kp * erro_M1) + (Ki * erroAcumulado_M1) + (Kd * derivativo_M1);
    erroAnterior_M1 = erro_M1;

    // PID Motor 2
    float erro_M2 = setpoint_RPM_M2 - rpm_M2;
    erroAcumulado_M2 += erro_M2 * dt;
    erroAcumulado_M2 = constrain(erroAcumulado_M2, -LIMITE_INTEGRAL, LIMITE_INTEGRAL);
    float derivativo_M2 = (erro_M2 - erroAnterior_M2) / dt;
    int pwm_M2 = (Kp * erro_M2) + (Ki * erroAcumulado_M2) + (Kd * derivativo_M2);
    erroAnterior_M2 = erro_M2;

    pwm_M1 = constrain(pwm_M1, -255, 255);
    pwm_M2 = constrain(pwm_M2, -255, 255);
    controlaMotoresPWM(pwm_M1, pwm_M2);
  }
}

void zeraPID() {
  setpoint_RPM_M1 = 0; setpoint_RPM_M2 = 0;
  erroAcumulado_M1 = 0; erroAcumulado_M2 = 0;
  erroAnterior_M1 = 0; erroAnterior_M2 = 0;
}

// ======================================================================================
// Máquina de estados do modo automático
// ======================================================================================

// PROCURANDO: anda devagar pra frente serpenteando ("olhando de um lado pro outro")
// até a câmera enxergar a tag certa (aí chega um VIS_COMP e trocamos de estado).
void executaProcura() {
  float fase = sin(2.0 * PI * (float)(millis() % PERIODO_SWEEP) / (float)PERIODO_SWEEP);
  float giro = GIRO_PROCURA * fase;
  setpoint_RPM_M1 = V_PROCURA + giro;   // esquerda
  setpoint_RPM_M2 = V_PROCURA - giro;   // direita
}

// APROXIMANDO: usa dist/ang do último VIS_COMP para centralizar e chegar perto.
void executaAproximacao() {
  float erro_ang = visao_ang;   // queremos levar a 0

  // Termo de giro proporcional ao erro angular, saturado.
  float giro = K_ANG * erro_ang;
  giro = constrain(giro, -GIRO_MAX, GIRO_MAX);
  if (fabs(erro_ang) < ANG_MORTO) giro = 0;  // não fica "caçando" o zero

  // Avanço: desacelera perto do alvo e reduz quando está muito torto
  // (assim ele gira mais no lugar antes de avançar).
  float avanco = V_APROX;
  float folga = visao_dist - DIST_ALVO;
  if (folga < 0.10) avanco = V_APROX * (folga / 0.10);   // rampa de frenagem
  avanco = constrain(avanco, 0.0, V_APROX);
  float fatorAlinho = 1.0 - min(1.0f, (float)(fabs(erro_ang) / 45.0));
  avanco *= fatorAlinho;

  setpoint_RPM_M1 = avanco + giro;   // esquerda
  setpoint_RPM_M2 = avanco - giro;   // direita

  // Chegou? Perto o suficiente E alinhado o suficiente.
  if (visao_dist <= DIST_ALVO && fabs(visao_ang) <= ANG_ALVO) {
    estadoAtual = ALVO_ALCANCADO;
    zeraPID();
    Serial.println("ESTADO: ALVO_ALCANCADO");
  }
}

void executaMaquinaDeEstados() {
  // Timeout de visão: se faz muito tempo sem VIS_COMP da tag certa, perdemos ela.
  if (millis() - tempoUltimaVisao > TIMEOUT_VISAO) {
    visao_valida = false;
  }

  switch (estadoAtual) {
    case IDLE:
      // Sem alvo: fica parado. Sai daqui quando chega um BUSCAR_TAG.
      setpoint_RPM_M1 = 0; setpoint_RPM_M2 = 0;
      break;

    case PROCURANDO:
      if (visao_valida) {                 // achou a tag certa
        estadoAtual = APROXIMANDO;
        Serial.println("ESTADO: APROXIMANDO");
      } else {
        executaProcura();
      }
      break;

    case APROXIMANDO:
      if (!visao_valida) {                // perdeu a tag -> volta a procurar
        estadoAtual = PROCURANDO;
        Serial.println("ESTADO: PROCURANDO (tag perdida)");
      } else {
        executaAproximacao();
      }
      break;

    case ALVO_ALCANCADO:
      setpoint_RPM_M1 = 0; setpoint_RPM_M2 = 0;
      break;
  }
}

// Recebe "VIS_COMP:ID;DIST;ANG" do Raspberry e atualiza os dados da visão.
void ajustaCaminhoAutomatico(String texto_info) {
  int id;
  float dist, ang;
  int lidos = sscanf(texto_info.c_str(), "VIS_COMP:%d;%f;%f", &id, &dist, &ang);

  if (lidos != 3) {
    Serial.println("Erro: formato da string invalido!");
    return;
  }
  if (id != id_tag) {
    // Tag detectada, mas não é a que estamos procurando: ignora.
    return;
  }

  // É a tag certa: guarda os dados e marca que estamos enxergando.
  visao_dist = dist;
  visao_ang  = ang;
  visao_valida = true;
  tempoUltimaVisao = millis();
}

// Define o alvo (vindo de "BUSCAR_TAG:ID") e entra em modo de busca.
void definirAlvo(String texto) {
  int novo_id;
  if (sscanf(texto.c_str(), "BUSCAR_TAG:%d", &novo_id) == 1) {
    id_tag = novo_id;
    visao_valida = false;
    tempoUltimaVisao = 0;
    zeraPID();
    estadoAtual = PROCURANDO;
    Serial.print("Novo alvo definido: tag ");
    Serial.println(id_tag);
    Serial.println("ESTADO: PROCURANDO");
  } else {
    Serial.println("Erro: BUSCAR_TAG mal formatado!");
  }
}

// ======================================================================================
// Setup e Loop
// ======================================================================================
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("Iniciando programa empilhadeira....");

  pinMode(primeiroMotorPin1, OUTPUT); pinMode(primeiroMotorPin2, OUTPUT);
  pinMode(segundoMotorPin1, OUTPUT);  pinMode(segundoMotorPin2, OUTPUT);
  pinMode(motorElevarPin1, OUTPUT);   pinMode(motorElevarPin2, OUTPUT);
  pinMode(primeiroMotorEncoder, INPUT);
  pinMode(segundoMotorEncoder, INPUT);

  attachInterrupt(digitalPinToInterrupt(primeiroMotorEncoder), lerPrimeiroMotor, RISING);
  attachInterrupt(digitalPinToInterrupt(segundoMotorEncoder), lerSegundoMotor, RISING);

  tempoAnteriorPID = millis();
}

void loop() {
  // 1) Processa comandos que chegaram pela serial
  if (Serial.available() > 0) {
    String texto = Serial.readStringUntil('\n');
    texto.toUpperCase();
    Serial.println("Comando recebido: " + texto);

    if (texto.startsWith("MANUAL")) {
      Serial.println("Modo manual ativado!");
      modo_manual = true;
      estadoAtual = IDLE;
      id_tag = -1; // <--- Reseta a Tag para não bugar ao voltar pro automático
      zeraPID();
      parar();
    }
    else if (texto.startsWith("AUTOMATICO")) {
      Serial.println("Modo automático ativado!");
      modo_manual = false;
      zeraPID();
      id_tag = -1;       // <--- Reseta a Tag antiga
      estadoAtual = IDLE; // <--- Sempre inicia parado em IDLE esperando ordem!
      parar();
    }
    else if (texto.startsWith("BUSCAR_TAG")) {
      // Vale em qualquer modo: registra o alvo. A busca só roda em automático.
      definirAlvo(texto);
    }
    else if (modo_manual) {
      // --- Comandos de pilotagem manual ---
      if      (texto.startsWith("FRENTE"))      { Serial.println("Indo para frente...");  irParaFrente(); }
      else if (texto.startsWith("DIREITA"))     { Serial.println("Indo para direita...");  irParaDireita(); }
      else if (texto.startsWith("ESQUERDA"))    { Serial.println("Indo para esquerda..."); irParaEsquerda(); }
      else if (texto.startsWith("TRAS"))        { Serial.println("Indo para trás...");     irParaTras(); }
      else if (texto.startsWith("PARAR_ANDAR")) { Serial.println("Parando...");            parar(); }
      else if (texto.startsWith("SUBIR"))       { Serial.println("Subindo...");            subir(); }
      else if (texto.startsWith("DESCER"))      { Serial.println("Descendo...");           descer(); }
      else if (texto.startsWith("PARAR_SUBIR")) { Serial.println("Parando garfo...");      pararElevacao(); }
      else                                      { Serial.println("Comando não aceito para modo manual!"); }
    }
    else {
      // --- Modo automático ---
      if (texto.startsWith("VIS_COMP")) {
        ajustaCaminhoAutomatico(texto);
      } else {
        Serial.println("Comando não aceito para modo automático!");
      }
    }
  }

  // 2) No modo automático, a máquina de estados decide os setpoints
  //    e o PID cuida da velocidade das rodas.
  if (!modo_manual) {
    executaMaquinaDeEstados();
    atualizaVelocidadeRodas();
  }
}
