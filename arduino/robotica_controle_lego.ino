/*
================================================================================
   PROJETO DE ROBÓTICA: EMPILHADEIRA AUTÔNOMA COM DETECÇÃO DE TAGS
================================================================================

OBJETIVO:
  Controlar uma empilhadeira Lego que se move de forma autônoma buscando e 
  aproximando-se de tags AprilTags detectadas pela câmera (Raspberry Pi).

MODOS DE OPERAÇÃO:
  1. MANUAL: Controlada via MQTT pelo servidor (joystick)
  2. AUTOMÁTICO: Máquina de estados que busca tags via visão computacional

COMPONENTES PRINCIPAIS:
  - Arduino (este arquivo): Controle dos motores e estados
  - Raspberry Pi (main.py): Detecção de tags + visão computacional
  - Servidor (servidor.py): Interface MQTT + GUI para usuário

FLUXO DE COMUNICAÇÃO:
  Arduino <--(Serial)--> Raspberry Pi <--(MQTT)--> Servidor

MÁQUINA DE ESTADOS AUTOMÁTICA:
  IDLE -> PROCURANDO -> APROXIMANDO -> ALVO_ALCANCADO

================================================================================
*/

#include <Arduino.h>

// ============================================================================
// CONFIGURAÇÃO DE PINOS (Referência do Hardware)
// ============================================================================
int primeiroMotorPin1 = 2;   // Motor 1 = roda ESQUERDA
int primeiroMotorPin2 = 3;

int segundoMotorPin1 = 4;    // Motor 2 = roda DIREITA
int segundoMotorPin2 = 5;

int motorElevarPin1 = 6;
int motorElevarPin2 = 7;

int primeiroMotorEncoder = 18;
int segundoMotorEncoder = 19;

// ============================================================================
// VARIÁVEIS GLOBAIS: Estado do Robô
// ============================================================================
int id_tag = -1;             // Tag que estamos procurando. -1 = nenhuma (fica parado)
bool modo_manual = false;    // true = controle manual via MQTT | false = automático

// --- Máquina de estados do modo automático ---
enum EstadoRobo { IDLE, PROCURANDO, APROXIMANDO, ALVO_ALCANCADO };
EstadoRobo estadoAtual = IDLE;

// --- Dados mais recentes vindos da visão (Raspberry) ---
float visao_dist = 0.0;      // distância (m) até a tag alvo  -> vem do "tz"
float visao_ang  = 0.0;      // ângulo (graus) até a tag alvo -> "bearing" (>0 = tag à direita)
bool  visao_valida = false;  // true = estamos enxergando a tag alvo agora
unsigned long tempoUltimaVisao = 0;
const unsigned long TIMEOUT_VISAO = 1500;  // ms sem ver a tag => consideramos "perdida"

// ============================================================================
// PARÂMETROS DE AJUSTE - TUNING DO CONTROLE
// ============================================================================
// NOTA: Ajuste estes parâmetros observando o comportamento do robô em testes.
// Se der tranco ou instabilidade, diminua os ganhos (Kp, velocidades).

// --- Alvo de VELOCIDADE (em RPM) ---
// O controle P é quem transforma isso em PWM usando os encoders.
// Ajuste observando: se a roda nem gira, aumente; se satura sempre no máximo, diminua.
const float VEL_PROCURA = 60.0;   // RPM alvo durante a busca
const float VEL_APROX   = 80.0;   // RPM alvo durante a aproximação

// --- Controle de velocidade: SOMENTE PROPORCIONAL (P) ---
const float Kp = 3.0;              // ganho proporcional. Maior = responde mais forte.
                                   // Se der tranco/oscilar, diminua. Se ficar mole, aumente.
const int   PWM_MAX = 200;         // teto de PWM (0..255). Segurança contra pico de corrente.
                                   // Se ainda travar/dar pico, BAIXE este valor.
const unsigned long INTERVALO_CONTROLE = 50;  // ms entre cada cálculo do P (janela de medição)
const float PULSOS_POR_VOLTA = 20.0;          // CALIBRAR: pulsos do encoder em 1 volta da roda

// --- Duração dos "passos curtos" (ms) ---
const unsigned long PROC_MOVER_MS  = 350;   // passo de busca: tempo andando
const unsigned long PROC_PAUSA_MS  = 400;   // passo de busca: tempo parado olhando
const unsigned long APROX_MOVER_MS = 250;   // passo de aproximação: tempo andando
const unsigned long APROX_PAUSA_MS = 350;   // passo de aproximação: tempo parado

// --- Critérios de chegada ---
const float DIST_ALVO = 0.60;   // (m) considera "perto o suficiente" abaixo disso
const float ANG_ALVO  = 8.0;    // (graus) considera "centralizado" abaixo disso

// --- Sequência de busca ---
// Códigos: 0 = frente, 1 = vira direita, 2 = vira esquerda
const int SEQ_BUSCA[] = {0, 1, 1, 2, 2, 2, 0};
const int NUM_PASSOS = sizeof(SEQ_BUSCA) / sizeof(SEQ_BUSCA[0]);

// Controle de tempo dos passos (não bloqueante - nada de delay())
unsigned long tempoPasso = 0;   // millis() em que o passo/pausa atual começou
bool emMovimento = false;       // true = andando neste passo | false = pausa
int  passoBuscaIdx = 0;         // qual passo da SEQ_BUSCA estamos

// ENCODERS + setpoints de velocidade
volatile long pulsos1 = 0;   // contagem de pulsos do encoder da roda ESQUERDA
volatile long pulsos2 = 0;   // contagem de pulsos do encoder da roda DIREITA

// Velocidade alvo de cada roda, COM SINAL (+ = frente, - = ré).
// As funções de movimento do modo automático só MEXEM nestes valores;
// quem realmente aciona os motores é o controlaVelocidadeP().
float setpointM1 = 0.0;
float setpointM2 = 0.0;

unsigned long tempoControleAnterior = 0;

// Rotinas de interrupção: cada pulso do encoder incrementa o contador.
void lerPrimeiroMotor() { pulsos1++; }
void lerSegundoMotor()  { pulsos2++; }

// ============================================================================
// FUNÇÕES DE MOVIMENTO: Controle Manual
// ============================================================================
// Estas funções controlam os motores diretamente nos pinos (modo manual)
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
void subir() {
  digitalWrite(motorElevarPin1, HIGH); digitalWrite(motorElevarPin2, LOW);
}
void descer() {
  digitalWrite(motorElevarPin1, LOW);  digitalWrite(motorElevarPin2, HIGH);
}
void pararElevacao() {
  digitalWrite(motorElevarPin1, LOW);  digitalWrite(motorElevarPin2, LOW);
}

// PARAR: zera os setpoints (modo automático coasta até parar) E corta os pinos
// na hora (garante parada imediata no modo manual, que não roda o controle P).
void parar() {
  setpointM1 = 0.0;
  setpointM2 = 0.0;
  digitalWrite(primeiroMotorPin1, LOW);  digitalWrite(primeiroMotorPin2, LOW);
  digitalWrite(segundoMotorPin1, LOW);   digitalWrite(segundoMotorPin2, LOW);
}

// Aplica um PWM COM SINAL em cada motor (+ = frente, - = ré),
// seguindo a mesma convenção do modo manual (frente = Pin1 LOW + PWM no Pin2).
void aplicaPWM_M1(int pwm) {
  if (pwm >= 0) { digitalWrite(primeiroMotorPin1, LOW); analogWrite(primeiroMotorPin2, pwm); }
  else          { analogWrite(primeiroMotorPin1, -pwm); digitalWrite(primeiroMotorPin2, LOW); }
}
void aplicaPWM_M2(int pwm) {
  if (pwm >= 0) { digitalWrite(segundoMotorPin1, LOW); analogWrite(segundoMotorPin2, pwm); }
  else          { analogWrite(segundoMotorPin1, -pwm); digitalWrite(segundoMotorPin2, LOW); }
}

// CONTROLE DE VELOCIDADE - SOMENTE P (proporcional)
// Mede a RPM de cada roda pelos encoders e ajusta o PWM proporcional ao erro.
//   PWM = Kp * (RPM_alvo - RPM_medido)
// Como o encoder é de 1 canal (não mede sentido), "assinamos" a RPM medida com o
// sinal do setpoint (a direção que mandamos girar).
void controlaVelocidadeP() {
  unsigned long agora = millis();
  if (agora - tempoControleAnterior < INTERVALO_CONTROLE) return;

  float dt = (agora - tempoControleAnterior) / 1000.0;
  tempoControleAnterior = agora;

  // Lê e zera os contadores com as interrupções desligadas (evita corrida de dados).
  noInterrupts();
  long p1 = pulsos1; long p2 = pulsos2;
  pulsos1 = 0; pulsos2 = 0;
  interrupts();

  // RPM em módulo
  float rpm1 = (p1 / PULSOS_POR_VOLTA) / dt * 60.0;
  float rpm2 = (p2 / PULSOS_POR_VOLTA) / dt * 60.0;

  // Assina a RPM medida pelo sentido comandado
  float rpm1_assinada = (setpointM1 >= 0 ? 1.0 : -1.0) * rpm1;
  float rpm2_assinada = (setpointM2 >= 0 ? 1.0 : -1.0) * rpm2;

  int pwm1, pwm2;

  // Setpoint zero -> desliga o motor (para por inércia, sem frear ao contrário).
  if (setpointM1 == 0.0) {
    pwm1 = 0;
  } else {
    float erro1 = setpointM1 - rpm1_assinada;   // somente o termo P
    pwm1 = (int)(Kp * erro1);
  }

  if (setpointM2 == 0.0) {
    pwm2 = 0;
  } else {
    float erro2 = setpointM2 - rpm2_assinada;   // somente o termo P
    pwm2 = (int)(Kp * erro2);
  }

  // Teto de segurança (evita o pico de corrente).
  pwm1 = constrain(pwm1, -PWM_MAX, PWM_MAX);
  pwm2 = constrain(pwm2, -PWM_MAX, PWM_MAX);

  aplicaPWM_M1(pwm1);
  aplicaPWM_M2(pwm2);
}

// ============================================================================
// FUNÇÕES DE MOVIMENTO: Controle Automático
// ============================================================================
// Estas funções definem os SETPOINTS de velocidade (RPM).
// O controlador P (controlaVelocidadeP) lê o encoder e ajusta o PWM para
// alcançar o setpoint. Funciona de forma não-bloqueante.

void andarFrente(float v)   { setpointM1 =  v; setpointM2 =  v; }   // ambas as rodas para frente
void girarDireita(float v)  { setpointM1 =  v; setpointM2 = -v; }   // esq. frente, dir. ré
void girarEsquerda(float v) { setpointM1 = -v; setpointM2 =  v; }   // esq. ré, dir. frente

// ============================================================================
// MÁQUINA DE ESTADOS - MODO PROCURANDO
// ============================================================================
// Executa uma ação da sequência de busca pré-definida (SEQ_BUSCA)
void executaAcaoBusca(int codigo) {
  switch (codigo) {
    case 0: andarFrente(VEL_PROCURA);   Serial.println("Busca: frente");   break;
    case 1: girarDireita(VEL_PROCURA);  Serial.println("Busca: direita");  break;
    case 2: girarEsquerda(VEL_PROCURA); Serial.println("Busca: esquerda"); break;
    default: parar(); break;
  }
}

// Transições entre estados (sempre param os motores ao trocar)
void entrarProcura() {
  parar();
  estadoAtual = PROCURANDO;
  passoBuscaIdx = 0;
  emMovimento = false;
  tempoPasso = millis();
  Serial.println("ESTADO: PROCURANDO");
}

void entrarAproximacao() {
  parar();
  estadoAtual = APROXIMANDO;
  emMovimento = false;
  tempoPasso = millis();
  Serial.println("ESTADO: APROXIMANDO");
}

void informarTagEncontrada(int tagId) {
  Serial.print("TAG_ENCONTRADA:");
  Serial.println(tagId);
}

void informarTagPerdida(int tagId) {
  Serial.print("TAG_PERDIDA:");
  Serial.println(tagId);
}

void entrarAlvoAlcancado() {
  parar();
  estadoAtual = ALVO_ALCANCADO;
  Serial.println("ESTADO: ALVO_ALCANCADO");
  informarTagEncontrada(id_tag);
}

// PROCURANDO: passos curtos em loop (frente -> direita -> esquerda -> frente -> ...)
// A cada passo ele anda um pouco, PARA, e nessa pausa a câmera consegue enxergar.
// Se a tag certa aparecer (visao_valida vira true), a máquina troca para APROXIMANDO.
void executaProcura() {
  unsigned long agora = millis();
  unsigned long dur = emMovimento ? PROC_MOVER_MS : PROC_PAUSA_MS;

  if (agora - tempoPasso >= dur) {
    emMovimento = !emMovimento;   // alterna: movendo <-> pausado
    tempoPasso = agora;

    if (emMovimento) {
      executaAcaoBusca(SEQ_BUSCA[passoBuscaIdx]);   // começa o passo atual
    } else {
      parar();                                       // pausa para "olhar"
      passoBuscaIdx = (passoBuscaIdx + 1) % NUM_PASSOS;  // prepara o próximo passo
    }
  }
}

// APROXIMANDO: controle simples por passos.
// Se estiver torto -> gira pro lado da tag. Se estiver centralizado -> anda pra frente.
// Quando fica perto E centralizado -> para de vez (ALVO_ALCANCADO).
void executaAproximacao() {
  // Chegou? Perto o suficiente E alinhado o suficiente -> trava aqui.
  if (visao_dist <= DIST_ALVO && fabs(visao_ang) <= ANG_ALVO) {
    entrarAlvoAlcancado();
    return;
  }

  unsigned long agora = millis();
  unsigned long dur = emMovimento ? APROX_MOVER_MS : APROX_PAUSA_MS;

  if (agora - tempoPasso >= dur) {
    emMovimento = !emMovimento;
    tempoPasso = agora;

    if (emMovimento) {
      if (fabs(visao_ang) > ANG_ALVO) {
        // Ainda desalinhado: gira no lugar para centralizar a tag.
        if (visao_ang > 0) girarDireita(VEL_APROX);    // tag à direita -> vira à direita
        else               girarEsquerda(VEL_APROX);   // tag à esquerda -> vira à esquerda
      } else {
        // Centralizado mas ainda longe: avança um passo.
        andarFrente(VEL_APROX);
      }
    } else {
      parar();   // pausa para receber um VIS_COMP fresco antes do próximo passo
    }
  }
}

// Máquina de estados (chamada continuamente no modo automático)
void executaMaquinaDeEstados() {
  // Perdeu a tag de vista? (faz tempo demais sem receber VIS_COMP dela)
  if (millis() - tempoUltimaVisao > TIMEOUT_VISAO) {
    visao_valida = false;
  }

  switch (estadoAtual) {
    case IDLE:
      parar();                                  // sem alvo: fica parado
      break;

    case PROCURANDO:
      if (visao_valida) entrarAproximacao();    // achou a tag certa
      else              executaProcura();       // continua procurando em passos
      break;

    case APROXIMANDO:
      if (!visao_valida) {                       // perdeu a tag -> volta a procurar
        Serial.println("Tag perdida, voltando a procurar...");
        informarTagPerdida(id_tag);
        entrarProcura();
      } else {
        executaAproximacao();
      }
      break;

    case ALVO_ALCANCADO:
      parar();                                   // chegou: não se move mais
      break;
  }
}

// Recebe "VIS_COMP:ID;DIST;ANG" do Raspberry e atualiza os dados da visão.
// Faço o parsing na mão (indexOf/substring) porque o sscanf com %f do Arduino AVR
// costuma NÃO ler número decimal e devolver zero. toFloat()/toInt() são confiáveis.
void ajustaCaminhoAutomatico(String texto_info) {
  int p1 = texto_info.indexOf(':');
  int p2 = texto_info.indexOf(';', p1 + 1);
  int p3 = texto_info.indexOf(';', p2 + 1);

  if (p1 < 0 || p2 < 0 || p3 < 0) {
    Serial.println("Erro: formato do VIS_COMP invalido!");
    return;
  }

  int   id   = texto_info.substring(p1 + 1, p2).toInt();
  float dist = texto_info.substring(p2 + 1, p3).toFloat();
  float ang  = texto_info.substring(p3 + 1).toFloat();

  if (id != id_tag) return;   // tag detectada, mas não é a que procuramos

  visao_dist = dist;
  visao_ang  = ang;
  visao_valida = true;
  tempoUltimaVisao = millis();
}

// Define o alvo (vindo de "BUSCAR_TAG:ID") e começa a busca.
void definirAlvo(String texto) {
  int p = texto.indexOf(':');
  if (p < 0) {
    Serial.println("Erro: BUSCAR_TAG mal formatado!");
    return;
  }
  id_tag = texto.substring(p + 1).toInt();
  visao_valida = false;
  tempoUltimaVisao = 0;
  Serial.print("Novo alvo definido: tag ");
  Serial.println(id_tag);
  entrarProcura();
}

// Setup e Loop
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

  tempoControleAnterior = millis();
  parar();
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
      id_tag = -1;
      parar();
    }
    else if (texto.startsWith("AUTOMATICO")) {
      Serial.println("Modo automático ativado!");
      modo_manual = false;
      id_tag = -1;           // começa sem alvo
      estadoAtual = IDLE;    // só anda depois que receber um BUSCAR_TAG
      parar();
    }
    else if (texto.startsWith("BUSCAR_TAG")) {
      definirAlvo(texto);    // registra o alvo (a busca só roda no modo automático)
    }
    else if (modo_manual) {
      // --- Comandos de pilotagem manual (inalterados) ---
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
      // --- Modo automático: só aceita dados da visão ---
      if (texto.startsWith("VIS_COMP")) {
        ajustaCaminhoAutomatico(texto);
      } else {
        Serial.println("Comando não aceito para modo automático!");
      }
    }
  }

  // 2) No modo automático: máquina de estados define os alvos de velocidade
  //    e o controle P (encoders) aciona os motores.
  if (!modo_manual) {
    executaMaquinaDeEstados();
    controlaVelocidadeP();
  }
}
