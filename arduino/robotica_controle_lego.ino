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
float visao_ang  = 0.0;      // ângulo (graus) até a tag alvo -> "bearing" (>0 = tag à direita)
bool  visao_valida = false;  // true = estamos enxergando a tag alvo agora
unsigned long tempoUltimaVisao = 0;
const unsigned long TIMEOUT_VISAO = 1500;  // ms sem ver a tag => consideramos "perdida"

// ======================================================================================
// PARÂMETROS DE AJUSTE  (mexa aqui testando no chão)
// ======================================================================================

// --- Velocidade (PWM, 0 a 255). SEM PID: é a força que vai direto pro motor. ---
// Comece baixo. Se o robô não sair do lugar, aumente. Se andar rápido demais, diminua.
const int VEL_PROCURA = 140;   // velocidade durante a busca
const int VEL_APROX   = 150;   // velocidade durante a aproximação

// --- Duração dos "passos curtos" (ms) ---
// MOVER = quanto tempo ele anda em cada passo. PAUSA = quanto tempo ele para
// entre um passo e outro (parado a câmera enxerga melhor, sem borrão).
const unsigned long PROC_MOVER_MS  = 350;   // passo de busca: tempo andando
const unsigned long PROC_PAUSA_MS  = 400;   // passo de busca: tempo parado olhando
const unsigned long APROX_MOVER_MS = 250;   // passo de aproximação: tempo andando
const unsigned long APROX_PAUSA_MS = 350;   // passo de aproximação: tempo parado

// --- Critérios de chegada ---
const float DIST_ALVO = 0.80;   // (m) considera "perto o suficiente" abaixo disso
const float ANG_ALVO  = 8.0;    // (graus) considera "centralizado" abaixo disso

// --- Sequência de busca (edite à vontade!) ---
// Códigos: 0 = frente, 1 = vira direita, 2 = vira esquerda
// O padrão {0,1,2,0} faz: frente -> direita -> esquerda -> frente -> (repete)
const int SEQ_BUSCA[] = {0, 1, 1, 2, 2, 0};
const int NUM_PASSOS = sizeof(SEQ_BUSCA) / sizeof(SEQ_BUSCA[0]);

// ======================================================================================
// Controle de tempo dos passos (não bloqueante - nada de delay())
// ======================================================================================
unsigned long tempoPasso = 0;   // millis() em que o passo/pausa atual começou
bool emMovimento = false;       // true = andando neste passo | false = pausa
int  passoBuscaIdx = 0;         // qual passo da SEQ_BUSCA estamos

// ======================================================================================
// Controle Manual do Robô  (IGUALZINHO ao original - não mexi em nada aqui)
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
// Movimento AUTOMÁTICO devagar (PWM fixo, SEM PID)
// Usa a MESMA convenção de sentido do modo manual, só que com velocidade reduzida.
// "Frente" de cada roda = Pin1 LOW + PWM no Pin2 (igual ao irParaFrente).
// ======================================================================================
void m1Frente(int v) { digitalWrite(primeiroMotorPin1, LOW); analogWrite(primeiroMotorPin2, v); }
void m1Tras(int v)   { analogWrite(primeiroMotorPin1, v);    digitalWrite(primeiroMotorPin2, LOW); }
void m2Frente(int v) { digitalWrite(segundoMotorPin1, LOW);  analogWrite(segundoMotorPin2, v); }
void m2Tras(int v)   { analogWrite(segundoMotorPin1, v);     digitalWrite(segundoMotorPin2, LOW); }

void andarFrente(int v)   { m1Frente(v); m2Frente(v); }   // as duas pra frente
void girarDireita(int v)  { m1Frente(v); m2Tras(v);   }   // esquerda frente, direita ré (= irParaDireita)
void girarEsquerda(int v) { m1Tras(v);   m2Frente(v); }   // esquerda ré, direita frente (= irParaEsquerda)

// Executa uma ação da sequência de busca
void executaAcaoBusca(int codigo) {
  switch (codigo) {
    case 0: andarFrente(VEL_PROCURA);   Serial.println("Busca: frente");   break;
    case 1: girarDireita(VEL_PROCURA);  Serial.println("Busca: direita");  break;
    case 2: girarEsquerda(VEL_PROCURA); Serial.println("Busca: esquerda"); break;
    default: parar(); break;
  }
}

// ======================================================================================
// Transições entre estados (sempre param os motores ao trocar)
// ======================================================================================
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

void entrarAlvoAlcancado() {
  parar();
  estadoAtual = ALVO_ALCANCADO;
  Serial.println("ESTADO: ALVO_ALCANCADO");
}

// ======================================================================================
// PROCURANDO: passos curtos em loop (frente -> direita -> esquerda -> frente -> ...)
// A cada passo ele anda um pouco, PARA, e nessa pausa a câmera consegue enxergar.
// Se a tag certa aparecer (visao_valida vira true), a máquina troca para APROXIMANDO.
// ======================================================================================
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

// ======================================================================================
// APROXIMANDO: sem PID, controle simples por passos.
// Se estiver torto -> gira pro lado da tag. Se estiver centralizado -> anda pra frente.
// Quando fica perto E centralizado -> para de vez (ALVO_ALCANCADO).
// ======================================================================================
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

// ======================================================================================
// Máquina de estados (chamada continuamente no modo automático)
// ======================================================================================
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

// ======================================================================================
// Recebe "VIS_COMP:ID;DIST;ANG" do Raspberry e atualiza os dados da visão.
// Faço o parsing na mão (indexOf/substring) porque o sscanf com %f do Arduino AVR
// costuma NÃO ler número decimal e devolver zero. toFloat()/toInt() são confiáveis.
// ======================================================================================
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

// ======================================================================================
// Setup e Loop
// ======================================================================================
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("Iniciando programa empilhadeira....");

  pinMode(primeiroMotorPin1, OUTPUT); pinMode(primeiroMotorPin2, OUTPUT);
  pinMode(segundoMotorPin1, OUTPUT);  pinMode(segundoMotorPin2, OUTPUT);
  pinMode(motorElevarPin1, OUTPUT);   pinMode(motorElevarPin2, OUTPUT);

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

  // 2) No modo automático, a máquina de estados cuida de tudo.
  if (!modo_manual) {
    executaMaquinaDeEstados();
  }
}
