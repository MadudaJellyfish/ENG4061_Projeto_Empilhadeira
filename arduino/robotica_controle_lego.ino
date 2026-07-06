#include <Arduino.h>

int primeiroMotorPin1 = 2;
int primeiroMotorPin2 = 3;

int segundoMotorPin1 = 4;
int segundoMotorPin2 = 5;

int motorElevarPin1 = 6;
int motorElevarPin2 = 7;

int primeiroMotorEncoder = 18;
int segundoMotorEncoder = 19;

int id_tag = -1;
bool modo_manual = false;

// ======================================================================================
// Variáveis do Encoder e PID (NOVO)
// ======================================================================================

volatile long pulsosMotor1 = 0;
volatile long pulsosMotor2 = 0;

// Quantidade de furos do seu disco de encoder (ajuste se o seu for diferente)
const float FUROS_POR_VOLTA = 20.0; 

// Alvos de velocidade (RPM) que o nível alto vai ditar
float setpoint_RPM_M1 = 0.0;
float setpoint_RPM_M2 = 0.0;

// Variáveis de tempo e estado do PID
unsigned long tempoAnteriorPID = 0;
float erroAcumulado_M1 = 0, erroAnterior_M1 = 0;
float erroAcumulado_M2 = 0, erroAnterior_M2 = 0;

// Ganhos do PID (Ajuste testando no chão: Kp dá a força, Ki corrige o erro final)
float Kp = 2.0;
float Ki = 5.0;
float Kd = 0.1;

// ======================================================================================
// Controle Manual Robô 
// ======================================================================================

void irParaTras()
{
  // Motor 1 sentido direto
  digitalWrite(primeiroMotorPin1, HIGH);
  digitalWrite(primeiroMotorPin2, LOW);

  // Motor 2 sentido direto
  digitalWrite(segundoMotorPin1, HIGH);
  digitalWrite(segundoMotorPin2, LOW);
}

void irParaFrente()
{
  // Motor 1 sentido inverso
  digitalWrite(primeiroMotorPin1, LOW);
  digitalWrite(primeiroMotorPin2, HIGH);

  // Motor 2 sentido inverso
  digitalWrite(segundoMotorPin1, LOW);
  digitalWrite(segundoMotorPin2, HIGH);
}

void irParaEsquerda()
{
  // Motor 1 sentido direto
  digitalWrite(primeiroMotorPin1, HIGH);
  digitalWrite(primeiroMotorPin2, LOW);

  // Motor 2 sentido inverso
  digitalWrite(segundoMotorPin1, LOW);
  digitalWrite(segundoMotorPin2, HIGH);
}

void irParaDireita()
{
  // Motor 1 sentido inverso
  digitalWrite(primeiroMotorPin1, LOW);
  digitalWrite(primeiroMotorPin2, HIGH);

  // Motor 2 sentido direto
  digitalWrite(segundoMotorPin1, HIGH);
  digitalWrite(segundoMotorPin2, LOW);
}

void parar()
{
  digitalWrite(primeiroMotorPin1, LOW);
  digitalWrite(primeiroMotorPin2, LOW);

  digitalWrite(segundoMotorPin1, LOW);
  digitalWrite(segundoMotorPin2, LOW);
}

void subir()
{
  digitalWrite(motorElevarPin1, HIGH);
  digitalWrite(motorElevarPin2, LOW);
}

void descer()
{
  digitalWrite(motorElevarPin1, LOW);
  digitalWrite(motorElevarPin2, HIGH);
}

void pararElevacao()
{
  digitalWrite(motorElevarPin1, LOW);
  digitalWrite(motorElevarPin2, LOW);
}

// ======================================================================================
// Controle Automático Robô e PID
// ======================================================================================

// Interrupções dos Encoders
void lerPrimeiroMotor() {
  pulsosMotor1++;
}

void lerSegundoMotor() {
  pulsosMotor2++;
}

// Atuador: Traduz o resultado do PID em sinais elétricos (PWM)
void controlaMotoresPWM(int pwm1, int pwm2) {
  // Motor 1 (Esquerda)
  if (pwm1 >= 0) 
  {
    analogWrite(primeiroMotorPin1, pwm1);
    digitalWrite(primeiroMotorPin2, LOW);
  } 
  else 
  {
    digitalWrite(primeiroMotorPin1, LOW);
    analogWrite(primeiroMotorPin2, abs(pwm1));
  }
  
  // Motor 2 (Direita)
  if (pwm2 >= 0) 
  {
    analogWrite(segundoMotorPin1, pwm2);
    digitalWrite(segundoMotorPin2, LOW);
  } 
  else 
  {
    digitalWrite(segundoMotorPin1, LOW);
    analogWrite(segundoMotorPin2, abs(pwm2));
  }
}

// O Coração do Baixo Nível: Executado constantemente no modo automático
void atualizaVelocidadeRodas() 
{
  unsigned long tempoAtual = millis();
  float dt = (tempoAtual - tempoAnteriorPID) / 1000.0; // Variação de tempo em segundos

  if (dt >= 0.1) // Roda o controle a cada 100 milissegundos
  { 
    
    // 1. Calcula o RPM atual de cada roda
    float rpm_M1 = (pulsosMotor1 / FUROS_POR_VOLTA) / dt * 60.0;
    float rpm_M2 = (pulsosMotor2 / FUROS_POR_VOLTA) / dt * 60.0;

    // 2. Zera os contadores para o próximo ciclo
    pulsosMotor1 = 0;
    pulsosMotor2 = 0;
    tempoAnteriorPID = tempoAtual;

    // 3. Cálculos do PID Motor 1
    float erro_M1 = setpoint_RPM_M1 - rpm_M1;
    erroAcumulado_M1 += erro_M1 * dt;
    float derivativo_M1 = (erro_M1 - erroAnterior_M1) / dt;
    int pwm_M1 = (Kp * erro_M1) + (Ki * erroAcumulado_M1) + (Kd * derivativo_M1);
    erroAnterior_M1 = erro_M1;

    // 4. Cálculos do PID Motor 2
    float erro_M2 = setpoint_RPM_M2 - rpm_M2;
    erroAcumulado_M2 += erro_M2 * dt;
    float derivativo_M2 = (erro_M2 - erroAnterior_M2) / dt;
    int pwm_M2 = (Kp * erro_M2) + (Ki * erroAcumulado_M2) + (Kd * derivativo_M2);
    erroAnterior_M2 = erro_M2;

    // 5. Limita o PWM aos limites do Arduino e envia aos motores
    pwm_M1 = constrain(pwm_M1, -255, 255);
    pwm_M2 = constrain(pwm_M2, -255, 255);

    controlaMotoresPWM(pwm_M1, pwm_M2);
  }
}

void ajustaCaminhoAutomatico(String texto_info)
{
  int id;
  float dist, ang;
  
  int lidos = sscanf(texto_info.c_str(), "VIS_COMP:%d;%f;%f", &id, &dist, &ang);
  
  if (lidos == 3) 
  {
    // Exemplo de como usar o PID aqui: 
    // Em vez de acionar motores direto, você define a velocidade alvo desejada (RPM)
    setpoint_RPM_M1 = 120.0; 
    setpoint_RPM_M2 = 120.0;
  } else {
    Serial.println("Erro: formato da string invalido!");
    return;
  }

  if(id != id_tag) {
    Serial.println("Tag errada lida!");
    return;
  }
}

// ======================================================================================
// Setup e Loop
// ======================================================================================

void setup() 
{
  Serial.begin(115200); delay(500);
  Serial.println("Iniciando programa empilhadeira....");

   // Para o controle da ponte H para os motores
  // Motor 1
  pinMode(primeiroMotorPin1, OUTPUT);
  pinMode(primeiroMotorPin2, OUTPUT);
  
  // Motor 2
  pinMode(segundoMotorPin1, OUTPUT);
  pinMode(segundoMotorPin2, OUTPUT);

  // Motor Subida
  pinMode(motorElevarPin1, OUTPUT);
  pinMode(motorElevarPin2, OUTPUT);

  // Encoder das rodas do Motor 1
  pinMode(primeiroMotorEncoder, INPUT);

  // Encoder das rodas do Motor 2
  pinMode(segundoMotorEncoder, INPUT);

  attachInterrupt(digitalPinToInterrupt(primeiroMotorEncoder), lerPrimeiroMotor, RISING);  
  attachInterrupt(digitalPinToInterrupt(segundoMotorEncoder), lerSegundoMotor, RISING);
}

void loop() 
{
  // O controle PID roda sozinho mantendo o robô no trajeto
  if (!modo_manual) 
  {
    atualizaVelocidadeRodas(); 
  }
  
  if (Serial.available() > 0) 
  {
    String texto = Serial.readStringUntil('\n');
    texto.toUpperCase();
    Serial.println("Comando recebido: " + texto);

    if (texto.startsWith("MANUAL"))
    {
      Serial.println("Modo manual ativado!");
      modo_manual = true;
      // Zera variáveis do PID para não acumular sujeira
      setpoint_RPM_M1 = 0; setpoint_RPM_M2 = 0;
      erroAcumulado_M1 = 0; erroAcumulado_M2 = 0;
      parar();
    }
    else if (texto.startsWith("AUTOMATICO"))
    {
      Serial.println("Modo automático ativado!");
      modo_manual = false;
    }

    if (modo_manual)
    {
      if (texto.startsWith("FRENTE")){
        Serial.println("Indo para frente...");
        irParaFrente();
      }
      else if (texto.startsWith("DIREITA")){
        Serial.println("Indo para direita...");
        irParaDireita();
      }
      else if (texto.startsWith("ESQUERDA")){
        Serial.println("Indo para esquerda...");
        irParaEsquerda();
      }
      else if (texto.startsWith("TRAS")){
        Serial.println("Indo para trás...");
        irParaTras();
      }
      else if (texto.startsWith("PARAR_ANDAR")){
        Serial.println("Parando...");
        parar();
      }
      else if (texto.startsWith("SUBIR")){
        Serial.println("Subindo...");
        subir();
      }
      else if (texto.startsWith("DESCER")){
        Serial.println("Descendo...");
        descer();
      }
      else if (texto.startsWith("PARAR_SUBIR")){
        Serial.println("Parando garfo...");
        pararElevacao();
      }
      else{
        Serial.println("Comando não aceito para modo manual!");
      }
    }
    else
    {
      if (texto.startsWith("VIS_COMP"))
      {
        ajustaCaminhoAutomatico(texto);
      }
      else if (texto != "MANUAL" && texto != "AUTOMATICO")
      {
        Serial.println("Comando não aceito para modo automático!");
      }
    }
  }
}
