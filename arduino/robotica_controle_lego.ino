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

// Controle Manual Robô =====================================================================

void irParaDireita()
{
  // Motor 1 sentido direto
  digitalWrite(primeiroMotorPin1, HIGH);
  digitalWrite(primeiroMotorPin2, LOW);

  // Motor 2 sentido direto
  digitalWrite(segundoMotorPin1, HIGH);
  digitalWrite(segundoMotorPin2, LOW);
}

void irParaEsquerda()
{
  // Motor 1 sentido inverso
  digitalWrite(primeiroMotorPin1, LOW);
  digitalWrite(primeiroMotorPin2, HIGH);

  // Motor 2 sentido inverso
  digitalWrite(segundoMotorPin1, LOW);
  digitalWrite(segundoMotorPin2, HIGH);
}

void irParaFrente()
{
  // Motor 1 sentido direto
  digitalWrite(primeiroMotorPin1, HIGH);
  digitalWrite(primeiroMotorPin2, LOW);

  // Motor 2 sentido inverso
  digitalWrite(segundoMotorPin1, LOW);
  digitalWrite(segundoMotorPin2, HIGH);
}

void irParaTras()
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
  digitalWrite(motorElevarPin1, LOW);
  digitalWrite(motorElevarPin2, HIGH);
}

void descer()
{
  digitalWrite(motorElevarPin1, HIGH);
  digitalWrite(motorElevarPin2, LOW);
}

void pararElevacao()
{
  digitalWrite(motorElevarPin1, LOW);
  digitalWrite(motorElevarPin2, LOW);
}

// ======================================================================================

// Controle Automático Robô =============================================================

void lerPrimeiroMotor() 
{
  // FAZER
}

void lerSegundoMotor() 
{
  // FAZER
}

void ajustaCaminhoAutomatico(String texto_info)
{
  int id;
  float dist, ang;
  
  int lidos = sscanf(texto_info.c_str(), "VIS_COMP:%d;%f;%f", &id, &dist, &ang);
  
  if (lidos == 3) 
  {
    Serial.print("ID: "); Serial.println(id);
    Serial.print("DIST: "); Serial.println(dist);
    Serial.print("ANG: "); Serial.println(ang);
  } 
  else 
  {
    Serial.println("Erro: formato da string invalido!");
    return;
  }

  if(id != id_tag)
  {
    Serial.println("Tag errada lida!");
    return;
  }
  else 
  {
    Serial.println("Tag correta lida!");
  }
  // FAZER
}

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
  if (!modo_manual) {
    atualizaVelocidadeRodas(); 
  }
  
  if (Serial.available() > 0) {
    String texto = Serial.readStringUntil('\n');
    texto.toUpperCase();
    Serial.println("Comando recebido: " + texto);

    if (texto.startsWith("MANUAL")){
      Serial.println("Modo manual ativado!");
      modo_manual = true;
    }
    else if (texto.startsWith("AUTOMATICO")){
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
      if (texto.startsWith("BUSCAR_TAG")){ // tag que tem que ser chegada
        Serial.println("Buscar Tag Id: ");
        
        int lidos = sscanf(texto_info.c_str(), "BUSCAR_TAG:%d", &id_tag);

        if(lidos == 1)
        {
          Serial.print("ID para Buscar: "); Serial.println(id_tag);
        }
        else
        {
          Serial.println("Ocorreu um problema no recebimento da Tag a ser buscada!");
        }
      }
      else if (texto.startsWith("VIS_COMP")){
        // "VIS_COMP:ID;DIST;ANG"
        ajustaCaminhoAutomatico(texto);
      }
      else{
        Serial.println("Comando não aceito para modo automático!");
      }
    }
  }

  if (tag_encontrada)
  {
    // caminhar até tag com visão computacional
  }
}
