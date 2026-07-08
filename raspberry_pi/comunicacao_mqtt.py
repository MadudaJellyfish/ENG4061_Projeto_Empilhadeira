"""
================================================================================
   COMUNICAÇÃO MQTT - Raspberry Pi
================================================================================

RESPONSABILIDADES:
  - Conectar ao broker MQTT remoto (mqtt.janks.dev.br)
  - Receber comandos do servidor para o robô
  - Publicar status e eventos (TAG_ENCONTRADA, TAG_PERDIDA)
  - Gerenciar modo de operação (MANUAL vs AUTOMÁTICO)

TÓPICOS MQTT:
  - Recebe: BMML/comando
    Exemplos: FRENTE, DIREITA, ESQUERDA, TRAS, SUBIR, DESCER, BUSCAR_TAG:1
  - Publica: BMML/status
    Exemplos: TAG_ENCONTRADA:1, TAG_PERDIDA:1, Sistema iniciado

VARIÁVEIS DE ESTADO:
  - modo_automatico: controla se está em busca automática ou controle manual
  - arduino_serial: referência para envio de comandos ao Arduino

"""

import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
import os
from dotenv import load_dotenv

load_dotenv()

# ============================================================================
# CONFIGURAÇÕES DO BROKER MQTT
# ============================================================================
BROKER = os.getenv("BROKER")     # Carrega do arquivo .env
PORT = int(os.getenv("PORT"))    # Porta TLS

# ============================================================================
# VARIÁVEIS GLOBAIS
# ============================================================================
arduino_serial = None              # Referência para serial do Arduino
modo_automatico = True             # Inicia como automático, será alterado por MQTT
MQTT_PREFIX = "BMML"               # Prefixo dos tópicos
mqtt_client = None                 # Cliente MQTT
mqtt_properties = None             # Propriedades MQTT (timeout, etc)


# ============================================================================
# FUNÇÕES AUXILIARES
# ============================================================================

def mqtt_topic(nome):
    """Constrói o nome completo do tópico MQTT com o prefixo."""
    return f"{MQTT_PREFIX}/{nome}"

def is_modo_automatico():
    global modo_automatico
    return modo_automatico

def publicar_status(mensagem):
    global mqtt_client, mqtt_properties
    if not mensagem or mqtt_client is None or mqtt_properties is None:
        return
    if mqtt_client.is_connected():
        mqtt_client.publish(mqtt_topic("status"), mensagem, qos=2, properties=mqtt_properties)

# ============================================================================
# CALLBACKS DO MQTT
# ============================================================================

def on_connect(client, userdata, flags, rc):
    """Chamado quando o cliente se conecta ao broker."""
    print(f"MQTT Conectado com código: {rc}")
    client.subscribe(mqtt_topic("comando"))  # Inscreve-se no tópico de comandos

def on_message(client, userdata, msg):
    """Chamado quando uma mensagem é recebida num tópico inscrito."""
    global modo_automatico
    topico = msg.topic
    payload = msg.payload.decode("utf-8")
    
    print(f"MQTT Recebido - Tópico: {topico} | Mensagem: {payload}")

    # Repassa a instrução
    if topico == mqtt_topic("comando"):
        comando = payload.strip().upper()

        if comando.startswith("BUSCAR_TAG:"):
            print(f"Enviando instrução de busca para o Arduino: {comando}")
            if arduino_serial and arduino_serial.is_open:
                # Manda a string "BUSCAR_TAG:ID\n" para o Arduino ler
                arduino_serial.write(f"{comando}\n".encode('utf-8'))
            return # Usa o 'return' para ele não continuar pros outros IFs abaixo
        
        # --- Atualiza a variável de controle de modo ---
        if comando == "MANUAL":
            modo_automatico = False
            print("Modo MANUAL ativado. Visão computacional pausada.")
        elif comando == "AUTOMATICO":
            modo_automatico = True
            print("Modo AUTOMÁTICO ativado. Visão computacional rodando.")
            
        if arduino_serial and arduino_serial.is_open:
            # Filtro de segurança para comandos de movimento
            if comando in ["FRENTE", "DIREITA", "ESQUERDA", "TRAS", "PARAR_ANDAR", "SUBIR", "DESCER", "PARAR_SUBIR", "MANUAL", "AUTOMATICO"]:
                arduino_serial.write(f"{comando}\n".encode('utf-8'))

def inicializar_mqtt(referencia_serial):
    """
    Inicializa a conexão MQTT com o broker remoto.
    
    Argumentos:
      referencia_serial: objeto Serial do Arduino para repasse de comandos
    
    Retorna:
      mqtt_client: cliente MQTT configurado e conectado
    """
    global arduino_serial, mqtt_client, mqtt_properties
    arduino_serial = referencia_serial  # Guarda referência para repasse de comandos
    
    mqtt_client = mqtt.Client(client_id="robotica_1b")
    mqtt_client.username_pw_set("aula", "zowmad-tavQez")
    mqtt_client.tls_set()
    
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    
    mqtt_properties = Properties(PacketTypes.PUBLISH)
    mqtt_properties.MessageExpiryInterval = 120
    
    mqtt_client.connect("mqtt.janks.dev.br", 8883, keepalive=60)
    mqtt_client.loop_start() 
    
    return mqtt_client
