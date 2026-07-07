import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
import os
from dotenv import load_dotenv

load_dotenv()

BROKER = os.getenv("BROKER")     
PORT = int(os.getenv("PORT"))

# Variáveis globais
arduino_serial = None 
modo_automatico = True # Inicia como True, combinando com o servidor.py
MQTT_PREFIX = "BMML"


def mqtt_topic(nome):
    return f"{MQTT_PREFIX}/{nome}"

# Função para o main.py consultar o estado atual
def is_modo_automatico():
    global modo_automatico
    return modo_automatico

def on_connect(client, userdata, flags, rc):
    print(f"MQTT Conectado com código: {rc}")
    client.subscribe(mqtt_topic("comando"))

def on_message(client, userdata, msg):
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
    global arduino_serial
    arduino_serial = referencia_serial
    
    client = mqtt.Client(client_id="robotica_1b")
    #user = os.getenv("USER") 
    #password = os.getenv("PASSWORD") 
    client.username_pw_set("aula", "zowmad-tavQez")
    client.tls_set()
    
    client.on_connect = on_connect
    client.on_message = on_message
    
    properties = Properties(PacketTypes.PUBLISH)
    properties.MessageExpiryInterval = 120
    
    client.connect("mqtt.janks.dev.br", 8883, keepalive=60)
    client.loop_start() 
    
    return client
