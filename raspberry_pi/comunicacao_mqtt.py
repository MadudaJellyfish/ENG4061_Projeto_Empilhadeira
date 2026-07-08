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
mqtt_client = None
mqtt_properties = None


def mqtt_topic(nome):
    return f"{MQTT_PREFIX}/{nome}"

# Função para o main.py consultar o estado atual
def is_modo_automatico():
    global modo_automatico
    return modo_automatico

def publicar_status(mensagem):
    global mqtt_client, mqtt_properties
    if not mensagem or mqtt_client is None or mqtt_properties is None:
        return
    if mqtt_client.is_connected():
        mqtt_client.publish(mqtt_topic("status"), mensagem, qos=2, properties=mqtt_properties)

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
    global arduino_serial, mqtt_client, mqtt_properties
    arduino_serial = referencia_serial
    
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
