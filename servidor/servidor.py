import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
import tkinter as tk
import cv2
import numpy as np
from PIL import Image, ImageTk
import websocket 
import threading 
import os
from dotenv import load_dotenv

load_dotenv()

BROKER = os.getenv("BROKER")
PORT = int(os.getenv("PORT"))
USER = os.getenv("USER")
PASSWORD = os.getenv("PASSWORD")

IP_RASPBERRY = os.getenv("IP_RASPBERRY")
WS_PORT = int(os.getenv("WS_PORT"))

# Variáveis globais
latest_frame = None
modo_manual_ativo = False
MQTT_PREFIX = "BMML"
root = None
status_var = None


def mqtt_topic(nome):
    return f"{MQTT_PREFIX}/{nome}"

# --- Rastreador de teclas agora inclui os garfos ---
estado_teclas = {
    "FRENTE": False,
    "TRAS": False,
    "ESQUERDA": False,
    "DIREITA": False,
    "SUBIR": False,
    "DESCER": False
}

# --- Lógica MQTT (Apenas Comandos) ---
def on_connect(client, userdata, flags, rc):
    print("Código MQTT conectado:", rc)
    client.subscribe(mqtt_topic("comando"))
    client.subscribe(mqtt_topic("status"))


def atualizar_status(mensagem):
    global root, status_var
    if root is not None and status_var is not None and root.winfo_exists():
        root.after(0, lambda: status_var.set(f"Status: {mensagem}"))


def on_message(client, userdata, msg):
    topico = msg.topic
    try:
        payload = msg.payload.decode("utf-8")
        print(f"Mensagem recebida no tópico {topico}: {payload}")
    except Exception as e:
        print(f"Erro ao ler payload: {e}")

    if topico == mqtt_topic("status"):
        atualizar_status(payload)
        return

def envia_mensagem(msg, topico):
    topico_com_prefixo = mqtt_topic(topico)
    print(f"Enviando '{msg}' -> {topico_com_prefixo}")
    client.publish(topico_com_prefixo, msg, qos=2, properties=properties)

# --- Lógica WebSocket (Apenas Imagem) ---
def on_ws_message(ws, message):
    global latest_frame
    
    if isinstance(message, bytes):
        nparr = np.frombuffer(message, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            latest_frame = frame_rgb 
        else:
            print("Falha ao decodificar a imagem do WebSocket.")

def on_ws_open(ws):
    print("✅ Vídeo WebSocket CONECTADO com sucesso ao Raspberry!")

def on_ws_error(ws, error):
    print(f"❌ Erro no WebSocket de vídeo (o Raspberry está na mesma rede?): {error}")

def iniciar_websocket():
    print(f"Conectando ao vídeo WebSocket em ws://{IP_RASPBERRY}:{WS_PORT}...")
    ws = websocket.WebSocketApp(
        f"ws://{IP_RASPBERRY}:{WS_PORT}", 
        on_message=on_ws_message,
        on_open=on_ws_open,
        on_error=on_ws_error
    )
    ws.run_forever()

# --- Lógica de Controle ---
cancelar_parada_ids = {} # Memória para ignorar o "falso soltar" da tecla

def alternar_modo_manual():
    global modo_manual_ativo
    modo_manual_ativo = not modo_manual_ativo
    
    if modo_manual_ativo:
        btn_modo.config(text="Desativar Modo Manual", bg="#d9534f", fg="white") 
        envia_mensagem("MANUAL", "comando")
        atualizar_status("Aguardando status...")
    else:
        btn_modo.config(text="Ativar Modo Manual", bg="#5cb85c", fg="white") 
        envia_mensagem("AUTOMATICO", "comando")
        atualizar_status("Aguardando status...")
        
    root.focus_set() 

def iniciar_movimento(direcao, event=None):
    # Se havia uma ordem de parada falsa agendada, nós a cancelamos!
    if direcao in cancelar_parada_ids and cancelar_parada_ids[direcao] is not None:
        root.after_cancel(cancelar_parada_ids[direcao])
        cancelar_parada_ids[direcao] = None

    if modo_manual_ativo:
        if not estado_teclas[direcao]:
            estado_teclas[direcao] = True
            envia_mensagem(direcao, "comando")
    else:
        if not estado_teclas[direcao]:
            estado_teclas[direcao] = True
            print("Aviso: Ative o modo manual para pilotar a empilhadeira.")

def parar_movimento(direcao, event=None):
    # Se foi um evento de teclado (event is not None), aguarda 50ms para confirmar
    if event is not None:
        cancelar_parada_ids[direcao] = root.after(50, lambda: executar_parada(direcao))
    else:
        # Se foi clique de mouse, para na mesma hora
        executar_parada(direcao)

def executar_parada(direcao):
    if estado_teclas[direcao]:
        estado_teclas[direcao] = False
        if direcao in ["FRENTE", "TRAS", "ESQUERDA", "DIREITA"]:
            envia_mensagem("PARAR_ANDAR", "comando")
        elif direcao in ["SUBIR", "DESCER"]:
            envia_mensagem("PARAR_SUBIR", "comando")

# Setup MQTT
client = mqtt.Client(client_id="robotica_servidor_1b")
client.username_pw_set(USER, PASSWORD)
client.tls_set()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, keepalive=60)

properties = Properties(PacketTypes.PUBLISH)
properties.MessageExpiryInterval = 120
client.loop_start()

# --- Configuração da Interface Gráfica (Tkinter) ---
root = tk.Tk()
root.title("Empilhadeira =D")
root.geometry("800x750")

status_var = tk.StringVar(value="Aguardando status...")
status_label = tk.Label(root, textvariable=status_var, bg="#1f1f1f", fg="#ffffff", wraplength=720, justify="left", padx=10, pady=8)
status_label.pack(fill="x", padx=10, pady=(10, 0))

label = tk.Label(root, text="Aguardando imagem...", bg="black", fg="white")
label.pack(pady=10)

# --- Frame e controles para enviar o ID da Tag ---
frame_busca = tk.Frame(root)
frame_busca.pack(pady=5)

lbl_tag = tk.Label(frame_busca, text="Buscar Tag ID:", font=("Arial", 12, "bold"))
lbl_tag.grid(row=0, column=0, padx=5)

entry_tag = tk.Entry(frame_busca, font=("Arial", 12), width=10)
entry_tag.grid(row=0, column=1, padx=5)

def enviar_id_tag():
    tag_id = entry_tag.get().strip()
    # Verifica se o usuário digitou apenas números
    if tag_id.isdigit(): 
        # Envia a mensagem no formato "BUSCAR_TAG:10"
        envia_mensagem(f"BUSCAR_TAG:{tag_id}", "comando")
        entry_tag.delete(0, tk.END) # Limpa o campo após enviar
        root.focus_set() # Tira o foco da caixa de texto para as teclas voltarem a funcionar
    else:
        print("Erro: O ID da tag deve ser um número válido!")

btn_enviar_tag = tk.Button(frame_busca, text="Enviar p/ Arduino", bg="#f0ad4e", fg="white", font=("Arial", 10, "bold"), command=enviar_id_tag)
btn_enviar_tag.grid(row=0, column=2, padx=5)

frame_controles = tk.Frame(root)
frame_controles.pack(pady=10)

# Aumentei o columnspan para abraçar as novas colunas do garfo
btn_modo = tk.Button(frame_controles, text="Ativar Modo Manual", bg="#5cb85c", fg="white", font=("Arial", 12, "bold"), width=25, command=alternar_modo_manual)
btn_modo.grid(row=0, column=0, columnspan=5, pady=(0, 15))

# Botões de Movimentação
btn_frente = tk.Button(frame_controles, text="Frente (W)", width=12, height=2)
btn_esq = tk.Button(frame_controles, text="Esquerda (A)", width=12, height=2)
btn_dir = tk.Button(frame_controles, text="Direita (D)", width=12, height=2)
btn_tras = tk.Button(frame_controles, text="Trás (S)", width=12, height=2)

btn_frente.grid(row=1, column=1, pady=2)
btn_esq.grid(row=2, column=0, padx=2)
btn_dir.grid(row=2, column=2, padx=2)
btn_tras.grid(row=3, column=1, pady=2)

# Colocamos eles na coluna 4 (com um espaçamento grande em x para separar do direcional)
btn_subir = tk.Button(frame_controles, text="Subir Garfo (Q)", width=15, height=2, bg="#5bc0de")
btn_descer = tk.Button(frame_controles, text="Descer Garfo (E)", width=15, height=2, bg="#5bc0de")

btn_subir.grid(row=1, column=4, padx=(40, 0))
btn_descer.grid(row=3, column=4, padx=(40, 0))

# --- Binds de Mouse (Rodas) ---
btn_frente.bind("<ButtonPress-1>", lambda e: iniciar_movimento("FRENTE"))
btn_frente.bind("<ButtonRelease-1>", lambda e: parar_movimento("FRENTE"))
btn_esq.bind("<ButtonPress-1>", lambda e: iniciar_movimento("ESQUERDA"))
btn_esq.bind("<ButtonRelease-1>", lambda e: parar_movimento("ESQUERDA"))
btn_dir.bind("<ButtonPress-1>", lambda e: iniciar_movimento("DIREITA"))
btn_dir.bind("<ButtonRelease-1>", lambda e: parar_movimento("DIREITA"))
btn_tras.bind("<ButtonPress-1>", lambda e: iniciar_movimento("TRAS"))
btn_tras.bind("<ButtonRelease-1>", lambda e: parar_movimento("TRAS"))

# --- Binds de Mouse (Garfos) ---
btn_subir.bind("<ButtonPress-1>", lambda e: iniciar_movimento("SUBIR"))
btn_subir.bind("<ButtonRelease-1>", lambda e: parar_movimento("SUBIR"))
btn_descer.bind("<ButtonPress-1>", lambda e: iniciar_movimento("DESCER"))
btn_descer.bind("<ButtonRelease-1>", lambda e: parar_movimento("DESCER"))

root.focus_set()

# --- Binds de Teclado (Rodas) ---
for key in ['w', 'W']: 
    root.bind(f'<KeyPress-{key}>', lambda e: iniciar_movimento("FRENTE"))
    root.bind(f'<KeyRelease-{key}>', lambda e: parar_movimento("FRENTE"))
for key in ['a', 'A']: 
    root.bind(f'<KeyPress-{key}>', lambda e: iniciar_movimento("ESQUERDA"))
    root.bind(f'<KeyRelease-{key}>', lambda e: parar_movimento("ESQUERDA"))
for key in ['s', 'S']: 
    root.bind(f'<KeyPress-{key}>', lambda e: iniciar_movimento("TRAS"))
    root.bind(f'<KeyRelease-{key}>', lambda e: parar_movimento("TRAS"))
for key in ['d', 'D']: 
    root.bind(f'<KeyPress-{key}>', lambda e: iniciar_movimento("DIREITA"))
    root.bind(f'<KeyRelease-{key}>', lambda e: parar_movimento("DIREITA"))

# --- Binds de Teclado (Garfos) ---
for key in ['q', 'Q']: 
    root.bind(f'<KeyPress-{key}>', lambda e: iniciar_movimento("SUBIR"))
    root.bind(f'<KeyRelease-{key}>', lambda e: parar_movimento("SUBIR"))
for key in ['e', 'E']: 
    root.bind(f'<KeyPress-{key}>', lambda e: iniciar_movimento("DESCER"))
    root.bind(f'<KeyRelease-{key}>', lambda e: parar_movimento("DESCER"))

img_tk = None 

def atualizar_interface():
    global latest_frame, img_tk
    if latest_frame is not None:
        img_pil = Image.fromarray(latest_frame)
        img_tk = ImageTk.PhotoImage(image=img_pil)
        
        label.config(image=img_tk, text="") 
        
        latest_frame = None 
    root.after(30, atualizar_interface)

atualizar_interface()

def on_closing():
    print("Desconectando MQTT e fechando interface...")
    client.loop_stop()
    client.disconnect()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

# Inicia a thread do WebSocket (Vídeo)
threading.Thread(target=iniciar_websocket, daemon=True).start()

try:
    print("Iniciando interface...")
    root.mainloop()
except KeyboardInterrupt:    
    on_closing()
