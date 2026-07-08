#!/usr/bin/env python3
"""
================================================================================
   RASPBERRY PI - VISÃO COMPUTACIONAL E COMUNICAÇÃO
================================================================================

RESPONSABILIDADES:
  1. Capturar vídeo da câmera em tempo real
  2. Detectar tags AprilTags usando Pupil Labs (detecção 2D e 3D)
  3. Transmitir vídeo para o servidor via WebSocket (feedback visual)
  4. Enviar dados de visão ao Arduino via Serial (controle automático)
  5. Receber comandos MQTT do servidor

FLUXO PRINCIPAL:
  Câmera -> Detector AprilTags -> Serial para Arduino + WebSocket para Servidor
                                -> MQTT (status/eventos)

TAGS AprilTag:
  - Família: tag25h9
  - Tamanho: 5cm (0.05m)
  - Formato de envio: VIS_COMP:id;distancia;bearing

"""

import cv2                          # Processamento de imagem
import numpy as np                  # Operações numéricas
import math                         # Trigonometria
import time                         # Controle de tempo
import serial                       # Comunicação com Arduino
import threading                    # Threads para servidor WebSocket
import asyncio                      # Event loop assíncrono
import websockets                   # WebSocket para streaming de vídeo
from pupil_apriltags import Detector  # Detecção de tags

# Importa o módulo de comunicacao_mqtt 
import comunicacao_mqtt

# ============================================================================
# VARIÁVEIS GLOBAIS
# ============================================================================
frame_bytes_ws = b''    # Buffer com o frame JPEG codificado para envio

# O '*args' garante que vai funcionar independente da versão do websockets do RPi
# ============================================================================
# SERVIDOR WEBSOCKET - Streaming de Vídeo para o Cliente
# ============================================================================

async def stream_video(websocket, *args):
    """
    Envia frames de vídeo JPEG comprimidos via WebSocket a cada nova imagem.
    - Usa '*args' para compatibilidade com múltiplas versões do websockets
    - Envia apenas quando há novo frame disponível
    """
    global frame_bytes_ws
    ultimo_frame_enviado = b''
    
    print("Novo cliente conectado ao vídeo!")
    
    while True:
        # Só envia se houver uma imagem E ela for diferente da que acabamos de enviar
        if len(frame_bytes_ws) > 0: # and frame_bytes_ws != ultimo_frame_enviado:
            try:
                await websocket.send(frame_bytes_ws)
                ultimo_frame_enviado = frame_bytes_ws
            except websockets.exceptions.ConnectionClosed:
                print("Cliente de vídeo desconectado.")
                break
        
        # Alivia o processador do RPi enquanto não tem imagem nova
        await asyncio.sleep(0.01)

async def run_ws_server():
    # O "async with" garante que o servidor suba corretamente já atrelado ao loop ativo
    async with websockets.serve(stream_video, "0.0.0.0", 8766):
        print("Servidor de Vídeo WebSocket iniciado na porta 8766")
        # Cria um futuro vazio e aguarda ele infinitamente (substitui o loop.run_forever())
        await asyncio.Future() 

def iniciar_servidor_ws():
    # O asyncio.run() cria o loop, executa a função e limpa tudo sozinho
    asyncio.run(run_ws_server())


# ============================================================================
# FUNÇÕES AUXILIARES - Conversão e Comunicação
# ============================================================================

def get_robot_yaw(R):
    """
    Extrai o ângulo yaw (rotação em Z) da matriz de rotação 3x3 do AprilTag.
    Retorna o ângulo em GRAUS (positivo = rotação anti-horária).
    """
    yaw_rad = math.atan2(R[0, 2], R[2, 2])
    return math.degrees(yaw_rad)

def configurar_arduino():
    """
    Abre a conexão Serial com o Arduino para envio de comandos.
    Porta: /dev/ttyACM0 (padrão para Arduino em Linux)
    Baudrate: 115200 (deve corresponder ao Arduino)
    """
    porta = '/dev/ttyACM0'
    baudrate = 115200
    try:
        ser = serial.Serial(porta, baudrate, timeout=1)
        time.sleep(2) 
        print("Conexão com o Arduino estabelecida com sucesso!")
        return ser
    except Exception as e:
        print(f"Erro na comunicação com o Arduino: {e}")
        return None

# ============================================================================
# FUNÇÃO PRINCIPAL
# ============================================================================

def main():
    """
    Inicializa todos os componentes e executa o loop principal:
    1. Lê frame da câmera
    2. Detecta tags AprilTags
    3. Envia dados ao Arduino (Serial) e Servidor (WebSocket)
    4. Processa mensagens MQTT do servidor
    """
    global frame_bytes_ws
    
    # Inicializa conexões
    ser = configurar_arduino()
    mqtt_client = comunicacao_mqtt.inicializar_mqtt(ser)

    # Inicia a thread do WebSocket Server
    threading.Thread(target=iniciar_servidor_ws, daemon=True).start()

    try:
        with np.load("params_multilaser.npz") as data:
            mtx = data['mtx']
            cam_params = [mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2]]
            print("Calibração carregada com sucesso!")
    except Exception as e:
        cam_params = None
        print(f"Aviso: Arquivo de calibração não carregado ({e}). Pose 3D desativada.")

    at_detector = Detector(
        families="tag25h9", nthreads=2, quad_decimate=2.0,
        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0
    )
    
    cap = cv2.VideoCapture(0) 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    TAG_SIZE = 0.05 

    print("Sistema iniciado. Iniciando detecção... Pressione Ctrl+C para sair.")

    tempo_ultimo_envio_visao = 0.0
    ultimo_comando_visao = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Falha ao capturar frame da câmera.")
                break
                        
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            if cam_params:
                results = at_detector.detect(gray, estimate_tag_pose=True, camera_params=cam_params, tag_size=TAG_SIZE)
            else:
                results = at_detector.detect(gray, estimate_tag_pose=False)

            for r in results:
                pts = np.array(r.corners, dtype=np.int32)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                origem_texto = (int(r.corners[0][0]), int(r.corners[0][1]) - 10)

                if r.pose_t is not None and r.pose_R is not None:
                    tx, tz = r.pose_t[0][0], r.pose_t[2][0] 
                    bearing = math.degrees(math.atan2(tx, tz))
                    tag_yaw = get_robot_yaw(r.pose_R)
                    info = f"ID:{r.tag_id} Z:{tz:.2f}m Pos:{bearing:.1f}deg Ori:{tag_yaw:.1f}deg"
                    
                    if comunicacao_mqtt.is_modo_automatico():
                        if ser and ser.is_open:
                            agora = time.time()
                            comando_visao = f"VIS_COMP:{r.tag_id};{tz:.2f};{bearing:.1f}\n"

                            if agora - tempo_ultimo_envio_visao >= 0.2 or comando_visao != ultimo_comando_visao:
                                ser.write(comando_visao.encode('utf-8'))
                                tempo_ultimo_envio_visao = agora
                                ultimo_comando_visao = comando_visao
                else:
                    info = f"ID: {r.tag_id} (Sem Pose)"

                cv2.putText(frame, info, origem_texto, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                cv2.putText(frame, info, origem_texto, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if len(results) > 0:
                print("Dados da Visão Computacional: ")
                print(info)
                print("\n")
                        
            # --- Envia o quadro para o cliente WebSocket ---
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60] # Qualidade menor = menos lag
            sucesso, buffer = cv2.imencode(".jpg", frame, encode_param)
            if sucesso:
                frame_bytes_ws = buffer.tobytes()
            
            if ser and ser.is_open and ser.in_waiting > 0:
                linha = ser.readline().decode('utf-8').rstrip()
                if linha:
                    print(f"Mensagem do Arduino: {linha}")
                    if linha.startswith("TAG_ENCONTRADA:") or linha.startswith("TAG_PERDIDA:"):
                        comunicacao_mqtt.publicar_status(linha)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nEncerrando o programa de forma segura...")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Conexão com o Arduino fechada.")
        
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        cap.release()
        print("Recursos liberados. Até logo!")

if __name__ == "__main__":
    main()
