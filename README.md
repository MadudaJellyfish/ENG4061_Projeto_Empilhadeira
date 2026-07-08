# 📦 EN4061 — Empilhadeira Autônoma com Detecção de AprilTags

Projeto de robótica (PUC) que implementa uma **empilhadeira LEGO autônoma** capaz de buscar e se aproximar de tags **AprilTag** detectadas por uma câmera. O sistema pode ser pilotado manualmente por uma interface gráfica ou operar de forma autônoma através de uma máquina de estados.

---

## 🎯 Visão Geral da Arquitetura

O sistema é dividido em **três componentes** que se comunicam entre si:

```
┌─────────────────────────────────────────────────────────┐
│                    SERVIDOR (PC) — servidor.py           │
│  - Interface Tkinter para controle manual                │
│  - Exibe o vídeo em tempo real da câmera (WebSocket)     │
│  - Envia comandos ao robô via MQTT                       │
└───────────────┬──────────────────────▲──────────────────┘
                │ MQTT (TLS, porta 8883)│ WebSocket (porta 8766)
                │ comandos              │ vídeo
        ┌───────▼──────────────────────┴──────┐
        │   RASPBERRY PI — main.py             │
        │  - Visão computacional (OpenCV)      │
        │  - Detecção de AprilTags (pupil)     │
        │  - Ponte MQTT ↔ Serial               │
        └───────────────┬─────────────────────┘
                        │ Serial USB (115200 baud)
                        │ VIS_COMP / BUSCAR_TAG / comandos
        ┌───────────────▼─────────────────────┐
        │   ARDUINO — robotica_controle_lego   │
        │  - Controle dos motores (PWM)        │
        │  - Máquina de estados autônoma       │
        │  - Controle de velocidade P + encoders│
        └──────────────────────────────────────┘
```

**Resumo do fluxo:** o Servidor manda comandos por MQTT → o Raspberry repassa ao Arduino por Serial e devolve o vídeo/estado. Em modo automático, o Raspberry envia a posição da tag (`VIS_COMP`) e o Arduino navega sozinho até ela.

---

## 📁 Estrutura do Projeto

```
EN4061_Projeto_Empilhadeira/
├── arduino/
│   └── robotica_controle_lego.ino    # Controle dos motores + máquina de estados
├── raspberry_pi/
│   ├── main.py                       # Loop de visão computacional + WebSocket
│   ├── comunicacao_mqtt.py           # Ponte MQTT (comandos ↔ Serial)
│   ├── calibragem.py                 # Calibração da câmera (tabuleiro de xadrez)
│   ├── requirements.txt              # Dependências Python do Raspberry
│   ├── params_multilaser.npz         # Parâmetros de calibração (gerado)
│   └── .env                          # Credenciais/config (não versionar)
├── servidor/
│   ├── servidor.py                   # GUI Tkinter + cliente MQTT/WebSocket
│   ├── requirements.txt              # Dependências Python do servidor
│   └── .env                          # Credenciais/config (não versionar)
└── README.md                         # Este arquivo
```

---

## 🔧 Componentes do Sistema

### 1️⃣ Arduino — `robotica_controle_lego.ino`

Controla os motores e implementa a autonomia. Roda a **115200 baud**.

**Modos de operação:**
- **Manual** — recebe comandos diretos via Serial (`FRENTE`, `DIREITA`, `SUBIR`, etc.).
- **Automático** — máquina de estados que busca uma tag alvo.

**Máquina de estados (modo automático):**
```
IDLE ──────► PROCURANDO ──────► APROXIMANDO ──────► ALVO_ALCANCADO
(sem alvo)   (varredura em      (gira/avança em      (parado no alvo,
             passos curtos)     passos até alinhar)  informa a tag)
```
- **IDLE:** aguardando um alvo (`BUSCAR_TAG:ID`).
- **PROCURANDO:** executa uma sequência de passos (`SEQ_BUSCA = {frente, direita, direita, esquerda, esquerda, esquerda, frente}`), andando e pausando para a câmera enxergar. Se a tag certa aparece → `APROXIMANDO`.
- **APROXIMANDO:** se estiver torto, gira para o lado da tag; se centralizado, avança. Se perder a tag por >1,5 s → volta a `PROCURANDO`.
- **ALVO_ALCANCADO:** perto e alinhado o suficiente → para e envia `TAG_ENCONTRADA:ID`.

**Controle de velocidade:** proporcional (P) sobre a RPM medida pelos encoders — `PWM = Kp × (RPM_alvo − RPM_medido)`, com teto de segurança `PWM_MAX`.

**Parâmetros de tuning** (topo do `.ino`):
| Parâmetro | Valor | Descrição |
|-----------|-------|-----------|
| `Kp` | `3.0` | Ganho proporcional (↑ responde mais forte, ↓ mais suave) |
| `VEL_PROCURA` | `60` RPM | Velocidade durante a busca |
| `VEL_APROX` | `80` RPM | Velocidade durante a aproximação |
| `PWM_MAX` | `200` | Teto de PWM (segurança contra pico de corrente) |
| `PULSOS_POR_VOLTA` | `20` | Pulsos do encoder por volta (**calibrar!**) |
| `DIST_ALVO` | `0.60` m | Distância considerada "chegou" |
| `ANG_ALVO` | `8°` | Ângulo considerado "centralizado" |
| `TIMEOUT_VISAO` | `1500` ms | Tempo sem ver a tag para considerá-la perdida |

**Guia rápido de ajuste:** se o robô **dá trancos**, diminua `Kp` e `PWM_MAX`; se fica **mole/não gira**, aumente `Kp`; se **para longe demais** do alvo, aumente `DIST_ALVO`; se **não se alinha**, aumente `ANG_ALVO`.

**Pinos:**
| Pino | Função |
|------|--------|
| 2, 3 | Motor roda **esquerda** |
| 4, 5 | Motor roda **direita** |
| 6, 7 | Motor do **garfo** (elevação) |
| 18, 19 | Encoders das rodas (interrupções) |

---

### 2️⃣ Raspberry Pi — Visão Computacional

#### `main.py` — Loop principal
1. Captura vídeo da câmera em **320×240** (`cv2.VideoCapture(0)`).
2. Detecta AprilTags da família **`tag25h9`** (tamanho real **5 cm**) com `pupil_apriltags`.
3. Calcula distância (`Z`) e ângulo (`bearing`) 3D de cada tag (requer calibração).
4. Envia os dados ao Arduino por Serial **só em modo automático** (limitado a ~5 Hz / quando muda).
5. Transmite o vídeo (JPEG, qualidade 60) ao servidor via **WebSocket na porta 8766**.
6. Repassa ao servidor por MQTT os eventos `TAG_ENCONTRADA` / `TAG_PERDIDA` que o Arduino emite.

**Protocolo Serial (Raspberry → Arduino):**
```
VIS_COMP:id_tag;distancia_m;bearing_deg
Exemplo:  VIS_COMP:1;0.50;-5.2
```

#### `comunicacao_mqtt.py` — Ponte MQTT
Conecta ao broker (`mqtt.janks.dev.br:8883`, TLS) e faz a ponte entre MQTT e a Serial do Arduino.

- **Inscreve em `BMML/comando`** e repassa ao Arduino:
  - Movimento: `FRENTE`, `TRAS`, `ESQUERDA`, `DIREITA`, `SUBIR`, `DESCER`, `PARAR_ANDAR`, `PARAR_SUBIR`
  - Modo: `MANUAL` / `AUTOMATICO`
  - Alvo: `BUSCAR_TAG:1`
- **Publica em `BMML/status`:** `TAG_ENCONTRADA:1`, `TAG_PERDIDA:1`
- A flag `modo_automatico` decide se a visão computacional envia dados ao Arduino (inicia em `True`).

#### `calibragem.py` — Calibração da câmera
Calcula os parâmetros intrínsecos (matriz `K` + distorção) a partir de um tabuleiro de xadrez, e salva em `params_multilaser.npz`. Sem isso, a detecção 2D funciona mas **a pose 3D (distância/ângulo) fica desativada**.
- Tabuleiro: **8×5 cantos internos**, quadrados de **2,5 cm** (`0.025 m`).
- Usa `cv2.VideoCapture(1)` (câmera USB secundária).
- Teclas: **`s`** salva um frame válido, **`q`** encerra e calibra. Capture **20+ imagens** em ângulos variados.

---

### 3️⃣ Servidor — `servidor.py`

GUI em Tkinter para pilotar o robô e ver a câmera. Conecta ao MQTT (comandos) e ao WebSocket do Raspberry (vídeo).

**Elementos da interface:**
- Label de **status** (recebe do tópico `BMML/status`).
- **Tela de vídeo** ao vivo com overlay das tags.
- Campo **"Buscar Tag ID"** + botão → envia `BUSCAR_TAG:<id>`.
- Botão **Ativar/Desativar Modo Manual** (verde = automático, vermelho = manual).
- Botões direcionais e do garfo (pressionar/soltar = anda/para).

**Teclas de atalho** (equivalem a pressionar e segurar os botões):
| Tecla | Ação |
|-------|------|
| `W` | Frente |
| `S` | Trás |
| `A` | Esquerda |
| `D` | Direita |
| `Q` | Subir garfo |
| `E` | Descer garfo |

> ⚠️ Os comandos de movimento só têm efeito com o **Modo Manual ativado**. Um debounce de 50 ms evita paradas falsas ao soltar a tecla.

---

## 🚀 Como Rodar

### Pré-requisitos
- **Arduino IDE** (para carregar o `.ino`)
- **Python 3.8+** no PC (servidor) e no Raspberry Pi
- Servidor e Raspberry na **mesma rede** (o vídeo WebSocket é local)

### 1. Arduino
1. Abra `arduino/robotica_controle_lego.ino` na Arduino IDE.
2. Selecione a placa e a porta COM corretas.
3. Faça o upload. O monitor serial roda a **115200 baud**.

### 2. Raspberry Pi

Conecte-se ao Raspberry por SSH (mesma rede):
```bash
ssh puc@<IP_DO_RASPBERRY>      # ex.: ssh puc@192.168.1.112
```

Instale as dependências e configure o `.env`:
```bash
cd raspberry_pi
pip install -r requirements.txt
# além do requirements, o projeto usa: pyserial, websockets, paho-mqtt, python-dotenv

# Arquivo .env (raspberry_pi/.env)
# BROKER=mqtt.janks.dev.br
# PORT=8883
```

(Opcional, mas recomendado) **calibre a câmera** antes:
```bash
python calibragem.py     # aperte 's' p/ capturar (20+ imagens), 'q' p/ finalizar
```
> Gere o tabuleiro em https://calib.io/pages/camera-calibration-pattern-generator
> — Checkerboard, **Columns 9 / Rows 6** (8×5 cantos internos), **Checker Width 25 mm**, papel A4.

Rode a visão computacional:
```bash
python main.py
```

### 3. Servidor (PC)

```bash
cd servidor
python -m venv .venv

# Ativar a venv:
#   PowerShell:      .venv\Scripts\Activate.ps1
#   CMD:             .venv\Scripts\activate.bat
#   Git Bash/Linux:  source .venv/bin/activate

python -m pip install --upgrade pip
pip install -r requirements.txt
python servidor.py
```

Configure o `.env` (`servidor/.env`):
```
BROKER=mqtt.janks.dev.br
PORT=8883
USER=aula
PASSWORD=<senha_do_broker>
IP_RASPBERRY=192.168.x.x
WS_PORT=8766
```

---

## 🎮 Operação

**Modo Manual:**
1. Clique em **"Ativar Modo Manual"** (o botão fica vermelho).
2. Pilote com **W A S D** (rodas) e **Q / E** (garfo), ou usando o mouse nos botões.

**Modo Automático:**
1. Deixe o botão em **"Ativar Modo Manual"** (verde = automático).
2. Coloque uma AprilTag `tag25h9` no chão.
3. Digite o **ID da tag** no campo "Buscar Tag ID" e clique em **Enviar** (envia `BUSCAR_TAG:<id>`).
4. O robô procura, se aproxima e para no alvo. O status **`TAG_ENCONTRADA`** aparece na GUI.

---

## 📊 Fluxo de Dados — Busca Automática

```
1. Servidor  ── MQTT "BUSCAR_TAG:1" ──►  Raspberry
2. Raspberry ── Serial "BUSCAR_TAG:1" ─►  Arduino    (entra em PROCURANDO)
3. Câmera detecta a tag → Raspberry calcula distância e ângulo
4. Raspberry ── Serial "VIS_COMP:1;0.50;-5.2" ─► Arduino  (muda p/ APROXIMANDO)
5. Arduino faz ajustes finos (girar/avançar) até ficar perto e alinhado
6. Arduino  ── Serial "TAG_ENCONTRADA:1" ─► Raspberry
7. Raspberry ── MQTT "BMML/status = TAG_ENCONTRADA:1" ─► Servidor (exibe na GUI)
```

---

## 🐛 Troubleshooting

| Problema | Solução |
|----------|---------|
| Arduino não conecta | Confira a porta (`/dev/ttyACM0` no Linux/RPi), baudrate 115200 e drivers CH340 |
| Câmera não inicia | Verifique o índice em `cv2.VideoCapture(0)` — tente `1`; note que a calibração usa índice `1` |
| MQTT não conecta | Confira credenciais no `.env` e firewall (porta 8883/TLS) |
| Vídeo não aparece | Servidor e Raspberry precisam estar na **mesma rede**; confira `IP_RASPBERRY` e `WS_PORT` |
| Vídeo travado/lag | Reduza a qualidade JPEG (`IMWRITE_JPEG_QUALITY`) no `main.py` |
| Tag não é detectada | Rode `calibragem.py`, melhore a iluminação, confira família `tag25h9` e tamanho de 5 cm |
| Pose 3D desativada | Falta o `params_multilaser.npz` — rode a calibração |
| Robô não para no alvo | Ajuste `DIST_ALVO` e `ANG_ALVO` no Arduino |
| Robô dá trancos | Diminua `Kp` e `PWM_MAX` |

---

## 📚 Referências

- **AprilTags:** https://april.eecs.umich.edu/
- **Pupil Labs AprilTags:** https://github.com/pupil-labs/apriltags
- **OpenCV:** https://opencv.org/
- **Paho MQTT:** https://www.eclipse.org/paho/
- **Gerador de tabuleiro:** https://calib.io/pages/camera-calibration-pattern-generator

---

## 👩‍💻 Informações

**Projeto:** EN4061 — Projeto de Robótica (PUC)
**Hardware:** Arduino + Raspberry Pi + LEGO Mindstorms
