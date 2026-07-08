# ENG4061_Projeto_Empilhadeira

Repositório que contém todo o material e código utilizado para a implementação das funcionalidades do robô empilhadeira.

# Como executar o código do Raspberry Pi

Para o programa main.py da pasta Raspberry é necessário estabelecer conexão via shh com o raspberry através de um computador conectado na mesma rede que o raspberry. Segue o passo a passo para conexão:
1. Abra um terminal
2. Insira o seguinte comando
ssh puc@"Endereço IP do raspberry"
  $ ssh puc@192.168.1.112
4. Realize o login


Para rodar Servidor:
python -m venv .venv
*mostrar diferentes formas de ativar a venv*
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
python servidor.py

Sobre o raspberry:
rodar 
pip install -r requirements.txt

realizar a calibragem antes rodando calibragem.py
gerar tabuleiro com
https://calib.io/pages/camera-calibration-pattern-generator
Para gerar o PDF exato que o seu código espera, preencha o site dessa forma:

Target Type: Checkerboard

Columns: 9

Rows: 6

Checker Width: 25 mm (isso equivale aos 0.025 metros do seu código)

Page format: A4 (ou o papel que você tiver na impressora)

rodar python main.py
