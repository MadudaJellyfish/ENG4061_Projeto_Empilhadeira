import cv2
import numpy as np

# Configurações do tabuleiro
CHECKERBOARD = (8, 5) # Cantos internos do seu tabuleiro
SQUARE_SIZE = 0.025   # Tamanho do quadrado em METROS (ex: 2.5cm = 0.025)

# Classe que organiza a calibração da camera
class Calibration:
    def __init__(self):
        self.capture = cv2.VideoCapture(1)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.criterio = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Modelo matemático do tabuleiro
        self.objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32) 
        self.objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE
        
        # Arrays com os pontos do mundo real e do plano da imagem
        self.objpoints = [] # Pontos 3d no mundo real
        self.imgpoints = [] # Pontos 2d no plano da imagem

    # ------------------------------------------ #
    # Captura a imagem do tabuleiro
    def capture_chess_image(self):
        print("Pressione 's' para salvar um frame ou 'q' para terminar a captura e iniciar a calibração.")
        print("Capture pelo menos 20 imagens!!!")
        
        while True:
            ret, self.img = self.capture.read()
            if not ret: 
                break

            # Cria uma cópia da imagem apenas para desenhar por cima sem estragar a original
            display_frame = self.img.copy()
            self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

            ret_c, corners = cv2.findChessboardCorners(
                self.gray, 
                CHECKERBOARD, 
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            if ret_c:
                # Refina as coordenadas encontradas
                corners2 = cv2.cornerSubPix(self.gray, corners, (11, 11), (-1, -1), self.criterio)
                cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners2, ret_c)
                cv2.putText(display_frame, "TABULEIRO ENCONTRADO! Aperte 'S'", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "Buscando tabuleiro...", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('Calibragem', display_frame)
            key = cv2.waitKey(1)
            
            if key == ord('s'):
                if ret_c:
                    self.objpoints.append(self.objp)
                    self.imgpoints.append(corners2)
                    print(f"Imagem {len(self.objpoints)} salva com sucesso!")
                else:
                    print("Tabuleiro não detectado. Ajuste a câmera.")
                    
            elif key == ord('q'):
                print("Encerrando captura...")
                break
        
        self.capture.release()
        cv2.destroyAllWindows()

    # ------------------------------------- #
    # Pega os pontos do mundo 3d e compara com os pontos 2d distorcidos capturados pela câmera
    def calibrate(self):
        ret, self.mtx, self.distort, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None
        )
        print ('ret: ', ret)
        print ('mtx:\n', self.mtx)
        print ('distort: ', self.distort)
        
        print("Calibração salva com sucesso no arquivo 'params_multilaser.npz'!")
        # ATUALIZADO: Salva com o nome esperado pelo main.py
        np.savez('params_multilaser.npz', mtx=self.mtx, dist=self.distort, rvecs=self.rvecs, tvecs=self.tvecs)

    # --------------------------------------- #
    def undistort(self):
        h, w = self.img.shape[:2]
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.distort, (w,h), 1, (w,h))
        
        # undistort
        undst = cv2.undistort(self.img, self.mtx, self.distort, None, self.newcameramtx)
        
        # crop the image
        x, y, self.w, self.h = self.roi
        undst = undst[y:y+self.h, x:x+self.w]
        
        cv2.imshow('undistorted', undst)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()

    # ------------------------------------------------------------ #
    def remapping(self):
        h, w = self.img.shape[:2]
        
        # undistort
        mapx, mapy = cv2.initUndistortRectifyMap(self.mtx, self.distort, None, self.newcameramtx, (w,h), 5)
        dstort = cv2.remap(self.img, mapx, mapy, cv2.INTER_LINEAR)
        
        # crop the image
        x, y, roi_w, roi_h = self.roi
        dst = dstort[y:y+roi_h, x:x+roi_w]
        
        cv2.imshow('Remapping', dst)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()

    # ------------------------------------------------------------ #
    def projection_Error(self):
        mean_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints[i], self.rvecs[i], self.tvecs[i], self.mtx, self.distort)
            
            # Padroniza os dois arrays para o formato exato que o OpenCV exige
            pontos_reais = self.imgpoints[i].reshape(-1, 2).astype(np.float32)
            pontos_projetados = imgpoints2.reshape(-1, 2).astype(np.float32)
            
            # Agora a comparação (norm) funciona perfeitamente
            error = cv2.norm(pontos_reais, pontos_projetados, cv2.NORM_L2) / len(pontos_projetados)
            mean_error += error
            
        print("Erro médio total (quanto menor, melhor): {}".format(mean_error/len(self.objpoints)))


# ================ Camera Calibration =============

calib_obj = Calibration()
calib_obj.capture_chess_image() # Corrigido o nome da chamada da função

if len(calib_obj.objpoints) > 0:
    print("Calibrando imagem...")
    calib_obj.calibrate()
    calib_obj.undistort()
    calib_obj.remapping()
    calib_obj.projection_Error()
    print("Calibragem finalizada :)")
else:
    print("Nenhuma imagem capturada. Calibração cancelada.")
