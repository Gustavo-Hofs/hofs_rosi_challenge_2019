#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import NavSatFix
import numpy as np
from geometry_msgs.msg import TwistStamped
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg

class HofsMain():

    # Inicia variaveis de controle do robo
    tempo = None
    t0 = None
    leitura_gps = None
    ultimo_t = None
    pose = (0.57, -0.05, np.pi)
    velocidades = np.array([0, 0])
    leitura_forca_ponteira = 0
    forca_maxima_ponteira = 0
    vendo_fogo = False

    coord_inicio_A = (-1.6131, 1.9000)
    coord_frente_1_rolo_A = (-5.3682, 1.9192)
    coord_inicio_coleta_lado_A = (-6.8000, 1.8435)
    coord_inicio_subida_A = (-35.3000, 1.8435)
    coord_frente_escada_A = (-41.4460, 1.8435)
    coord_fogo_A = (-46.2500, 1.8435)
    coord_fim_suspensa = (-51.0000, 1.8435)
    coord_volta_por_fora0 = (-42.000, 4.000)
    coord_volta_por_fora1 = (-52.600, 2.6750)
    coord_volta_por_fora2 = (-52.600, -2.8750)
    coord_volta_por_fora3 = (-42.000, -4.000)
    coord_frente_escada_B = (-41.2500, -1.9000)
    coord_inicio_area_restrita = (-38.3000, -1.9000)
    coord_fim_descida_B = (-34.725, -1.9000)
    coord_fim_area_restrita = (-27.6000, -1.9000)
    coord_inicio_obstaculo_menor = (-23.0000, -1.9000)
    coord_meio_obstaculo_menor = (-21.5250, -2.5250)
    coord_fim_obstaculo_menor = (-19.5000, -2.0000)
    coord_inicio_obstaculo_maior = (-16.5000, -2.0000)
    coord_meio_obstaculo_maior = (-13.6500, -3.0500)
    coord_fim_obstaculo_maior = (-10.0000, -2.0000)
    coord_frente_2_rolo_B = (-5.7500, -2.0000)
    coord_frente_1_rolo_B = (-4.8500, -2.0000)

    checkpoints_coords = [
        coord_inicio_A,
        coord_frente_1_rolo_A,
        coord_inicio_coleta_lado_A,
        coord_inicio_subida_A,
        coord_frente_escada_A,
        coord_fogo_A,
        coord_fim_suspensa,
        coord_volta_por_fora0,
        coord_volta_por_fora1,
        coord_volta_por_fora2,
        coord_volta_por_fora3,
        coord_frente_escada_B,
        coord_inicio_area_restrita,
        coord_fim_descida_B,
        coord_fim_area_restrita,
        coord_inicio_obstaculo_menor,
        coord_meio_obstaculo_menor,
        coord_fim_obstaculo_menor,
        coord_inicio_obstaculo_maior,
        coord_meio_obstaculo_maior,
        coord_fim_obstaculo_maior,
        coord_frente_2_rolo_B,
        coord_frente_1_rolo_B,
    ]

    checkpoints_flags = [0]*len(checkpoints_coords)
    checkpoint_atual = 0

    numero_de_tarefas = len(checkpoints_coords)
    tarefas_flags = [0]*numero_de_tarefas
    tempo_inicio_tarefas = [None]*numero_de_tarefas

    lista_anomalias = []
    tempo_inicio_tratamento_anomalias = []
    numero_de_anomalias_nao_tratadas = 0

    rospack = rospkg.RosPack()
    rospack.list()
    mapa_path = str(rospack.get_path('hofs_rosi_challenge_2019_oficial_02')) + '/outputs/Mapa das ocorrencias de fogo.txt'

    def __init__(self):
        rospy.loginfo('HofsMain node started')
        node_sleep_rate = rospy.Rate(10)

        rospy.Subscriber("/simulation/time", Float32, self.pega_tempo)
        rospy.Subscriber("/sensor/gps", NavSatFix, self.pega_gps)
        rospy.Subscriber("/ur5/forceTorqueSensorOutput", TwistStamped, self.pega_forca_ponteira)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/sensor/ur5toolCam",Image,self.ve_fogo)

        self.tarefas = [self.tarefa_faz_nada]*len(self.checkpoints_coords)
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_A)] = self.tarefa_olha_pra_cima
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_coleta_lado_A)] = self.tarefa_toca_cavalete_1
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_subida_A)] = self.tarefa_coleta_sem_subida_A
        self.tarefas[self.checkpoints_coords.index(self.coord_frente_escada_A)] = self.tarefa_coleta_com_subida
        self.tarefas[self.checkpoints_coords.index(self.coord_fogo_A)] = self.tarefa_sobe_escada
        self.tarefas[self.checkpoints_coords.index(self.coord_fim_suspensa)] = self.tarefa_toca_cavalete_alto
        self.tarefas[self.checkpoints_coords.index(self.coord_volta_por_fora0)] = self.tarefa_desce_escada_com_raiva
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_area_restrita)] = self.tarefa_vira_e_coleta_com_descida
        self.tarefas[self.checkpoints_coords.index(self.coord_fim_descida_B)] = self.tarefa_coleta_com_descida
        self.tarefas[self.checkpoints_coords.index(self.coord_fim_area_restrita)] = self.tarefa_coleta_com_descida_sutil_em_B
        self.tarefas[self.checkpoints_coords.index(self.coord_meio_obstaculo_menor)] = self.tarefa_coleta_indo_obstaculo_menor
        self.tarefas[self.checkpoints_coords.index(self.coord_fim_obstaculo_menor)] = self.tarefa_coleta_vindo_obstaculo_menor
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_obstaculo_maior)] = self.tarefa_coleta_sem_subida_B
        self.tarefas[self.checkpoints_coords.index(self.coord_meio_obstaculo_maior)] = self.tarefa_coleta_indo_obstaculo_menor
        self.tarefas[self.checkpoints_coords.index(self.coord_fim_obstaculo_maior)] = self.tarefa_coleta_vindo_obstaculo_menor
        self.tarefas[self.checkpoints_coords.index(self.coord_frente_2_rolo_B)] = self.tarefa_coleta_sem_subida_B
        self.tarefas[self.checkpoints_coords.index(self.coord_frente_1_rolo_B)] = self.tarefa_toca_cavalete_baixo

        f = open(self.mapa_path, 'w')
        f.close()

        while not rospy.is_shutdown():

            if self.tempo==None: continue
            if self.t0==None:
                self.t0 = self.tempo
                self.ultimo_t = self.tempo

            tempo_decorrido = self.tempo - self.t0
            delta_t, self.ultimo_t = tempo_decorrido - self.ultimo_t, tempo_decorrido
            if delta_t<=0: continue
            print '---'
            print 'Tempo:',tempo_decorrido
            print 'GPS:',self.leitura_gps
            print 'Dt:',delta_t
            self.pose = self.improvisa_com_o_gps()
            print 'Pose:', self.pose
            print 'Forca ponteira:', self.leitura_forca_ponteira, "| Forca max:", self.forca_maxima_ponteira
            print self.checkpoint_atual
            print self.lista_anomalias

            if self.checkpoint_atual >= len(self.checkpoints_coords):
                self.envia_comando_tracao(0, 0)
                self.envia_comando_bracos(0,0)
                self.envia_comando_manipulador([0, 0, 0, 0, 0, 0])
                break
            self.vai_pra(self.checkpoints_coords[self.checkpoint_atual])
            self.tarefas[self.checkpoint_atual](tempo_decorrido, delta_t)
            self.verifica_se_chegou(self.checkpoint_atual)

            node_sleep_rate.sleep()

    def pega_tempo(self,data):
        self.tempo = data.data

    def envia_comando_tracao(self,v,w):
        pub = rospy.Publisher('/controlador_tracao', Float32MultiArray, queue_size=1)
        comando = Float32MultiArray()
        comando.data = [v,w]
        pub.publish(comando)

    def envia_comando_bracos(self,f,t):
        pub = rospy.Publisher('/controlador_bracos', Float32MultiArray, queue_size=1)
        comando = Float32MultiArray()
        comando.data = [f,t]
        pub.publish(comando)

    def envia_comando_manipulador(self,vetor_com_angulos_das_juntas):
        pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=1)
        comando = ManipulatorJoints()
        comando.joint_variable = vetor_com_angulos_das_juntas
        pub.publish(comando)

    def pega_gps(self,data):
        self.leitura_gps = (data.latitude, data.longitude)

    @staticmethod
    def ajeita_angulo(teta):
        if teta > np.pi: teta = -2*np.pi + teta
        if teta <= -np.pi: teta = 2*np.pi + teta
        return teta

    def improvisa_com_o_gps(self):
        x, y = self.leitura_gps
        if (y-self.pose[1])**2 + (x-self.pose[0])**2 > .333**2/1000:
            teta = np.arctan2([y-self.pose[1]], [x-self.pose[0]])
        else:
            teta = np.array([self.pose[2]])
        return (x, y, teta[0])

    def vai_pra(self, coord):
        x, y, teta = self.pose
        v = .06
        angulo_relativo = self.ajeita_angulo(np.arctan2([coord[1]-y], [coord[0]-x]) - teta)
        wmax = .1
        w = angulo_relativo[0] * .5
        if w > wmax: w = wmax
        elif w < -wmax: w = -wmax
        self.envia_comando_tracao(v, w)

    def verifica_se_chegou(self, ind_check):
        d_limite = .1
        d = ((self.checkpoints_coords[ind_check][0] - self.pose[0])**2 + (self.checkpoints_coords[ind_check][1] - self.pose[1])**2)**.5
        if d<d_limite: self.checkpoint_atual += 1

    def pega_forca_ponteira(self, data):
        self.leitura_forca_ponteira = data.twist.linear.z
        if self.leitura_forca_ponteira > self.forca_maxima_ponteira: self.forca_maxima_ponteira = self.leitura_forca_ponteira

    def tarefa_faz_nada(self, tempo_decorrido, dt):
        pass
    def tarefa_teste(self, tempo_decorrido, dt):
        if self.tarefas_flags[self.checkpoint_atual] == 0:
            if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
                self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
            self.envia_comando_tracao(0, 0)
            self.envia_comando_manipulador([0, -1.3, 2.6, -1.3, 1.57, 0])
            if tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual] > 2:
                self.envia_comando_manipulador([0, 0, 0, 0, 0, 0])
                self.tarefas_flags[self.checkpoint_atual] = 1
    def tarefa_olha_pra_cima(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([0, 0, 0, -np.pi/2, -np.pi/2, 0])
    def tarefa_toca_cavalete_1(self, tempo_decorrido, dt):
        if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
            self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
        tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual]
        if tempo_decorrido_do_inicio_da_tarefa <= 3:
            self.envia_comando_tracao(0, 0)
            comando = np.array([1.3, 0.2, 1.4, -1.6, -1.5708, 0]) * tempo_decorrido_do_inicio_da_tarefa/3
            self.envia_comando_manipulador(list(comando))
        elif self.tarefas_flags[self.checkpoint_atual] == 0:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_manipulador([1.3, 0.2 + (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, 1.4, -1.6, -1.5708, 0])
            if self.leitura_forca_ponteira > 0.5:
                self.tarefas_flags[self.checkpoint_atual] = 1
        else:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_manipulador([np.pi/2, 0, 0, 0, -np.pi/2, 0])
            self.tempo_inicio_tarefas[self.checkpoint_atual] = None
            self.tarefas_flags[self.checkpoint_atual] = 0
            self.tarefas[self.checkpoints_coords.index(self.coord_inicio_coleta_lado_A)] = self.tarefa_toca_eixo_1
    def tarefa_toca_eixo_1(self, tempo_decorrido, dt):
        if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
            self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
        tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual]
        if tempo_decorrido_do_inicio_da_tarefa <= 3:
            self.envia_comando_tracao(0, 0)
            comando = np.array([1.3, 0.6, 0, -0.5, -1.5708, 0]) * tempo_decorrido_do_inicio_da_tarefa/3
            self.envia_comando_manipulador(list(comando))
        elif self.tarefas_flags[self.checkpoint_atual] == 0:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_manipulador([1.3, 0.6 + (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, 0, -0.5 - (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, -1.5708, 0])
            if self.leitura_forca_ponteira > 0.5:
                self.tarefas_flags[self.checkpoint_atual] = 1
        else:
            self.envia_comando_manipulador([1.3, 0.6, -1.4, -0.5, -1.5708, 0])
    def tarefa_coleta_sem_subida_A(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([np.pi/2, 0, 0, -0.1, -np.pi/2, 0])
    def tarefa_coleta_com_subida(self, tempo_decorrido, dt):
        angulo = -0.15+(-0.55+0.15) * (self.pose[0] - self.coord_inicio_subida_A[0]) / (self.coord_frente_escada_A[0] - self.coord_inicio_subida_A[0])
        self.envia_comando_manipulador([np.pi/2, 0, 0, angulo, -np.pi/2, 0])
    def tarefa_sobe_escada(self, tempo_decorrido, dt):
        if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
            self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
        tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual]
        if tempo_decorrido_do_inicio_da_tarefa < 2:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_bracos(-0.5, 0.5)
        elif tempo_decorrido_do_inicio_da_tarefa < 4:
            self.envia_comando_tracao(0.06, 0)
            self.envia_comando_bracos(0.5, 0)
            self.envia_comando_manipulador([np.pi/2, 0, 0, -0.35, -np.pi/2, 0])
        elif tempo_decorrido_do_inicio_da_tarefa < 8:
            self.envia_comando_tracao(0.06, 0)
            self.envia_comando_bracos(0.5, -0.5)
        elif tempo_decorrido_do_inicio_da_tarefa < 10:
            self.envia_comando_tracao(0.06, 0)
            self.envia_comando_bracos(-0.5, 0)
        elif tempo_decorrido_do_inicio_da_tarefa < 14:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_bracos(-0.5, 0)
        elif tempo_decorrido_do_inicio_da_tarefa < 16:
            self.envia_comando_tracao(0.06, 0)
            self.envia_comando_bracos(-0.5, -0.5)
        elif tempo_decorrido_do_inicio_da_tarefa < 17.5:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_bracos(0.5, 0.5)
        elif tempo_decorrido_do_inicio_da_tarefa < 23:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_bracos(0.5, 0)
        else:
            self.envia_comando_bracos(0, 0)
    def tarefa_toca_cavalete_alto(self, tempo_decorrido, dt):
        if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
            self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
        tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual]
        if tempo_decorrido_do_inicio_da_tarefa <= 3:
            self.envia_comando_tracao(0, 0)
            comando = np.array([0, 0.6, 0, -0.6, 0, 0]) * tempo_decorrido_do_inicio_da_tarefa/3 + np.array([1.5708, 0, 0, 0, -1.5708, 0])
            self.envia_comando_manipulador(list(comando))
        elif self.tarefas_flags[self.checkpoint_atual] == 0:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_manipulador([1.5708, 0.6 + (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, 0, -0.6 - (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, -1.5708, 0])
            if self.leitura_forca_ponteira > 0.5:
                self.tarefas_flags[self.checkpoint_atual] = 1
        else:
            self.envia_comando_manipulador([np.pi/2, 0, 0, -0.35, -np.pi/2, 0])
    def tarefa_desce_escada_com_raiva(self, tempo_decorrido, dt):
        if self.tarefas_flags[self.checkpoint_atual] == 0:
            self.envia_comando_tracao(-0.2, 0)
            self.envia_comando_manipulador([np.pi/2, 0, 0, -0.35, -np.pi/2, 0])
        if self.pose[0] > -41:
            self.tarefas_flags[self.checkpoint_atual] = 1
            self.envia_comando_manipulador([np.pi/2, 0, 0, -1.5708, -np.pi/2, 0])
    def tarefa_vira_e_coleta_com_descida(self, tempo_decorrido, dt):
        if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
            self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
        tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual]
        if tempo_decorrido_do_inicio_da_tarefa < 3:
            self.envia_comando_tracao(0, -0.1)
        elif tempo_decorrido_do_inicio_da_tarefa < 3.5:
            self.envia_comando_tracao(0.06, 0)
        else:
            angulo = -0.55+(-0.15+0.55) * (self.pose[0] - self.coord_frente_escada_B[0]) / (self.coord_fim_descida_B[0] - self.coord_frente_escada_B[0])
            self.envia_comando_manipulador([np.pi/2, 0, 0, angulo, -np.pi/2, 0])
    def tarefa_coleta_com_descida(self, tempo_decorrido, dt):
        angulo = -0.55+(-0.15+0.55) * (self.pose[0] - self.coord_frente_escada_B[0]) / (self.coord_fim_descida_B[0] - self.coord_frente_escada_B[0])
        self.envia_comando_manipulador([np.pi/2, 0, 0, angulo, -np.pi/2, 0])
    def tarefa_coleta_com_descida_sutil_em_B(self, tempo_decorrido, dt):
        angulo = -0.15+(0+0.15) * (self.pose[0] - self.coord_fim_descida_B[0]) / (self.coord_fim_area_restrita[0] - self.coord_fim_descida_B[0])
        self.envia_comando_manipulador([np.pi/2, 0, 0, angulo, -np.pi/2, 0])
    def tarefa_coleta_indo_obstaculo_menor(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([2, 0, 0, 0, -np.pi/2, 0])
    def tarefa_coleta_vindo_obstaculo_menor(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([np.pi/2, 0, 0, 0, -np.pi/2, 0])
    def tarefa_coleta_sem_subida_B(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([np.pi/2, 0, 0, -0.05, -np.pi/2, 0])
    def tarefa_toca_cavalete_baixo(self, tempo_decorrido, dt):
        if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
            self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
        tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual]
        if tempo_decorrido_do_inicio_da_tarefa <= 3:
            self.envia_comando_tracao(0, 0)
            comando = np.array([0, -0.1, 1.5708, -1.5708, 0, 0]) * tempo_decorrido_do_inicio_da_tarefa/3 + np.array([1.55, 0, 0, 0, -1.5708, 0])
            self.envia_comando_manipulador(list(comando))
        elif self.tarefas_flags[self.checkpoint_atual] == 0:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_manipulador([1.55, -0.1 + (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, 1.5708, -1.5708, -1.5708, 0])
            if self.leitura_forca_ponteira > 0.5:
                self.tarefas_flags[self.checkpoint_atual] = 1
        else:
            self.envia_comando_manipulador([np.pi/2, 0, 0, 0, -np.pi/2, 0])
    def tarefa__calibra_camera(self, tempo_decorrido, dt):
        self.envia_comando_tracao(0, 0)
        self.envia_comando_manipulador([0, 0, 0, 0.3, -np.pi/2, 0])

    def verifica_se_ja_foi_anotada(self):
        ja_anotado = False
        for anomalia in self.lista_anomalias:
            x, y, th = anomalia[0]
            d = ((x - self.pose[0])**2 + (y - self.pose[1])**2)**.5
            if d < 2: ja_anotado = True
        return  ja_anotado

    def ve_fogo(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame = cv_image
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # define range of blue color in HSV
        lower_blue = np.array([23,187,179])
        upper_blue = np.array([41,255,255])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # cv2.imshow('frame',frame)
        # cv2.imshow('mask',mask)
        # cv2.waitKey(3)
        _,contours,hierarchy = cv2.findContours(mask, 1, 2)
        # print contours != []
        # self.vendo_fogo = contours != []
        # self.vendo_fogo = False
        if contours != []:
            cnt = contours[0]
            x,y,w,h = cv2.boundingRect(cnt)
            # print w*h
            if w*h > 25000:
                # self.vendo_fogo = True
                if not self.verifica_se_ja_foi_anotada():
                    self.lista_anomalias.append((self.pose, self.tempo))
                    f = open(self.mapa_path, 'a')
                    f.write('Fogo identificado em latitude ')
                    f.write(str(self.pose[0]))
                    f.write(' longitude ')
                    f.write(str(self.pose[1]))
                    f.write(' no tempo ')
                    f.write(str(self.tempo))
                    f.write(' s\n')
                    f.close()


if __name__ == '__main__':
	rospy.init_node('HofsMain')
	try:
		node_obj = HofsMain()
	except rospy.ROSInterruptException: pass
