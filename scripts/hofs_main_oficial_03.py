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

# Se estiver usando o editor Atom, o comando Ctrl + Alt + Shift + [ pode ser util para visualizar melhor

class HofsMain():

    # Inicia variaveis de controle do robo
    tempo = None
    t0 = None
    leitura_gps = None
    ultimo_t = None
    pose = (0.57, -0.05, np.pi)
    leitura_forca_ponteira = 0

    # Inicia as coordenadas desejadas (x, y)
    coord_inicio_inicio = (0.5700, -0.0500)
    coord_inicio2 = (2.0000, -4.0000)
    coord_inicio_A = (-1.6131, 1.9000)
    coord_frente_1_rolo_A = (-5.3682, 1.9192)
    coord_inicio_coleta_lado_A = (-6.8000, 1.8435)
    coord_inicio_subida_A = (-35.3000, 1.8435)
    coord_frente_escada_A = (-41.4460, 1.8435)
    coord_costas_escada_A = (-44.0000, 1.8435)
    coord_fim_suspensa = (-51.0000, 1.8435)
    coord_volta_por_fora0 = (-42.000, 4.000)
    coord_volta_por_fora1 = (-52.600, 2.6750)
    coord_volta_por_fora2 = (-52.600, -2.8750)
    coord_volta_por_fora3 = (-42.000, -4.000)
    coord_frente_escada_B = (-41.2500, -1.9000)
    coord_inicio_area_restrita = (-38.3000, -1.9000)
    coord_fim_descida_B = (-34.725, -1.9000)
    coord_frente_1_rolo_B = (-4.8500, -2.0000)

    # Cria lista das coordenadas
    checkpoints_coords = [
        coord_inicio_inicio,
        coord_inicio2,
        coord_inicio_A,
        coord_frente_1_rolo_A,
        coord_inicio_coleta_lado_A,
        coord_inicio_subida_A,
        coord_frente_escada_A,
        coord_costas_escada_A,
        coord_fim_suspensa,
        coord_volta_por_fora0,
        coord_volta_por_fora1,
        coord_volta_por_fora2,
        coord_volta_por_fora3,
        coord_frente_escada_B,
        coord_inicio_area_restrita,
        coord_fim_descida_B,
        coord_frente_1_rolo_B,
    ]

    # Em qual parte da lista esta agora
    checkpoint_atual = 0

    # Cria lista de tarefas
    numero_de_tarefas = len(checkpoints_coords)

    # Cria lista de tarefas concluidas
    tarefas_flags = [0]*numero_de_tarefas

    # Cria lista com tempo de inicio de cada tarefa
    tempo_inicio_tarefas = [None]*numero_de_tarefas

    # Cria lista onde vai ser armazenado informacoes sobre chamas detectadas
    lista_chamas_detectadas = []

    # Cria lista onde vai ser armazenado informacoes sobre obstaculos detectados
    lista_obstaculos_detectados = []

    # O Mapa das ocorrencias de fogo eh um arquivo txt localizado na pasta 'outputs' deste pacote
    # Para acressar-lo, eh necessario saber o path do arquivo e o nome dele
    # E eh isso que essa proximas 3 linhas de comando estao fazendo, gravando o path na variavel mapa_path
    rospack = rospkg.RosPack()
    rospack.list()
    mapa_path = str(rospack.get_path('hofs_rosi_challenge_2019_oficial_03')) + '/outputs/Mapa das ocorrencias de fogo.txt'

    # Cria valiavel para armazenar a tarefa interrompidada pra tocar no cavalete em chamas
    tarefa_interrompidada_pra_tocar_no_cavalete_em_chamas = None

    # Incia variavel que vai dizer o quanto que o robo esta alinhado com o fogo para tocar no cavalete
    distancia_do_fogo_ao_centro_em_pixels = None

    # Inicia a Classe
    def __init__(self):

        # Avisa que vai comecar
        rospy.loginfo('HofsMain node started')

        # Seta a taxa de repeticao
        node_sleep_rate = rospy.Rate(10)

        # Comeca a escutar varios topicos
        rospy.Subscriber("/simulation/time", Float32, self.pega_tempo)
        rospy.Subscriber("/sensor/gps", NavSatFix, self.pega_gps)
        rospy.Subscriber("/ur5/forceTorqueSensorOutput", TwistStamped, self.pega_forca_ponteira)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/sensor/ur5toolCam",Image,self.ve_fogo)
        rospy.Subscriber("/sensor/kinect_depth", Image, self.kinect)

        # Preenche a lista de tarefas com as tarefas de cada Checkpoint
        self.tarefas = [self.tarefa_faz_nada]*len(self.checkpoints_coords)
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_inicio)] = self.tarefa_olha_pra_cima
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_A)] = self.tarefa_olha_pra_cima
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_coleta_lado_A)] = self.tarefa_toca_cavalete_1
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_subida_A)] = self.tarefa_coleta_sem_subida_A
        self.tarefas[self.checkpoints_coords.index(self.coord_frente_escada_A)] = self.tarefa_coleta_com_subida
        self.tarefas[self.checkpoints_coords.index(self.coord_costas_escada_A)] = self.tarefa_sobe_escada
        self.tarefas[self.checkpoints_coords.index(self.coord_fim_suspensa)] = self.tarefa_coleta_alto
        self.tarefas[self.checkpoints_coords.index(self.coord_volta_por_fora0)] = self.tarefa_desce_escada_com_raiva
        self.tarefas[self.checkpoints_coords.index(self.coord_inicio_area_restrita)] = self.tarefa_vira_e_coleta_com_descida
        self.tarefas[self.checkpoints_coords.index(self.coord_fim_descida_B)] = self.tarefa_coleta_com_descida
        self.tarefas[self.checkpoints_coords.index(self.coord_frente_1_rolo_B)] = self.tarefa_coleta_sem_subida_B

        # Limpa o mapa
        f = open(self.mapa_path, 'w')
        f.close()

        # Loop principal
        while not rospy.is_shutdown():

            # Se ainda nao tiver comunicado com o tempo de simulacao, ignora o resto e tenta de novo
            if self.tempo==None: continue

            # Se ainda nao armazenou o tempo zero do inicio deste node junto com a simulacao
            if self.t0==None:

                # Tempo zero vai ser o momento atual
                self.t0 = self.tempo

                # Inicia o ultimo tempo como sendo o momento atual tambem
                self.ultimo_t = self.tempo

            # Calcula o tempo decorrido desde o inicio do node junto com a simulacao
            tempo_decorrido = self.tempo - self.t0

            # Calcula o tempo decorrido desde a ultima interacao do loop principal
            delta_t, self.ultimo_t = tempo_decorrido - self.ultimo_t, tempo_decorrido

            # Se nao passou tempo desde a ultima interacao do loop principal, ignora o resto e passa pra proxima interacao
            if delta_t<=0: continue

            # Atualiza posicao do robo
            self.pose = self.improvisa_com_o_gps()

            # Printa algumas informacoes uteis para o desenvolvedor
            print '---'
            print 'Tempo:',tempo_decorrido
            print 'GPS:',self.leitura_gps
            print 'Dt:',delta_t
            print 'Pose:', self.pose
            print 'Forca ponteira:', self.leitura_forca_ponteira
            print 'Checkpoint atual:', self.checkpoint_atual
            print 'Lista chamas', self.lista_chamas_detectadas
            print 'Lista obstaculos', self.lista_obstaculos_detectados
            print 'Centralidade do fogo:', self.distancia_do_fogo_ao_centro_em_pixels

            # Se ja passou por todos checkpoints da lista
            if self.checkpoint_atual >= len(self.checkpoints_coords):

                # Para o robo completamente
                self.envia_comando_tracao(0, 0)
                self.envia_comando_bracos(0,0)
                self.envia_comando_manipulador([0, 0, 0, 0, 0, 0])

                # Encerra node
                break

            # Move robo em direcao ao checkpoint da lista
            self.vai_pra(self.checkpoints_coords[self.checkpoint_atual])

            # Ao mesmo tempo, executa tarefa do checkpoint
            # OBS: os comandos das tarefas importam mais do que os comandos de movimento.
            # Logo, se a tarefa mandar o robo parar, ele vai parar.
            self.tarefas[self.checkpoint_atual](tempo_decorrido, delta_t)

            # Verifica se o robo chegou no destino atual
            self.verifica_se_chegou(self.checkpoint_atual)

            # Dorme um pouco
            node_sleep_rate.sleep()

    # Funcao para atualizar o tempo da simulacao
    def pega_tempo(self,data):
        '''
        Recebe data do topico do tempo de simulacao
        Armazena o tempo de simulacao na variavel 'tempo'
        '''
        self.tempo = data.data

    # Funcao para controlar a tracao das rodas do robo
    def envia_comando_tracao(self,v,w):
        '''
        Recebe velocidade linear 'v' e velocidade angular 'w'
        Publica isso no topico '/controlador_tracao'
        '''

        # Registra o publisher
        pub = rospy.Publisher('/controlador_tracao', Float32MultiArray, queue_size=1)

        # Ajeita as informacoes recebidas para o formato de msg do topico
        comando = Float32MultiArray()
        comando.data = [v,w]

        # Publica
        pub.publish(comando)

    # Funcao para controlar os bracos do robo
    def envia_comando_bracos(self,f,t):
        '''
        Recebe a velocidade de rotacao dos dois bracos frontais 'f' e dos bracos traseiros 't'
        Publica isso no topico '/controlador_bracos'
        '''

        # Registra o publisher
        pub = rospy.Publisher('/controlador_bracos', Float32MultiArray, queue_size=1)

        # Ajeita informacoes recebidas para o formato de msg do topico
        comando = Float32MultiArray()
        comando.data = [f,t]

        # Publica
        pub.publish(comando)

    # Funcao para controlar o manipulador
    def envia_comando_manipulador(self,vetor_com_angulos_das_juntas):
        '''
        Recebe vetor contendo os angulos de todas as juntas do manipulador
        Publica isso no topico '/ur5/jointsPosTargetCommand'
        '''

        # Registra o publisher
        pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=1)

        # Ajeita informacoes recebidas para o formato de msg do topico
        comando = ManipulatorJoints()
        comando.joint_variable = vetor_com_angulos_das_juntas

        # Publica
        pub.publish(comando)

    # Funcao para atualizar leitura do GPS
    def pega_gps(self,data):
        '''
        Recebe data do topico do sensor GPS
        Armazena na variavel 'leitura_gps'
        '''
        self.leitura_gps = (data.latitude, data.longitude)

    # Funcao para limitar valor de angulo entre pi e -pi
    @staticmethod
    def ajeita_angulo(teta):
        '''
        Recebe angulo 'teta'
        Retorna angulo 'teta' limitado entre pi e -pi
        '''
        if teta > np.pi: teta = -2*np.pi + teta
        if teta <= -np.pi: teta = 2*np.pi + teta
        return teta

    # Funcao para atualizar a posicao do robo usando apenas o GPS
    def improvisa_com_o_gps(self):
        '''
        Retorna a posicao x, y e orientacao do robo de acordo com a leitura do GPS e a ultima posicao do robo
        '''

        # Pega a leitura do GPS
        x, y = self.leitura_gps

        # Se distancia deslocada pelo robo for grande suficiente
        if (y-self.pose[1])**2 + (x-self.pose[0])**2 > .333**2/1000:
            # Estima orientacao atual
            teta = np.arctan2([y-self.pose[1]], [x-self.pose[0]])
        # Caso contrario
        else:
            # Nao atualiza orientecao
            teta = np.array([self.pose[2]])

        # Retorna nova posicao
        return (x, y, teta[0])

    # Funcao que controla o movimento do robo
    def vai_pra(self, coord):
        '''
        Recebe as coordenadas 'coord' do checkpoint atual
        Mantem a velocidade linear fixa e usa um simples controlador proporcional para a velocidade angular
        '''

        # Pega x, y e orientacao atual do robo
        x, y, teta = self.pose

        # Seta velocidade linear
        v = .06

        # Calcula angulo relativo do robo com o objetivo
        angulo_relativo = self.ajeita_angulo(np.arctan2([coord[1]-y], [coord[0]-x]) - teta)

        # Seta velocidade angular maxima
        wmax = .1

        # Aplica o controle proporcional
        w = angulo_relativo[0] * .5

        # Limita na velocidade angular maxima
        if w > wmax: w = wmax
        elif w < -wmax: w = -wmax

        # Envia o comando de tracao
        self.envia_comando_tracao(v, w)

    # Funcao que controla o checkpoint atual
    def verifica_se_chegou(self, ind_check):
        '''
        Receve o valor do checkpoint atual
        Se o robo chegou no destino, vai para o proximo checkpoint
        '''

        # Seta distancia limite
        d_limite = .1

        # Calcula distancia do robo ao destido
        d = ((self.checkpoints_coords[ind_check][0] - self.pose[0])**2 + (self.checkpoints_coords[ind_check][1] - self.pose[1])**2)**.5

        # Se distancia do robo ao destino for menor que distancia limite, entao chego. Vai para o proximo
        if d<d_limite: self.checkpoint_atual += 1

    # Funcao para atualizar leitura do GPS
    def pega_forca_ponteira(self, data):
        '''
        Recebe data do topico do sensor de forca na ponteira
        Armazena na variavel 'leitura_forca_ponteira'
        '''
        self.leitura_forca_ponteira = data.twist.linear.z

    # Funcoes de tarefas
    # Cada uma dessas funcoes execuma uma sequencia de acoes adequadas para concluir o objetivo descrito no nome da tarefa
    def tarefa_faz_nada(self, tempo_decorrido, dt):
        pass
    def tarefa_olha_pra_cima(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([0, 0, 0, -np.pi/2, -np.pi/2, 0])
    def tarefa_toca_cavalete_1(self, tempo_decorrido, dt):
        if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
            self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
        tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual]
        if tempo_decorrido_do_inicio_da_tarefa <= 3:
            self.envia_comando_tracao(0, 0)
            comando = np.array([1.25, 0.2, 1.4, -1.6, -1.5708, 0]) * tempo_decorrido_do_inicio_da_tarefa/3
            self.envia_comando_manipulador(list(comando))
        elif self.tarefas_flags[self.checkpoint_atual] == 0:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_manipulador([1.25, 0.2 + (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, 1.4, -1.6, -1.5708, 0])
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
            comando = np.array([1.25, 0.6, 0, -0.5, -1.5708, 0]) * tempo_decorrido_do_inicio_da_tarefa/3
            self.envia_comando_manipulador(list(comando))
        elif self.tarefas_flags[self.checkpoint_atual] == 0:
            self.envia_comando_tracao(0, 0)
            self.envia_comando_manipulador([1.25, 0.6 + (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, 0, -0.5 - (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, -1.5708, 0])
            if self.leitura_forca_ponteira > 0.5:
                self.tarefas_flags[self.checkpoint_atual] = 1
        else:
            self.envia_comando_manipulador([1.25, 0.6, -1.4, 1, -1.5708, 0])
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
    def tarefa_coleta_alto(self, tempo_decorrido, dt):
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
        if tempo_decorrido_do_inicio_da_tarefa < 2.75:
            self.envia_comando_tracao(0, -0.1)
        elif tempo_decorrido_do_inicio_da_tarefa < 3.25:
            self.envia_comando_tracao(0.06, 0)
        else:
            angulo = -0.55+(-0.15+0.55) * (self.pose[0] - self.coord_frente_escada_B[0]) / (self.coord_fim_descida_B[0] - self.coord_frente_escada_B[0])
            self.envia_comando_manipulador([np.pi/2, 0, 0, angulo, -np.pi/2, 0])
    def tarefa_coleta_com_descida(self, tempo_decorrido, dt):
        angulo = -0.55+(-0.15+0.55) * (self.pose[0] - self.coord_frente_escada_B[0]) / (self.coord_fim_descida_B[0] - self.coord_frente_escada_B[0])
        self.envia_comando_manipulador([np.pi/2, 0, 0, angulo, -np.pi/2, 0])
    def tarefa_coleta_indo_obstaculo_menor(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([2, 0, 0, 0, -np.pi/2, 0])
    def tarefa_coleta_vindo_obstaculo_menor(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([np.pi/2, 0, 0, 0, -np.pi/2, 0])
    def tarefa_coleta_sem_subida_B(self, tempo_decorrido, dt):
        self.envia_comando_manipulador([np.pi/2, 0, 0, -0.05, -np.pi/2, 0])
    def tarefa_toca_cavalete_em_chamas(self, tempo_decorrido, dt):
        if self.tarefas_flags[self.checkpoint_atual] == 0:
            if self.tempo_inicio_tarefas[self.checkpoint_atual] == None:
                self.tempo_inicio_tarefas[self.checkpoint_atual] = tempo_decorrido
            tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[self.checkpoint_atual]
            if tempo_decorrido_do_inicio_da_tarefa < 3:
                self.envia_comando_tracao(0.1 * self.distancia_do_fogo_ao_centro_em_pixels, 0)
            elif self.pose[0] > -35:
                if tempo_decorrido_do_inicio_da_tarefa <= 6:
                    self.envia_comando_tracao(0, 0)
                    comando = np.array([0, -0.1, 1.5708, -1.5708, 0, 0]) * (tempo_decorrido_do_inicio_da_tarefa - 3)/3 + np.array([1.55, 0, 0, 0, -1.5708, 0])
                    self.envia_comando_manipulador(list(comando))
                else:
                    self.envia_comando_tracao(0, 0)
                    self.envia_comando_manipulador([1.55, -0.1 + (tempo_decorrido_do_inicio_da_tarefa - 6) * 0.02, 1.5708, -1.5708, -1.5708, 0])
                    if self.leitura_forca_ponteira > 0.5:
                        self.tarefas_flags[self.checkpoint_atual] = 1
            elif self.pose[0] <= -35:
                if tempo_decorrido_do_inicio_da_tarefa <= 6:
                    self.envia_comando_tracao(0, 0)
                    comando = np.array([0, 0.6, 0, -0.6, 0, 0]) * (tempo_decorrido_do_inicio_da_tarefa - 3)/3 + np.array([1.5708, 0, 0, 0, -1.5708, 0])
                    self.envia_comando_manipulador(list(comando))
                elif self.tarefas_flags[self.checkpoint_atual] == 0:
                    self.envia_comando_tracao(0, 0)
                    self.envia_comando_manipulador([1.5708, 0.6 + (tempo_decorrido_do_inicio_da_tarefa - 6) * 0.02, 0, -0.6 - (tempo_decorrido_do_inicio_da_tarefa - 3) * 0.02, -1.5708, 0])
                    if self.leitura_forca_ponteira > 0.5:
                        self.tarefas_flags[self.checkpoint_atual] = 1
        else:
            self.tempo_inicio_tarefas[self.checkpoint_atual] = 0
            self.tarefas_flags[self.checkpoint_atual] = 0
            self.tarefas[self.checkpoint_atual] = self.tarefa_interrompidada_pra_tocar_no_cavalete_em_chamas

    # Funcao que evita que uma mesma anomalia (como chamas ou obstaculos) seja registrada mais de uma vez
    def verifica_se_ja_foi_anotada(self, lista, distancia_do_ultimo_anotado):
        '''
        Recebe a lista de anomalias a ser verificada 'lista' e o valor da distancia aceitavel entre anomalias 'distancia_do_ultimo_anotado'
        Verifica se a anomalia detectada neste momento ja foi registrada, vendo se esta distante suficiente de todas outras anomalias registradas
        Retorna 'False' se esta nova anomalia esta distante suficiente de todas outras anomalias registradas, e 'True' do contrario
        '''

        # Comeca considerando que ainda nao foi anotada
        ja_anotado = False

        # Para todas anomalias da lista
        for anomalia in lista:

            # Pega coordenadas da anomalia da lsita
            x, y, th = anomalia[0]

            # Calcula distancia da anomalia atual ate a anomalia da lista
            d = ((x - self.pose[0])**2 + (y - self.pose[1])**2)**.5

            # Se distancia for menor que a aceitavel, ja foi anotado
            if d < distancia_do_ultimo_anotado: ja_anotado = True

        # Retorna se ja foi anotado ou nao
        return  ja_anotado

    # Funcao que procura por chamas e toma providencias quando encontra
    def ve_fogo(self,data):
        '''
        Recebe data do topico da camera do manipulador
        Faz todo um tratamento na imagem para identificar fogo
        Caso identifique fogo, registra no mapa e troca a tarefa atual por uma que vai tocar no cavalete em chamas
        '''

        # Tenta extrair a imagem. Se der erro, fala
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame = cv_image
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # define range of yellow color in HSV
        lower_yellow = np.array([23,187,179])
        upper_yellow = np.array([41,255,255])
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # Contorna o que encontrou
        _,contours,hierarchy = cv2.findContours(mask, 1, 2)
        # Se encontrou algo
        if contours != []:
            # Pega informacao do contorno
            cnt = contours[0]
            # Extrai coordenadas, altura e largura
            x,y,w,h = cv2.boundingRect(cnt)
            # Se fogo estiver perto suficiente da camera
            if w*h > 25000:
                # Calcula o meio da tela multiplicando por uma constante e compensacao
                meio_da_tela = mask.shape[1] / 2.0 * .8
                # Calcula o quanto o fogo esta centralizado na tela (util pra posicionar o robo reto pra tocar cavalete)
                self.distancia_do_fogo_ao_centro_em_pixels = (meio_da_tela - x) / (meio_da_tela)
                # Se o fogo ainda nao foi anotado
                if not self.verifica_se_ja_foi_anotada(self.lista_chamas_detectadas, 2):
                    # Registra ocorrencia
                    self.lista_chamas_detectadas.append((self.pose, self.tempo))
                    # Marca no mapa
                    f = open(self.mapa_path, 'a')
                    f.write('Fogo identificado em latitude ')
                    f.write(str(self.pose[0]))
                    f.write(' longitude ')
                    f.write(str(self.pose[1]))
                    f.write(' no tempo ')
                    f.write(str(self.tempo))
                    f.write(' s\n')
                    f.close()
                    # Troca a tarefa atual por uma que vai tocar no cavalete em chamas
                    # Reseta o tempo de inicio
                    self.tempo_inicio_tarefas[self.checkpoint_atual] = None
                    # Reseta a situacao da tarega
                    self.tarefas_flags[self.checkpoint_atual] = 0
                    # Armazena tarefa atual para mais tarde retomar
                    self.tarefa_interrompidada_pra_tocar_no_cavalete_em_chamas = self.tarefas[self.checkpoint_atual]
                    # Substitui tarefa atual
                    self.tarefas[self.checkpoint_atual] = self.tarefa_toca_cavalete_em_chamas

    # Funcao que procura por obstaculos e toma providencias quando encontra
    def kinect(self, data):
        '''
        Recebe data do topico do kinect
        Faz todo um tratamento na imagem para identificar obastaculos
        Caso identifique obastaculo, adciona 2 checkpoints pra dar a volta sem colisao
        '''

        # Tenta extrair a imagem. Se der erro, fala
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)

        # Define coordenadas do pixel a ser observado
        pixel_x, pixel_y = 190, 390
        # Observa pixel
        intencidade = cv_image[pixel_x, pixel_y]
        # Se observado algo no pixel e o robo esta numa posicao correta
        if intencidade < 2000 and abs(self.pose[2]) < 0.02 and self.pose[0] > -35:
            # Se ainda nao foi anotado
            if not self.verifica_se_ja_foi_anotada(self.lista_obstaculos_detectados, 7):
                # Mede tamanho do obstaculo
                for i in range(pixel_y):
                    if cv_image[pixel_x, i] < 2000:
                        tamanho = pixel_y - i
                        break
                # Registra obstaculo
                self.lista_obstaculos_detectados.append((self.pose, tamanho, intencidade))
                # Define tamanho do desvio de acordo com o tamnho do obstaculo
                desvio = -1 if tamanho > 30 else -0.5
                # Adciona os checkpoints na lista
                self.checkpoints_coords = self.checkpoints_coords[:self.checkpoint_atual] + [(self.pose[0] + 4 - desvio, -2)] + self.checkpoints_coords[self.checkpoint_atual:]
                self.checkpoints_coords = self.checkpoints_coords[:self.checkpoint_atual] + [(self.pose[0] + 2, -2 + desvio)] + self.checkpoints_coords[self.checkpoint_atual:]
                # Adciona tarefas de coleta para esses checkpoints
                self.tarefas = self.tarefas[:self.checkpoint_atual] + [self.tarefa_coleta_vindo_obstaculo_menor] + self.tarefas[self.checkpoint_atual:]
                self.tarefas = self.tarefas[:self.checkpoint_atual] + [self.tarefa_coleta_indo_obstaculo_menor] + self.tarefas[self.checkpoint_atual:]
                # Adciona situacao e tempo dessas tarefas
                self.tarefas_flags = self.tarefas_flags[:self.checkpoint_atual] + [0, 0] + self.tarefas_flags[self.checkpoint_atual:]
                self.tempo_inicio_tarefas = self.tempo_inicio_tarefas[:self.checkpoint_atual] + [None, None] + self.tempo_inicio_tarefas[self.checkpoint_atual:]

# Comeca aqui
if __name__ == '__main__':

    # Inicia o node
	rospy.init_node('HofsMain')

    # Tenta iniciar
	try:
		node_obj = HofsMain()
    # Se der alguem problema, nao faz nada
	except rospy.ROSInterruptException: pass
