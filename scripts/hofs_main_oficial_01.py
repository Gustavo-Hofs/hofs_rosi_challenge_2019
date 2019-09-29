#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import NavSatFix
import numpy as np
from filterpy.monte_carlo.resampling import *

class HofsMain():

    tempo = None
    t0 = None
    leitura_gps = None
    ultimo_t = None
    pose = (0.57, -0.05, np.pi)
    velocidades = np.array([0, 0])

    coord_inicio_A = (-1.6131, 1.9000)
    coord_inicio_coleta_lado_A = (-6.5000, 1.8423)
    coord_frente_escada_A = (-41.4460, 1.8435)
    coord_fogo_A = (-46.0587, 1.8076)
    checkpoints_coords = [coord_inicio_A, coord_inicio_coleta_lado_A, coord_frente_escada_A, coord_fogo_A]
    checkpoints_flags = [0]*len(checkpoints_coords)
    checkpoint_atual = 0

    numero_de_tarefas = 2
    tarefas_flags = [0]*numero_de_tarefas
    tempo_inicio_tarefas = [None]*numero_de_tarefas

    def __init__(self):
        rospy.loginfo('HofsMain node started')
        node_sleep_rate = rospy.Rate(10)

        rospy.Subscriber("/simulation/time", Float32, self.pega_tempo)
        rospy.Subscriber("/sensor/gps", NavSatFix, self.pega_gps)


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

            if self.checkpoints_flags[0] == 0:
                self.vai_pra(self.checkpoints_coords[0])
                self.verifica_se_chegou(0)
            elif self.checkpoints_flags[1] == 0:
                self.vai_pra(self.checkpoints_coords[1])
                self.verifica_se_chegou(1)
            elif self.checkpoints_flags[2] == 0:
                self.vai_pra(self.checkpoints_coords[2])
                self.verifica_se_chegou(2)
            elif self.tarefas_flags[0] == 0:
                if self.tempo_inicio_tarefas[0] == None:
                    self.tempo_inicio_tarefas[0]=tempo_decorrido
                tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[0]
                if tempo_decorrido_do_inicio_da_tarefa>=0 and tempo_decorrido_do_inicio_da_tarefa<=2:
                    self.envia_comando_tracao(0, 0)
                    self.envia_comando_bracos(-0.5, 0.5)
                    self.envia_comando_manipulador([0, 0, 0, 0, 0, 0])
                elif tempo_decorrido_do_inicio_da_tarefa>2 and tempo_decorrido_do_inicio_da_tarefa<=4:
                    self.envia_comando_tracao(0.05, 0)
                    self.envia_comando_bracos(0, 0)
                    self.envia_comando_manipulador([0, np.pi/3, 0, 0, 0, 0])
                elif tempo_decorrido_do_inicio_da_tarefa>4 and tempo_decorrido_do_inicio_da_tarefa<=6:
                    self.envia_comando_tracao(0.635, 0)
                    self.envia_comando_bracos(0.5, -0.5)
                elif tempo_decorrido_do_inicio_da_tarefa>6 and tempo_decorrido_do_inicio_da_tarefa<=10:
                    self.envia_comando_bracos(0, -0.5)
                elif tempo_decorrido_do_inicio_da_tarefa>10 and tempo_decorrido_do_inicio_da_tarefa<=11:
                    self.envia_comando_tracao(0.1, 0)
                    self.envia_comando_bracos(-0.5, 0.5)
                else:
                    self.tarefas_flags[0] = 1
                    self.envia_comando_tracao(0, 0)
                    self.envia_comando_bracos(0, 0)
                    self.envia_comando_manipulador([0, 0, 0, 0, 0, 0])
            elif self.checkpoints_flags[3] == 0:
            # if self.checkpoints_flags[3] == 0:
                self.vai_pra(self.checkpoints_coords[3])
                self.verifica_se_chegou(3)
            elif self.tarefas_flags[1] == 0:
                if self.tempo_inicio_tarefas[1] == None:
                    self.tempo_inicio_tarefas[1]=tempo_decorrido
                tempo_decorrido_do_inicio_da_tarefa = tempo_decorrido - self.tempo_inicio_tarefas[1]
                if tempo_decorrido_do_inicio_da_tarefa>=0 and tempo_decorrido_do_inicio_da_tarefa<=1:
                    self.envia_comando_tracao(0, 0)
                    self.envia_comando_bracos(0, 0)
                    self.envia_comando_manipulador([np.pi/2, 0, 0, 0, -np.pi/2, 0])
                elif tempo_decorrido_do_inicio_da_tarefa>=1 and tempo_decorrido_do_inicio_da_tarefa<=2:
                    self.envia_comando_manipulador([np.pi/2, 0.4, 0, -0.4, -np.pi/2, 0])
                elif tempo_decorrido_do_inicio_da_tarefa>=2 and tempo_decorrido_do_inicio_da_tarefa<=3:
                    self.envia_comando_manipulador([np.pi/2, 0.6, 0, -0.6, -np.pi/2, 0])
                else:
                    self.tarefas_flags[1] = 1
                    self.envia_comando_tracao(0, 0)
                    self.envia_comando_bracos(0, 0)
                    self.envia_comando_manipulador([0, 0, 0, 0, 0, 0])
            else:
                self.envia_comando_tracao(0, 0)
                self.envia_comando_bracos(0,0)
                self.envia_comando_manipulador([0, 0, 0, 0, 0, 0])
                break
            print 'Checkpoints',self.checkpoints_flags
            print 'Tarefas', self.tarefas_flags

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
        v = .1
        angulo_relativo = self.ajeita_angulo(np.arctan2([coord[1]-y], [coord[0]-x]) - teta)
        wmax = .1
        w = angulo_relativo[0] * .5
        if w > wmax: w = wmax
        elif w < -wmax: w = -wmax
        self.envia_comando_tracao(v, w)

    def verifica_se_chegou(self, ind_check):
        d_limite = .1
        d = ((self.checkpoints_coords[ind_check][0] - self.pose[0])**2 + (self.checkpoints_coords[ind_check][1] - self.pose[1])**2)**.5
        if d<d_limite: self.checkpoints_flags[ind_check] = 1


if __name__ == '__main__':
	rospy.init_node('HofsMain')
	try:
		node_obj = HofsMain()
	except rospy.ROSInterruptException: pass
