# Hofs ROSI CHALLENGE 2019

Solução da equipe Hofs.

# Instalação

Seguir todos os passos de instalação de https://github.com/filRocha/rosiChallenge-sbai2019.
Após isso, ir no Workspace (`../catkin_ws/src`) e executar:

```
$ git clone https://github.com/Gustavo-Hofs/hofs_rosi_challenge_2019.git hofs_rosi_challenge_2019_oficial_03
```

Ir em (`../catkin_ws`) e executar:

```
$ catkin build
```

# Execução

```
$ roslaunch hofs_rosi_challenge_2019_oficial_03 hofs_oficial_03.launch
```

# Descrição sobre Nós e Tópicos

Este pacote contém 2 nós:

- `controlador_de_movimento` - Funciona como uma ponte entre o nó principal `hofs_main` e o nó `vrep_ros_interface`. Traduz velocidade linear e angular em velocidade rotacional para cada roda do robô através do subscriber `/controlador_tracao` e do publisher `/rosi/command_traction_speed`. Traduz velocidade frontal e traseira em velocidade rotacional para cada braço do robô através do subscriber `/controlador_bracos` e do publisher `/rosi/command_arms_speed`.

- `hofs_main` - Nó principal. Pega a leitura dos sensores de tempo, GPS, força de toque, câmera e kinect pelos tópicos `/simulation/time`, `/sensor/gps`, `/ur5/forceTorqueSensorOutput`, `/sensor/ur5toolCam` e `/sensor/kinect_depth`. Comanda a tração, os braços e o manipulador pelos tópicos `/controlador_tracao`, `/controlador_bracos` e `/ur5/jointsPosTargetCommand`. Registra ocorrencias de fogo em `outputs/Mapa das ocorrencias de fogo.txt`.

# Contato

Email: `hofstatter.gustavo@engenharia.ufjf.br`

LinkedIn: https://www.linkedin.com/in/hofs
