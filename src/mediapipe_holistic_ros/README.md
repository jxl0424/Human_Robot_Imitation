# mediapipe_holistic_ros 

 

O MediaPipe Holistic pode ser acessado em https://google.github.io/mediapipe/solutions/holistic.html. Se trata de uma ferramenta capaz de fornecer a identificação de partes do corpo humano como mãos, face, corpo entre outros a partir do processamento de imagem.   

![MediaPipe](https://google.github.io/mediapipe/images/mobile/holistic_sports_and_gestures_example.gif) 


Este pacote tem como finalidade converter o MediaPipe Holistic em um pacote ROS (https://www.ros.org/) para que possa ser utilizado por profissionais e entusiastas de robótica. Desta forma, as entradas e saídas do pacote serão em formato de tópicos ROS, podendo ser configurado diretamente do Launch!   

 

## Primeiro, vamos aos requisitos:  

### ROS (http://wiki.ros.org/ROS/Installation) 
### MediaPipe (https://google.github.io/mediapipe/getting_started/python.html): 

``` 
$ pip install mediapipe 

``` 

## Sobre o pacote mediapipe_holistic_ros 

O pacote foi projetado para operar com tópicos ROS. O pacote irá ler a imagem de um tópico, processá-la e publicar os resultados em outros dois tópicos, um contendo somente os landmarks, ou seja, os pontos que correspondem as partes do corpo detectados, e outro contendo a imagem com os landmarks, para uma visualização do resultado.  

Os landmarks são publicados em um tópico personalizado, sendo do tipo “**mediapipe_holistic_ros/MediaPipeHolistic**”. 

 

## mediapipe_holistic_ros /MediaPipeHolistic 

Composição da mensagem ROS: 
- Header header 
- face_landmarks[468] 
- pose_landmarks[33] 
- pose_world_landmarks[33] 
- left_hand_landmarks[21] 
- right_hand_landmarks[21]  

Em cada posição de cada vetor, temos uma mensagem do tipo “mediapipe_holistic_ros / MediaPipePose”, sendo ela: 

- int32 id 
- float32 x 
- float32 y 
- float32 z 
- float32 visibility 

 
## face_landmarks 

- Corresponde aos pontos do rosto.  

![MediaPipe](https://google.github.io/mediapipe/images/mobile/face_mesh_android_gpu.gif) 

- Não utiliza o campo “visibility” 

 

## pose_landmarks 

- Corresponde aos 33 pontos representando do corpo humano. Utiliza os campos “x”,“y” e “visibility”, que representa a probabilidade de o ponto estar oculto na imagem ou não.  
![MediaPipe](https://google.github.io/mediapipe/images/mobile/pose_tracking_example.gif) 

 

## pose_world_landmarks 

- Mesmas informações do “pose_landmarks”, porém tras informações 3D referentes ao mundo real.  

![MediaPipe](https://1.bp.blogspot.com/-T-VwABuYXoo/YSlER0yqyjI/AAAAAAAAEdQ/PKk0E8DdViUSgEqII8IELWrCyHaNpLhZgCLcBGAsYHQ/w1200-h630-p-k-no-nu/TF%2Bimage%2B2.gif) 

 
- Os pontos referentes ao “pose_landmarks” e “pose_world_landmarks” com suas posições associados a um id podem ser vistos a seguir: 

![MediaPipe](https://google.github.io/mediapipe/images/mobile/pose_tracking_full_body_landmarks.png) 

 

## left_hand_landmarks e right_hand_landmarks 

- Representa os 21 pontos identificados tanto da mão esquerda quanto da mão esquerda. Não utiliza o “visibility”. 

![Hand](https://google.github.io/mediapipe/images/mobile/hand_crops.png) 

![MediaPipe](https://google.github.io/mediapipe/images/mobile/hand_landmarks.png) 

 

 

# Configurações do Launch 

## Argumentos 

- **Argumento**: python3 
  - **Valores aceitáveis**: true e false 
  - **Descrição**: Caso esteja usando o Python3 (Padrão no ROS Noetic), deixar marcado como true. Caso esteja utilizando versões antigas do ROS com o Python2, deixar marcado como false, e o pacote funcionará normalmente, com as mesmas funcionalidades!  
  - **Não implementado (ainda)**
 

- **Argumento**: run_usb_cam 
  - **Valores aceitáveis**: true e false 
  - **Descrição**: Caso queira executar o pacote ROS “usb_cam” diretamente deste launch, deixe marcado como verdadeiro, e será executado o usb_cam para converter a imagem de sua câmera em um tópico ROS. Caso queira utilizar uma imagem já publicada em outro tópico, deixe marcado como “false”. Lembrando que o tópico deve ser do tipo “sensor_msgs/Image” 

 
## Parametros 

- **Parâmetro**: “/mediapipe_holistic_ros/input_usb_cam_topic” 
  - **Valor padrão**: "/usb_cam/image_raw" 
  - **Valores aceitáveis**: Qualquer nome no formato String. 
  - **Descrição**: Nome do tópico contendo a imagem que será utilizada para ser aplicado ao MediaPipe Holistic.  
  - **Tipo do tópico**: “sensor_msgs/Image” 

- **Parâmetro**: /mediapipe_holistic_ros/pub_image_output 
  - **Valor padrão**: “true” 
  - **Valores aceitáveis**: “true” ou “false” 
  -**Descrição**:  Ativa e desativa a publicação da imagem processada pelo MediaPipe.      

- **Parâmetro**: "/mediapipe_holistic_ros/output_image_topic" 
  - **Valor padrão**: "/mediapipe_holistic/output_image" 
  - **Valores aceitáveis**: Qualquer nome no formato String. 
  - **Descrição**: Nome do tópico de saída no formato imagem. Contém imagem de entrada com os landmarks marcados na imagem. Só será publicado se o parâmetro “**/mediapipe_holistic_ros/pub_image_output**” estiver marcado como “**true**”  	 
  - **Tipo do tópico:** “sensor_msgs/Image” 

- **Parâmetro**: "/mediapipe_holistic_ros/pub_landmark_output"" 
  - **Valor padrão**: “true” 
  - **Valores aceitáveis**: “true” ou “false” 
  - **Descrição**:  Ativa e desativa a publicação dos resultados  (landmarks) em um tópico ROS.  

- **Parâmetro**: “/mediapipe_holistic_ros/output_mediapipe_topic" 
  - **Valor padrão**: "/MediaPipePose/holistic/landmarks" 
  - **Valores aceitáveis**: Qualquer nome no formato String. 
  - **Descrição**: Nome do tópico contendo os landmarks identificados na imagem pelo MediaPipe. Se trata de um tipo de mensagem personalizado presente no pacote. 
  - **Tipo do tópico**: “mediapipe_holistic_ros /MediaPipeHolistic” 

- **Parâmetro**: "/mediapipe_holistic_ros/open_view_image" 
  - **Valor padrão**: “true” 
  - **Valores aceitáveis**: “true” ou “false” 
  - **Descrição**:  Ativa e desativa a visualização da imagem de saída via openCV (fora do ROS).   
