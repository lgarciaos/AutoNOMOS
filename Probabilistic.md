# Probabilistic Localization

Para ejecutar la versión con navegación probabilistica ejecutar cualquiera de los siguientes nodos:
```
autonomos_gazebo_control_ccircuito.launch
autonomos_gazebo_control_cderecho.launch
autonomos_gazebo_control_covalo.launch
autonomos_gazebo_control_cs.launch
```
Posteriormente abrir el nodo para sensado con cámara.
```
roslaunch line_detection_fu_mod line_detection_fu.launch
```
El nodo de localización probabilistica de estados.
```
roslaunch lane_states_original states.launch
```
En este momento si ejecutamos `rqt` podrémos observar en el topico `//` un texto con el estado donde se localiza el vehículo de acuerdo a lo observado por la camara y el movimiento como se indica en la localización Bayesiana. 
Al ejecutar el nodo `roslaunch lane_controller_sim controller.launch`, enviamos señales a los nodos que controlar giro y velocidad para enviar las señales de acuerdo al centro del estado actual.

Por último para cambiar nuestro estado a uno a la izquierda o a la derecha se puede publicar un mensaje al nodo
```
rostopic pub -1 desired_state std_msgs/Float64 '{data: -1}'
```
En el cual enviamos:
* -1 si queremos movernos un estado a la izquierda
* 1 si queremos movernos un estado a la derecha 
* 0 para permanecer en el estado actual
