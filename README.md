[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/oiTPsMcz)
# 2024-P4-ServiceForward
## Introduccion
El objetivo de la practica es conseguir que el [Kobuki](https://robots.ros.org/kobuki/) avanza la distancia exacta que manda el cliente al servidor, para ello haré uso de TFs y comunicaciones entre servidor y cliente.

## Creación de paquetes
Esta práctica hace uso de 2 paquetes:
- En el paquete [service_forward_interfaces](https://github.com/Docencia-fmrico/2024-p4-serviceforward-jmartinm2021/tree/main/service_forward_interfaces) están definidos los servicios, mensajes y acciones.
- El paquete [service_forward](https://github.com/Docencia-fmrico/2024-p4-serviceforward-jmartinm2021/tree/main/service_forward) contiene los nodos del cliente y servidor.

## Descripcion y procedimiento
En esta práctica, he utilizado como base el paquete [**ASR_2024**](https://github.com/Docencia-fmrico/ASR_2024) proporcionado por [fmrico](https://github.com/fmrico). 

En cuanto el servidor recibe el mensaje del cliente con una distancia, el servidor toma la posicion del robot.
```cpp
if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
   auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
    tf2::fromMsg(odom2bf_msg, odom2bf_);
}
```

Posteriormente el server va comprobando la posicion del robot en ese mmoemnto y saca la transformada entre la posicion del robot y la posicion inicial para sacar la distancia
```cpp
tf2::Transform bf2bfa = odom2bf_inverse * odom2bfa;

//  Extracts the x and y coordinates from the obtained transformation.
double x = bf2bfa.getOrigin().x();
double y = bf2bfa.getOrigin().y();

//  Calculate the distance between (0,0) and (x,y)
actual_distance_ = sqrt(x * x + y * y);
```

Dependiendo de la distancia, la FSM estara en el estado FORWARD o STOP
```cpp
switch (state_) {
    case FORWARD:
      RCLCPP_INFO(get_logger(), "Moving forward!");
      l_vel_.linear.x = MOVE_SPEED;
      vel_->publish(l_vel_);

      if (check_distance()) {
        go_state(STOP);
      }
      break;

    case STOP:
      RCLCPP_INFO(get_logger(), "STOP!");
      l_vel_.linear.x = STOP_SPEED;
      vel_->publish(l_vel_);

      if (!check_distance()) {
        go_state(FORWARD);
      }
      break;
}
```
## Launcher y ejecución
Para ejecutar estos nodos he usado un launcher en el que al cliente le paso como parametros la distancia.
```cpp
client_node = Node(
        package='service_forward',
        executable='client',
        arguments=['2'],
        output='screen'
    )
```

Tambien se puede ejecutar escribiendo en una terminal el server
```shell
ros2 run service_forward server 
```

Y en otra terminal el cliente con la distancia deseada
```shell
ros2 run service_forward client '[distancia]'
```
  
## Video demostración
[Grabación de pantalla desde 20-02-24 13:54:14.webm](https://github.com/Docencia-fmrico/2024-p4-serviceforward-jmartinm2021/assets/92941332/534170f8-ff8d-41e5-ae7d-14286a572684)  


## Enunciado
En esta práctica debes crear 2 paquetes:

1. En un paquete debes definir los interfaces necesarios.
2. En otro paquete debes crear dos nodos:
    * **Nodo servidor**: Este nodo implementa un servicio por el que se le envían comandos de avance, indicando la distancia a recorrer. El nodo está latente hasta que recibe el comando, devolviendo un mensaje `std_msgs/msg/Empty`. Cuando ha acabado, se para y espera un nuevo comando. Si recibe un comando mientras está ejecutando otro, el nuevo comando se empieza a ejecutar inmediatamente, obviando el anterior.
    * **Nodo cliente**: Este nodo se encarga de recibir como argumento la distancia a recorrer, y llamar al servicio del servidor. Termina inmediatamente al recibir la respuesta.

