# 2024-P4-ServiceForward

En esta práctica debes crear 2 paquetes:

1. En un paquete debes definir los interfaces necesarios.
2. En otro paquete debes crear dos nodos:
    * **Nodo servidor**: Este nodo implementa un servicio por el que se le envían comandos de avance, indicando la distancia a recorrer. El nodo está latente hasta que recibe el comando, devolviendo un mensaje `std_msgs/msg/Empty`. Cuando ha acabado, se para y espera un nuevo comando. Si recibe un comando mientras está ejecutando otro, el nuevo comando se empieza a ejecutar inmediatamente, obviando el anterior.
    * **Nodo cliente**: Este nodo se encarga de recibir como argumento la distancia a recorrer, y llamar al servicio del servidor. Termina inmediatamente al recibir la respuesta.

