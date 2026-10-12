# PARCIAL_ROBOTICA_READY

Workspace ROS 2 Humble preparado a partir del repositorio `Steven060405/PARCIAL_ROBOTICA` y del enunciado del Examen Parcial — Gran Reto JetCobot.

## Estado

- **Pregunta 1:** preparada para compilar y probar con LAYA en otra PC (Windows) por HTTP. Incluye `/interpretar_orden`, timeout, fallback por palabras clave, latencias y benchmark de 50 frases.
- **Pregunta 2:** se conserva el broker RB-2 y los paquetes de percepción/planificación del repositorio. **No se afirma que el agarre físico esté terminado**: la versión del repositorio aún tiene XYZ/calibración y pipeline físico pendientes.
- **Pregunta 3:** se incluye un nodo base de transcripción con Google AI Studio/Gemini y fallback a texto. Debe validarse con el modelo/API disponible el día del examen y con la cadena física de P2.

## Requisitos

- Ubuntu 22.04 / Jetson con ROS 2 Humble.
- Python 3, `requests`.
- Para P2: OpenCV/Numpy, MoveIt 2 y la configuración MoveIt del JetCobot.
- Para P3: variable `GOOGLE_API_KEY` y acceso a Internet.

## Primera prueba recomendada: Pregunta 1

```bash
cd PARCIAL_ROBOTICA_READY
chmod +x preparar.sh iniciar_item1.sh iniciar_item2.sh iniciar_item3.sh
./preparar.sh p1
./iniciar_item1.sh
```

En una segunda terminal usa el mismo `ROS_DOMAIN_ID`:

```bash
cd PARCIAL_ROBOTICA_READY
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=62
export ROS_LOCALHOST_ONLY=0
ros2 node list
ros2 service list | grep interpretar
ros2 service call /interpretar_orden parcial_interfaces/srv/InterpretarOrden "{frase: 'urgente recoge el cubo rojo'}"
```

Si el nodo se inició correctamente deben aparecer `/interprete_ordenes` y `/interpretar_orden`.

## LAYA en Windows

Obtén la IPv4 de Windows con `ipconfig`. LAYA debe escuchar en una interfaz accesible desde la máquina ROS, no solamente en `127.0.0.1`. Antes de ROS prueba desde Ubuntu/Jetson:

```bash
curl http://IP_WINDOWS:PUERTO/health
```

`iniciar_item1.sh` pide IP, puerto, modelo y timeout cada vez: no hay una IP fija guardada en el código.

## Benchmark P1

Con `/interpretar_orden` activo:

```bash
python3 benchmark/benchmark_item1.py
```

Por defecto usa `benchmark/frases_50.csv`. También acepta un CSV distinto:

```bash
python3 benchmark/benchmark_item1.py /ruta/frases_profesor.csv
```

Ejecuta una vez en calma y otra bajo carga y conserva ambos CSV/resúmenes.
