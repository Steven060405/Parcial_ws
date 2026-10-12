# Paso a paso — Pregunta 1

## A. Windows: LAYA

1. Deja LAYA ejecutándose.
2. En PowerShell obtiene la IPv4:

```powershell
ipconfig
```

3. Identifica el puerto donde escucha LAYA.
4. Comprueba el health local:

```powershell
curl.exe http://127.0.0.1:PUERTO/health
```

5. Para que Ubuntu/Jetson pueda entrar, LAYA debe escuchar en una interfaz de red accesible. Si desde Ubuntu el `curl` no llega, revisa bind/firewall antes de culpar a ROS.

## B. Ubuntu/Jetson: instalar y compilar P1

Descomprime el ZIP. Luego:

```bash
cd PARCIAL_ROBOTICA_READY
chmod +x *.sh
source /opt/ros/humble/setup.bash
./preparar.sh p1
```

Comprueba que el ejecutable quedó instalado:

```bash
source install/setup.bash
ros2 pkg executables parcial_interprete
```

Debe aparecer:

```text
parcial_interprete interprete_ordenes
```

## C. Probar conectividad con Windows

Antes de lanzar ROS:

```bash
curl http://IP_WINDOWS:PUERTO/health
```

Si esto falla, `/interpretar_orden` podrá arrancar, pero las órdenes caerán al fallback por keywords.

## D. Levantar P1

Terminal 1:

```bash
cd PARCIAL_ROBOTICA_READY
./iniciar_item1.sh
```

Para una prueba local sin Discovery Server, deja ese campo vacío. Usa la IP de Windows como IP LAYA.

La terminal debe quedarse abierta y mostrar `interprete_ordenes listo`.

## E. Ver el nodo que antes no aparecía

Terminal 2, usando el mismo `ROS_DOMAIN_ID`:

```bash
cd PARCIAL_ROBOTICA_READY
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=62
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER
ros2 node list
ros2 service list | grep interpretar
```

Resultado esperado:

```text
/interprete_ordenes
/interpretar_orden
```

Si no aparece, ejecuta:

```bash
./diagnostico_p1.sh
```

## F. Probar una frase

```bash
./probar_item1.sh urgente recoge el cubo rojo
```

Con LAYA funcionando esperamos `degradado: false` y `metodo: laya` (salvo veto local de seguridad).

## G. Probar el fallback obligatorio

Detén LAYA en Windows y repite:

```bash
./probar_item1.sh urgente recoge el cubo rojo
```

Ahora debe aparecer `degradado: true` y `metodo: keywords`. El nodo no debe quedar esperando indefinidamente.

## H. Benchmark

Con LAYA otra vez encendido:

```bash
python3 benchmark/benchmark_item1.py
```

Genera un CSV y un resumen con mediana, P95, HTTP total, inferencia y RTT/overhead estimado. El día del examen usa el CSV de 50 frases del docente:

```bash
python3 benchmark/benchmark_item1.py /ruta/frases_docente.csv
```

Haz una corrida en calma y otra bajo carga y conserva ambas.
