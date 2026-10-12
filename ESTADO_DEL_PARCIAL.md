# Estado técnico del ZIP

Origen de referencia: repositorio `Steven060405/PARCIAL_ROBOTICA`, rama `master`, árbol observado `926d5497cebaed0395499059926a25ce1aafa0b4`.

## P1

Lista para prueba de software: servicio, LAYA HTTP, timeout, fallback, decisión tipada, mediciones y benchmark.

## P2

El ZIP conserva la arquitectura del broker RB-2 (cola, política, worker único y publicación única). También incluye la percepción por color y el planificador MoveIt que solo planifica. El código del repositorio de referencia aún deja la ejecución semántica con el mensaje `pipeline fisico pendiente`, y la percepción devuelve XYZ=0 hasta calibrar. Por eso **no debe considerarse P2 físicamente terminada** sin el JetCobot.

## P3

Se incluye una base funcional de transcripción por archivo de audio usando una clave en `GOOGLE_API_KEY`, con fallback a teclado y llamada a `/interpretar_orden`. Puede encolar al broker si se habilita, pero el movimiento final depende de completar/validar P2.

## Importante

Este ZIP se creó para que puedas levantar y depurar el workspace en tu máquina. No sustituye las pruebas físicas ni las evidencias exigidas durante el examen.
