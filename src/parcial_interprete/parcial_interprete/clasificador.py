import unicodedata


def normalizar(texto):
    texto = texto.lower().strip()
    texto = unicodedata.normalize('NFD', texto)
    return ''.join(c for c in texto if unicodedata.category(c) != 'Mn')


def detectar_accion(frase):
    texto = normalizar(frase)
    for palabra in ['recoge', 'recoger', 'agarra', 'agarrar', 'toma', 'tomar', 'coge']:
        if palabra in texto:
            return 'recoger'
    for palabra in ['deposita', 'depositar', 'coloca', 'colocar', 'pon', 'poner', 'lleva']:
        if palabra in texto:
            return 'depositar'
    for palabra in ['deten', 'detener', 'para', 'parar', 'alto', 'stop']:
        if palabra in texto:
            return 'detener'
    return 'desconocida'


def detectar_color(frase):
    texto = normalizar(frase)
    colores = {
        'rojo': ['rojo', 'roja'],
        'verde': ['verde'],
        'azul': ['azul'],
        'amarillo': ['amarillo', 'amarilla'],
    }
    for canonico, variantes in colores.items():
        if any(v in texto for v in variantes):
            return canonico
    return 'desconocido'


def detectar_prioridad(frase):
    texto = normalizar(frase)
    if any(p in texto for p in ['urgente', 'inmediatamente', 'ahora mismo', 'emergencia']):
        return 3
    if any(p in texto for p in ['rapido', 'rapidamente', 'prioridad alta', 'importante']):
        return 2
    if any(p in texto for p in ['cuando puedas', 'sin apuro', 'despacio', 'prioridad baja']):
        return 0
    return 1


def detectar_permitido(frase):
    texto = normalizar(frase)
    return not any(p in texto for p in ['golpea', 'golpear', 'lastima', 'lastimar', 'ataca', 'atacar'])


def detectar_objeto(frase):
    texto = normalizar(frase)
    for objeto in ['cubo', 'bloque', 'botella', 'pelota']:
        if objeto in texto:
            return objeto
    return 'desconocido'


def clasificar_local(frase):
    return {
        'accion': detectar_accion(frase),
        'objeto': detectar_objeto(frase),
        'color': detectar_color(frase),
        'prioridad': detectar_prioridad(frase),
        'permitido': detectar_permitido(frase),
    }
