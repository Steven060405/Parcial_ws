#!/usr/bin/env python3
import csv
import re
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from parcial_interfaces.srv import InterpretarOrden
from parcial_interprete.clasificador import clasificar_local

BASE = Path(__file__).resolve().parent
ENTRADA = Path(sys.argv[1]).expanduser().resolve() if len(sys.argv) > 1 else BASE / 'frases_50.csv'


def as_bool(v):
    return str(v).strip().lower() in {'true', '1', 'si', 'sí', 'yes'}


def percentile(values, p):
    values = sorted(float(x) for x in values if x is not None and float(x) >= 0)
    if not values:
        return float('nan')
    if len(values) == 1:
        return values[0]
    pos = (len(values) - 1) * p
    lo = int(pos)
    hi = min(lo + 1, len(values) - 1)
    frac = pos - lo
    return values[lo] + (values[hi] - values[lo]) * frac


def timing_line(name, values):
    vals = [float(x) for x in values if x is not None and float(x) >= 0]
    if not vals:
        return f'{name}: sin datos validos'
    return f'{name}: n={len(vals)}, mediana={statistics.median(vals):.3f} ms, p95={percentile(vals, 0.95):.3f} ms'


def extraer(detalle, campo):
    m = re.search(rf'{re.escape(campo)}=(True|False|\d+)', detalle or '')
    if not m:
        return ''
    value = m.group(1)
    if value in ('True', 'False'):
        return value == 'True'
    return int(value)


class Bench(Node):
    def __init__(self):
        super().__init__('benchmark_item1')
        self.cli = self.create_client(InterpretarOrden, '/interpretar_orden')

    def call(self, frase):
        req = InterpretarOrden.Request()
        req.frase = frase
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        if not fut.done():
            raise TimeoutError('Servicio ROS sin respuesta en 20 s')
        if fut.exception():
            raise RuntimeError(str(fut.exception()))
        return fut.result()


def main():
    if not ENTRADA.exists():
        raise SystemExit(f'No existe: {ENTRADA}')
    with ENTRADA.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit('CSV vacio')

    rclpy.init()
    node = Bench()
    if not node.cli.wait_for_service(timeout_sec=10.0):
        node.destroy_node(); rclpy.shutdown()
        raise SystemExit('/interpretar_orden no disponible')

    out = []
    for i, row in enumerate(rows, 1):
        frase = row['frase'].strip()
        esperado = {
            'accion': row['accion_esperada'].strip(),
            'objeto': row['objeto_esperado'].strip(),
            'color': row['color_esperado'].strip(),
            'prioridad': int(row['prioridad_esperada']),
            'permitido': as_bool(row['permitido_esperado']),
        }
        kw = clasificar_local(frase)
        t0 = time.perf_counter()
        try:
            r = node.call(frase)
        except Exception as e:
            print(f'[{i:02d}/{len(rows):02d}] ERROR {frase}: {e}')
            continue
        client_ms = (time.perf_counter() - t0) * 1000.0
        final = {
            'accion': r.accion, 'objeto': r.objeto, 'color': r.color,
            'prioridad': int(r.prioridad), 'permitido': bool(r.permitido),
        }
        laya_ok = not r.degradado
        laya = {
            'accion': r.accion if laya_ok else '',
            'objeto': r.objeto if laya_ok else '',
            'color': r.color if laya_ok else '',
            'prioridad': extraer(r.detalle, 'prioridad_laya') if laya_ok else '',
            'permitido': extraer(r.detalle, 'permitido_laya') if laya_ok else '',
        }
        record = {
            'numero': i, 'frase': frase,
            **{f'{k}_esperado': v for k, v in esperado.items()},
            **{f'{k}_keywords': v for k, v in kw.items()},
            **{f'{k}_laya': v for k, v in laya.items()},
            **{f'{k}_final': v for k, v in final.items()},
            'laya_disponible': laya_ok,
            'keywords_total_ok': all(kw[k] == esperado[k] for k in esperado),
            'laya_total_ok': all(laya[k] == esperado[k] for k in esperado) if laya_ok else '',
            'final_total_ok': all(final[k] == esperado[k] for k in esperado),
            'degradado': r.degradado, 'metodo': r.metodo, 'detalle': r.detalle,
            'tiempo_cliente_ms': client_ms,
            'tiempo_proceso_ms': r.tiempo_proceso_ms,
            'http_total_ms': r.http_total_ms,
            'inferencia_laya_ms': r.inferencia_laya_ms,
            'rtt_red_ms': r.rtt_red_ms,
        }
        out.append(record)
        print(f'[{i:02d}/{len(rows):02d}] {frase} -> {final} metodo={r.metodo}')

    node.destroy_node(); rclpy.shutdown()
    if not out:
        raise SystemExit('No hubo resultados')

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = BASE / f'resultados_item1_{stamp}.csv'
    with csv_path.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=out[0].keys())
        w.writeheader(); w.writerows(out)

    laya_rows = [x for x in out if x['laya_disponible']]
    pct = lambda n, d: (100.0*n/d) if d else 0.0
    lines = [
        'RESUMEN BENCHMARK ITEM 1', '='*60,
        f'Frases procesadas: {len(out)}',
        f'LAYA disponible: {len(laya_rows)}/{len(out)}',
        f'Exactitud total LAYA: {pct(sum(x["laya_total_ok"] is True for x in laya_rows), len(laya_rows)):.2f}%',
        f'Exactitud total Keywords: {pct(sum(x["keywords_total_ok"] is True for x in out), len(out)):.2f}%',
        f'Exactitud total Sistema final: {pct(sum(x["final_total_ok"] is True for x in out), len(out)):.2f}%',
        '', 'LATENCIAS', '-'*60,
        timing_line('Cliente ROS', [x['tiempo_cliente_ms'] for x in out]),
        timing_line('Proceso nodo', [x['tiempo_proceso_ms'] for x in out]),
        timing_line('HTTP total', [x['http_total_ms'] for x in out]),
        timing_line('Inferencia LAYA', [x['inferencia_laya_ms'] for x in out]),
        timing_line('RTT/overhead estimado', [x['rtt_red_ms'] for x in out]),
        '', f'Respuestas degradadas: {sum(bool(x["degradado"]) for x in out)}/{len(out)}',
    ]
    summary = BASE / f'resumen_item1_{stamp}.txt'
    summary.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n' + '\n'.join(lines))
    print(f'\nCSV: {csv_path}\nResumen: {summary}')


if __name__ == '__main__':
    main()
