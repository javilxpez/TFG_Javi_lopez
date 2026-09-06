#!/usr/bin/env python3
"""Lee un CSV de ensayo de cycles/ y resume lo que hay dentro.

Sólo biblioteca estándar: no hace falta instalar nada.

    python3 leer_ciclo.py                  # el ensayo más reciente
    python3 leer_ciclo.py cycles/xxx.csv   # uno concreto
    python3 leer_ciclo.py --plano salida.csv   # sin cabecera '#', para Excel/Calc
"""
import csv
import sys
from pathlib import Path

CYCLE_DIR = Path(__file__).parent / "cycles"


def cargar(path: Path):
    """Devuelve (constantes, filas). Las líneas '#' son la cabecera del ensayo."""
    consts, lineas = {}, []
    for linea in path.read_text(encoding="utf-8").splitlines():
        if linea.startswith("#"):
            # "# lc1_cuentas_por_N=1234  lc2_tara=-42" → {'lc1_cuentas_por_N': '1234', ...}
            for trozo in linea.lstrip("#").split():
                if "=" in trozo:
                    k, v = trozo.split("=", 1)
                    consts[k] = v
        else:
            lineas.append(linea)
    return consts, list(csv.DictReader(lineas))


def resumen(consts, filas):
    if not filas:
        print("El fichero no tiene ni una muestra."); return
    num = lambda c: [float(f[c]) for f in filas if f[c] not in ("", None)]
    t, pos = num("t_rel_s"), num("pos_rev")
    lc1, lc2 = num("lc1_base"), num("lc2_raw")
    reps = sorted({int(f["rep"]) for f in filas})

    print(f"\n  Muestras ....... {len(filas)}  en {t[-1]:.2f} s"
          f"  ({len(filas)/max(t[-1], 1e-9):.1f} Hz)")
    print(f"  Repeticiones ... {reps[0]}..{reps[-1]}  ({len(reps)})")
    print(f"  Posición ....... {min(pos):+.4f} .. {max(pos):+.4f} rev")
    print(f"  LC1 (tarada) ... {min(lc1):+.0f} .. {max(lc1):+.0f} cuentas")
    print(f"  LC2 (cruda) .... {min(lc2):+.0f} .. {max(lc2):+.0f} cuentas")

    # Sólo se convierte a newtons si el ensayo dejó escrita la constante
    for nombre, datos, clave in (("LC1", lc1, "lc1_cuentas_por_N"),
                                 ("LC2", lc2, "lc2_cuentas_por_N")):
        cal = float(consts.get(clave, 0) or 0)
        if cal:
            print(f"  {nombre} en fuerza . {min(datos)/cal:+.2f} .. {max(datos)/cal:+.2f} N"
                  f"   (recorrido {(max(datos)-min(datos))/cal:.2f} N)")
        else:
            print(f"  {nombre} en fuerza . sin calibrar ({clave}=0)")

    print("\n  Por repetición:")
    print(f"    {'rep':>4} {'muestras':>9} {'t (s)':>8} {'LC1 máx':>9} {'pos máx':>9}")
    for r in reps:
        f = [x for x in filas if int(x["rep"]) == r]
        ts = [float(x["t_rel_s"]) for x in f]
        print(f"    {r:>4} {len(f):>9} {ts[-1]-ts[0]:>8.2f}"
              f" {max(float(x['lc1_base']) for x in f):>9.0f}"
              f" {max(float(x['pos_rev']) for x in f):>9.4f}")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if args:
        path = Path(args[0])
    else:
        csvs = sorted(CYCLE_DIR.glob("*.csv"), key=lambda p: p.stat().st_mtime)
        if not csvs:
            sys.exit(f"No hay ensayos en {CYCLE_DIR}/ — lanza un ciclo primero.")
        path = csvs[-1]
    if not path.is_file():
        sys.exit(f"No existe: {path}")

    consts, filas = cargar(path)
    print(f"\n{path}")
    print("  " + "  ".join(f"{k}={v}" for k, v in consts.items()))
    resumen(consts, filas)

    if "--plano" in sys.argv:
        destino = Path(sys.argv[sys.argv.index("--plano") + 1])
        with destino.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=filas[0].keys())
            w.writeheader(); w.writerows(filas)
        print(f"\n  Escrito sin cabecera '#': {destino}")
    print()


if __name__ == "__main__":
    main()
