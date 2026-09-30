"""Genera la version ligera de la memoria para el plan gratuito de Overleaf.

Hace tres cosas, que juntas bajan el compilado de ~17 s a ~5 s:
  1. Rasteriza a PDF cada diagrama de tikz/pgfgantt y lo sustituye por un
     \\includegraphics, de modo que no hay que redibujarlos en cada pasada.
  2. Quita tikz y pgfgantt del preambulo.
  3. Incrusta la bibliografia ya resuelta por BibTeX (\\input de un .bbl), con lo
     que Overleaf no tiene que ejecutar BibTeX ni las dos pasadas extra.

Uso:   python memoria/figuras/generar_overleaf.py [--salida DIR] [--tectonic RUTA]
Deja la carpeta lista para subir y, si encuentra zip, tambien el .zip.
"""
import argparse, os, re, shutil, subprocess, sys, tempfile, zipfile

AQUI    = os.path.dirname(os.path.abspath(__file__))
MEMORIA = os.path.dirname(AQUI)
RAIZ    = os.path.dirname(MEMORIA)

PREAMBULO = r"""\documentclass[border=2pt]{standalone}
\usepackage{iftex}
\ifPDFTeX
  \usepackage[utf8]{inputenc}\usepackage[T1]{fontenc}\usepackage{lmodern}\usepackage{textcomp}
\else
  \usepackage{fontspec}
\fi
\usepackage[spanish,es-tabla,es-noshorthands]{babel}
\usepackage{amsmath}\usepackage{amssymb}
\usepackage{siunitx}
\sisetup{output-decimal-marker={,}, group-separator={.}, per-mode=symbol,
         range-phrase={ a }, list-final-separator={ y }}
\DeclareSIUnit{\uacc}{u.a.}
\usepackage{xcolor}
\usepackage{tikz}
\usetikzlibrary{arrows.meta,positioning,shapes.geometric,shapes.misc,fit,calc,backgrounds,babel}
\usepackage{pgfgantt}
\newcommand{\codigo}[1]{\texttt{#1}}
\newcommand{\fichero}[1]{\texttt{#1}}
\newcommand{\ua}{u.\,a.}
\DeclareRobustCommand{\completar}[1]{\textcolor{red!75!black}{\textbf{[COMPLETAR:} #1\textbf{]}}}
\DeclareRobustCommand{\verificar}[1]{\textcolor{orange!80!black}{\textbf{[VERIFICAR:} #1\textbf{]}}}
\begin{document}
%s
\end{document}
"""

LEEME = """# Memoria del TFG — version para Overleaf

Sube esta carpeta a Overleaf, abre `TFG.tex` y compila (pdfLaTeX). Esta preparada para caber
en el limite de tiempo del plan gratuito: compila en unos 5 s en vez de unos 17 s.

## Que la diferencia de la version del repositorio

1. **Diagramas ya rasterizados.** Los dibujos de `tikz` y `pgfgantt` son aqui PDF ya generados
   (`figuras/fig_*.pdf`), y esos dos paquetes no se cargan.
2. **Bibliografia ya resuelta.** En vez de `\\bibliography{...}` hay un
   `\\input{bibliografia/bibliografia_resuelta}` con las entradas ya formateadas, asi que Overleaf
   no ejecuta BibTeX ni las dos pasadas adicionales que exige.

El PDF resultante es el mismo: mismas paginas, mismo indice y misma bibliografia.

## Esta carpeta es generada: no la edites

Se crea con `python memoria/figuras/generar_overleaf.py` desde el repositorio. Escribe aqui, y
cualquier cambio que hagas en esta copia se perdera la proxima vez que la generes. **Edita siempre
`memoria/` en el repositorio** y vuelve a generar.

Eso incluye la bibliografia: `bibliografia/bibliografia.bib` viaja en el zip por comodidad, pero al
compilar no se usa. Para anadir una referencia, editala en el repositorio y regenera.

## Si aun asi se agota el tiempo

En Overleaf, menu izquierdo -> *Compiler*, cambia **Normal** por **Fast [draft]**: reutiliza los
ficheros auxiliares y hace una sola pasada. Vuelve a *Normal* para la version final, porque solo
entonces se cuadran indices y referencias cruzadas.

## Pendientes

Las marcas `\\completar{...}` (rojo) y `\\verificar{...}` (naranja) senalan lo que falta por
rellenar o comprobar. Para listarlas:

    grep -rn "completar{\\|verificar{" capitulos TFG.tex
"""

ENTORNOS = ('tikzpicture', 'ganttchart')
ANCHO_TEXTO_PT = 425.2          # \textwidth con los margenes de 3 cm en A4


def bloques(texto):
    """Localiza los entornos de dibujo de nivel superior, sin solaparse."""
    fuera = []
    for env in ENTORNOS:
        for m in re.finditer(r'\\begin\{%s\}' % env, texto):
            ini = m.start()
            if any(a <= ini < b for a, b, _ in fuera):
                continue
            fin = texto.find('\\end{%s}' % env, ini)
            if fin == -1:
                continue
            fin += len('\\end{%s}' % env)
            fuera.append((ini, fin, texto[ini:fin]))
    return sorted(fuera)


def ancho_pdf(ruta):
    try:
        from pypdf import PdfReader
        return float(PdfReader(ruta).pages[0].mediabox.width)
    except Exception:
        return 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--salida', default=os.path.join(RAIZ, 'TFG_overleaf'))
    ap.add_argument('--tectonic', default=shutil.which('tectonic') or 'tectonic')
    ap.add_argument('--zip', default=os.path.join(RAIZ, 'TFG_overleaf.zip'))
    a = ap.parse_args()

    if not (os.path.isfile(a.tectonic) or shutil.which(a.tectonic)):
        sys.exit('No encuentro tectonic. Pasa la ruta con --tectonic.')

    dst = a.salida
    if os.path.isdir(dst):
        shutil.rmtree(dst)
    shutil.copytree(MEMORIA, dst, ignore=shutil.ignore_patterns(
        '_v1', '__pycache__', '*.aux', '*.log', '*.toc', '*.lof', '*.lot',
        '*.out', '*.blg', '*.synctex.gz', '*.fdb_latexmk', '*.fls'))

    # ── 1. rasterizar los diagramas ──────────────────────────────────────
    tmp = tempfile.mkdtemp(prefix='fig_')
    n = 0
    for nombre in sorted(os.listdir(os.path.join(dst, 'capitulos'))):
        if not nombre.endswith('.tex'):
            continue
        ruta = os.path.join(dst, 'capitulos', nombre)
        s = open(ruta, encoding='utf-8').read()
        bs = bloques(s)
        if not bs:
            continue
        base = nombre[:-4]
        for i, (ini, fin, cuerpo) in enumerate(reversed(bs), 1):
            idx = len(bs) - i + 1
            fig = 'fig_%s_%d' % (base, idx)
            tex = os.path.join(tmp, fig + '.tex')
            open(tex, 'w', encoding='utf-8').write(PREAMBULO % cuerpo)
            r = subprocess.run([a.tectonic, '-o', tmp, tex],
                               capture_output=True, text=True, cwd=tmp)
            pdf = os.path.join(tmp, fig + '.pdf')
            if r.returncode != 0 or not os.path.exists(pdf):
                print('  !! fallo al rasterizar %s' % fig)
                err = [l for l in r.stderr.splitlines() if l.startswith('error')]
                print('     ' + '\n     '.join(err[:3]))
                continue
            shutil.copy(pdf, os.path.join(dst, 'figuras', fig + '.pdf'))
            opt = '[width=\\textwidth]' if ancho_pdf(pdf) > ANCHO_TEXTO_PT else ''
            s = s[:ini] + '\\includegraphics%s{%s.pdf}' % (opt, fig) + s[fin:]
            n += 1
        open(ruta, 'w', encoding='utf-8').write(s)
    print('  %d diagramas rasterizados' % n)

    # ── 2. quitar tikz y pgfgantt del preambulo ──────────────────────────
    tfg = os.path.join(dst, 'TFG.tex')
    s = open(tfg, encoding='utf-8').read()
    s = s.replace('\\usepackage{tikz}\n', '')
    s = re.sub(r'\\usetikzlibrary\{[^}]*\}\n', '', s)
    s = s.replace('\\usepackage{pgfgantt}\n', '')
    open(tfg, 'w', encoding='utf-8').write(s)

    # ── 3. bibliografia ya resuelta ──────────────────────────────────────
    r = subprocess.run([a.tectonic, '--keep-intermediates', 'TFG.tex'],
                       capture_output=True, text=True, cwd=dst)
    bbl = os.path.join(dst, 'TFG.bbl')
    if r.returncode != 0 or not os.path.exists(bbl):
        sys.exit('  !! la compilacion previa fallo; no puedo extraer el .bbl\n' +
                 '\n'.join(l for l in r.stderr.splitlines() if l.startswith('error'))[:800])
    shutil.copy(bbl, os.path.join(dst, 'bibliografia', 'bibliografia_resuelta.tex'))
    s = open(tfg, encoding='utf-8').read()
    s = s.replace('\\bibliographystyle{plain}\n\n', '')
    s = s.replace('\\bibliography{bibliografia/bibliografia}',
                  '%  Bibliografia ya resuelta por BibTeX: evita que Overleaf tenga que\n'
                  '%  ejecutar BibTeX y dos pasadas extra en cada compilado.\n'
                  '\\input{bibliografia/bibliografia_resuelta}')
    open(tfg, 'w', encoding='utf-8').write(s)

    # ── README propio de la version de Overleaf ──────────────────────────
    open(os.path.join(dst, 'README.md'), 'w', encoding='utf-8').write(LEEME)

    # ── comprobacion final ───────────────────────────────────────────────
    for f in os.listdir(dst):
        if os.path.splitext(f)[1] in ('.aux', '.log', '.toc', '.lof', '.lot',
                                      '.out', '.bbl', '.blg', '.pdf'):
            os.remove(os.path.join(dst, f))
    import time
    t0 = time.time()
    r = subprocess.run([a.tectonic, 'TFG.tex'], capture_output=True, text=True, cwd=dst)
    dt = time.time() - t0
    if r.returncode != 0:
        sys.exit('  !! la version de Overleaf no compila\n' +
                 '\n'.join(l for l in r.stderr.splitlines() if l.startswith('error'))[:800])
    print('  compila en %.0f s' % dt)
    try:
        from pypdf import PdfReader
        print('  %d paginas' % len(PdfReader(os.path.join(dst, 'TFG.pdf')).pages))
    except Exception:
        pass

    # ── empaquetar ───────────────────────────────────────────────────────
    # Ojo: las figuras son .pdf y tienen que ir. Solo se excluye el PDF compilado.
    salta = {'.aux', '.log', '.toc', '.lof', '.lot', '.out', '.blg',
             '.synctex.gz', '.fdb_latexmk', '.fls'}
    with zipfile.ZipFile(a.zip, 'w', zipfile.ZIP_DEFLATED) as z:
        for raiz, _, ficheros in os.walk(dst):
            for f in sorted(ficheros):
                p = os.path.join(raiz, f)
                rel = os.path.relpath(p, dst)
                if os.path.splitext(f)[1] in salta or rel == 'TFG.pdf':
                    continue
                z.write(p, rel)
        nz = len(z.namelist())
    print('  %s: %d ficheros, %.2f MB' % (a.zip, nz, os.path.getsize(a.zip) / 1e6))
    shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    main()
