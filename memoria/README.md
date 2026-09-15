# Memoria del TFG

Memoria en LaTeX: 12 capítulos, 6 anexos y unas 106 páginas.

## Compilar

- **Overleaf**: sube la carpeta `memoria/` y compila `main.tex`. Funciona con pdfLaTeX y con XeLaTeX.
- **Local**: `latexmk -pdf main.tex`, o bien `tectonic main.tex`, que no necesita tener TeX Live instalado.

## Figuras

Las figuras de resultados y `figuras/datos.json` se generan a partir de `cycles/*.csv`:

```
pip install matplotlib numpy
python memoria/figuras/generar_figuras.py
```

Si cambias las cotas del mecanismo (`L2`, `L3`, `L4`, masa, barrido) o la calibración de la célula (`CAL`, `OFF`), edítalas al principio del script y vuelve a ejecutarlo. Después hay que actualizar a mano las cifras que cita el texto en los capítulos 3 y 10.

## Pendientes

Busca en el texto las dos marcas de pendientes:

- `\completar{...}` (en rojo en el PDF): datos que sólo puedes aportar tú. Por ejemplo: escuela, grado, tutor, marca del servo, célula de carga, fotos y precios.
- `\verificar{...}` (en naranja): afirmaciones deducidas del código que conviene confirmar sobre el montaje real.

Para comprobar que no queda ninguna antes de entregar:

```
grep -rn "completar{\|verificar{" capitulos anexos main.tex
```
