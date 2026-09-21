# Memoria del TFG

Memoria escrita sobre la **plantilla TFG ETSIDI v3** (Alberto Brunete, UPM): 13 capítulos, 6 anexos
y unas 106 páginas.

## Compilar

- **Overleaf**: sube la carpeta `memoria/` y compila `TFG.tex` (pdfLaTeX + BibTeX).
- **Local**: `latexmk -pdf TFG.tex`, o bien `tectonic TFG.tex`, que no necesita TeX Live instalado.

La estructura, los márgenes y la portada son los de la plantilla. Sobre ella sólo se han añadido los
paquetes que la memoria necesita (`siunitx`, `booktabs`, `tabularx`, `tikz`, `pgfgantt`, `bytefield`
y `listings` configurado), declarados al principio de `TFG.tex`.

## Estructura

```
TFG.tex                  documento principal
capitulos/               portada, firmas, licencia, evaluación, resumen, acrónimos,
                         13 capítulos y 6 anexos
figuras/                 figuras generadas + cabecera.png y Logo_UPM.jpg de la plantilla
bibliografia/            bibliografia.bib (BibTeX)
_v1/                     versión anterior de la memoria, antes de adoptar la plantilla
```

## Figuras y cifras

Las figuras de resultados y los ficheros `datos*.json` se generan a partir de `cycles/*.csv`:

```
pip install matplotlib numpy scipy
python figuras/generar_figuras.py      # campaña preliminar (14-15 sept)
python figuras/generar_figuras21.py    # campaña del 21 de septiembre
```

`figuras/modelo_mecanismo.py` es el modelo del capítulo 4 portado a Python. Sus parámetros
(`L2`, `dx`, `L3`, `L4`, `a0`, `red`, `r0`, `r1`, `barr`) deben coincidir con los de
`mecanismo.json`. Si cambias las cotas, vuelve a ejecutar los dos programas y actualiza a mano las
cifras que cita el texto en los capítulos 4 y 11.

## Pendientes

Dos marcas señalan lo que queda:

- `\completar{...}` (rojo en el PDF): datos que sólo puedes aportar tú (escuela, tutor, marca del
  servo, célula, fotos, precios).
- `\verificar{...}` (naranja): afirmaciones deducidas del código o de los datos que conviene
  confirmar sobre el montaje real. La más importante: **medir las cotas del mecanismo y el perfil de
  la caracola**, de las que dependen todas las cifras del modelo.

Para listarlas:

```
grep -rn "completar{\|verificar{" capitulos TFG.tex
```
