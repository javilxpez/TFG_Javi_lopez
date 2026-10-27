# Memoria del TFG

Memoria escrita sobre la **plantilla TFG ETSIDI v3** (Alberto Brunete, UPM): 13 capítulos, 6 anexos
y unas 106 páginas.

## Compilar

- **Local**: `latexmk -pdf TFG.tex`, o bien `tectonic TFG.tex`, que no necesita TeX Live instalado.
- **Overleaf**: usa el zip `TFG_overleaf.zip` de la raíz del repositorio, no esta carpeta. En el
  plan gratuito el compilado se corta a los pocos segundos, y esta versión tarda unos 16 s: las
  figuras `tikz`/`pgfgantt` se redibujan en cada pasada y BibTeX obliga a tres pasadas de pdfLaTeX.
  El zip lleva esas figuras ya rasterizadas y la bibliografía ya resuelta (`\input` de un `.bbl`
  en vez de `\bibliography`), con lo que baja a unos 5 s dando el mismo PDF. Su propio `README.md`
  explica cómo regenerar la bibliografía si añades referencias.

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
python figuras/modelo_mecanismo.py     # valida el modelo (balance de energía)
python figuras/generar_campana.py      # los 10 ensayos más recientes: figuras n14_* y datos14.json
```

`modelo_mecanismo.py` es un **port fiel de `static/mecanismo.js`**, que es la fuente de verdad del
modelo: cable tangente al tambor, cable equivalente y contacto exacto con el perfil. Si cambias la
geometría, cámbiala en el JS y vuelve a ejecutar el port, que comprueba el balance de energía.

**Tambor: espiral frente a cilindro.** `python figuras/analisis_tambor.py` (después de
`generar_campana.py`) escribe `tambor14.json` y `n14_tambor.pdf`. Las conclusiones, el objetivo de
diseño y las limitaciones están en [`figuras/ANALISIS_TAMBOR.md`](figuras/ANALISIS_TAMBOR.md). Ojo: la
geometría confirmada es **sólo la espiral** de 56 a 28,18 mm en 180°, **sin meseta**, así que la
«espiral con meseta» de `generar_campana.py` y la parte de `resultados.tex` que la usa están
superadas.

El modelo anterior (salida del cable en un punto fijo con un desplazamiento `dx`) está superado, y
con él las conclusiones de las campañas de septiembre. `generar_figuras.py` sigue en el repositorio
pero usa ese modelo viejo: sus cifras no se citan en la memoria.

La versión para Overleaf se genera con:

```
python figuras/generar_overleaf.py
```

que rasteriza los diagramas, incrusta la bibliografía resuelta y deja `TFG_overleaf.zip` en la raíz
del repositorio.

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
