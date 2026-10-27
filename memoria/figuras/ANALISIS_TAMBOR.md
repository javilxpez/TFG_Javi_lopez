# Análisis del tambor: espiral frente a cilindro

Campaña del 14-10-2026 (10 ensayos, `cycles/20261014-1856*` a `-185848`). Análisis hecho el
14-10-2026 y documentado el 27-10-2026. Todas las cifras de este documento salen de
`analisis_tambor.py`, que escribe `tambor14.json` y `n14_tambor.pdf`.

**En resumen**

- La espiral actual reduce el **par pico un 5,2 %** frente a un cilindro de **igual alcance**, lo
  que equivale a un 5,0 % más de alcance a igual par pico.
- El techo teórico, con par constante en todo el recorrido, es del **14,3 %**, pero ese perfil **no
  se puede construir**. El mejor caso realista, en el que el cable sigue tangente al perfil, se
  estima en torno al **10 %**.
- **Objetivo de diseño**: arrancar con 65–70 mm, bajar a 45–48 mm tan rápido como lo permita el
  cable y seguir **plano** hasta el final. Con eso se llega al **−8,4 %** de par pico, el mismo
  margen que +8,3 % de alcance, y queda pendiente suavizar la unión.

---

## 1. Geometría confirmada

| | |
|---|---|
| Perfil | **sólo espiral lineal**, de 56 a 28,18 mm en **180°** de tambor |
| Arranque | el cable arranca en el extremo de **56 mm** y el radio **baja** al subir la barra |
| Meseta de radio constante | **no hay**: no está en uso |
| Holgura del cable | ≤ 10° de tambor; el ajuste da **9,4°** (0,13 vueltas de motor) |
| Tramo usado | 0° → ~118° de espiral (56 → ~38 mm), unos **100 mm** de cable |

> **Superado:** el modelo «espiral con meseta» de `generar_campana.py` (meseta de 0,17 vueltas,
> unos 61°, a 56 mm seguida de una espiral de 0,367 vueltas) y el texto de
> `capitulos/resultados.tex` que lo usa ($R^2 = 0{,}84$, la meseta de 61°, el arco de unos 50° del
> CAD) **no corresponden a la pieza montada**. La meseta del ajuste absorbía en realidad la holgura
> inicial. Hay que reescribir esa parte del capítulo con esta geometría.

Con la geometría confirmada, el ajuste al par medido da $R^2 = 0{,}80$. Si se dejara libre el
punto de arranque, el ajuste lo pondría a 22° y con 14° de holgura ($R^2 = 0{,}82$), lo que no cambia
las conclusiones.

## 2. Datos

- Se usan **sólo los 10 ensayos del 14-10**. Todos se ajustan igual al modelo ($R^2$ de 0,74 a 0,81
  por ensayo) y con la misma escala de par.
- La campaña del **07-10** tuvo **el cable enganchándose** en algunos ensayos ($R^2$ entre −0,20 y
  0,76, y uno con recorrido corto), así que **no se usa** para estas cifras.

## 3. Métrica: igual alcance

A **igual alcance**, es decir el mismo recorrido de barra en las mismas vueltas de motor, el trabajo
($m g \Delta h$) y por tanto el **par medio** son los mismos con cualquier tambor. Lo único que puede
mejorar un perfil es el **par pico**. La referencia es el **cilindro** que hace ese recorrido en esas
vueltas.

- Un cilindro más pequeño siempre tiene menos par pico, pero también menos alcance: los cilindros
  forman una **línea alcance–par**, que es la de la figura (b).
- La ganancia de un perfil es lo que queda **por debajo de esa línea**. Se puede leer como «x % menos
  par pico a igual alcance» o, lo que es casi lo mismo, «x % más alcance a igual par pico».
- Son cocientes, así que **no dependen de la escala de par** (u. del accionamiento → N·m), que es
  ajustada y tiene una incertidumbre de ~15 %.

Recorrido de referencia: la barra va de −82° a −3°, de colgando hasta casi horizontal.

## 4. Resultados

### 4.1 El par medido y por qué cae al final

| | Inicio (−47°) | Pico | Final (−3°) |
|---|---|---|---|
| Par de gravedad | 1,79 N·m | **1,91 N·m** | 1,70 N·m (**−11 %** desde el pico) |
| Célula (tensión del cable) | 32,9 N | 38,8 N | **42,1 N** (máximo) |
| Brazo efectivo = par / célula | 54 mm | 49 mm | 40 mm |

- El pico cae a **75° de espiral**, que son 85° desde la posición B si se suma la holgura, con la barra
  a unos −23°. Es **breve**: sólo 18° de tambor por encima del 98 % del pico.
- La **tensión sube de forma monótona hasta el final**, así que la carga no baja. Lo que hace caer el
  par es que **la espiral sigue encogiendo el radio**: el brazo medido pasa de 54 a 40 mm.

### 4.2 Ganancia de la espiral actual

| | Vueltas | Alcance | Par pico | Frente al cilindro (r = 48,0 mm) |
|---|---|---|---|---|
| Espiral actual 56→28 | 1,61 | 48,9°/vuelta | 1,88 N·m | **−5,2 % de par pico** / +5,0 % de alcance |

- La ganancia sale del **principio del recorrido** (de colgando a unos −50°): ahí la carga es pequeña
  y el radio grande mueve la barra deprisa.
- En el tramo medido con carga (−47° a −3°) la espiral y el cilindro **quedan igual**: −0,5 % a +1,4 %
  según el ajuste. La ganancia es, por tanto, un resultado **del modelo**, coherente con la medida pero
  no medido directamente.

### 4.3 Techo teórico: par constante (no construible)

Con el mismo alcance, un par constante de **1,70 N·m** daría un **−14,3 %**. Hace falta $r = \tau_0 / T$:

| Barra | −75° | −70° | −60° | −50° | −40° | −20° | −3° |
|---|---|---|---|---|---|---|---|
| Radio necesario | 159 mm | 90 mm | 59 mm | 50 mm | 46 mm | 42 mm | 41 mm |
| Ángulo de presión | **88°** | **82°** | **55°** | 27° | 13° | 3° | 1° |

- Cerca de la posición colgando la tensión tiende a 0 y el radio necesario se dispara.
- Por encima de unos 25–30° de ángulo de presión el cable **puentea el perfil** como una cuerda. Es la
  «forma imposible» que salía en el optimizador de `mecanismo.js`.
- Todo ese 14 % está en los primeros ~30° de barra. Desde −40° el ideal es **casi un cilindro**
  (46 → 41 mm).

### 4.4 Perfiles candidatos

Bajada con una espiral logarítmica a **25° de ángulo de presión**, la más rápida que sigue el cable, y
después plano. Todos se comparan en el mismo recorrido (−82° → −3°).

| Perfil | Vueltas | Alcance | Par pico (dónde) | Par pico frente al cilindro | Alcance frente al cilindro |
|---|---|---|---|---|---|
| Actual 56→28 (180°) | 1,61 | 48,9°/v | 1,88 (−33°) | −5,2 % | +5,0 % |
| **65 → 45 plano** | 1,57 | 50,0°/v | 1,86 (−3°) | **−8,4 %** | **+8,3 %** |
| **70 → 48 plano** | 1,47 | 53,6°/v | 2,01 (−44°) | **−8,4 %** | **+8,3 %** |
| 75 → 53 plano | 1,35 | 58,1°/v | 2,19 (−3°) | −8,8 % | +8,6 % |
| 70 → 45 plano (plano corto) | 1,51 | 52,1°/v | 2,01 (−44°) | −5,7 % | +5,4 % |
| 65 → 40 → 45 (valle) | 1,61 | 48,8°/v | 1,85 (−45°) | −6,5 % | +6,3 % |

## 5. Objetivo de diseño y reglas

**Objetivo**, elegido el 14-10: **entre 65 → 45 y 70 → 48**. Es decir, arranque de 65–70 mm, bajada a
25° hasta 45–48 mm y plano hasta el final. Da −8,4 % de par pico, igual a +8,3 % de alcance, con
50–54° de barra por vuelta de motor y 1,86–2,01 N·m de pico en el tambor. Elegir un punto dentro del
rango es elegir cuánto alcance se quiere a cambio de cuánto par pico: los dos extremos quedan igual de
por debajo de la línea de cilindros.

Reglas que salen del análisis:

1. **Arrancar grande y bajar lo más rápido posible**, limitado por el ángulo de presión (25°).
2. **El radio plano tiene que ir con el de arranque**, aproximadamente $r_\text{plano} \approx 0{,}69\,r_0$.
   Si el plano es demasiado pequeño, el pico lo pone la bajada y el plano desperdicia alcance
   (70 → 45: −5,7 %). Si es demasiado grande, el pico lo pone el final.
3. **Mientras la tensión suba, el radio sólo puede bajar o quedarse plano**. Bajar a 40 y volver a
   subir a 45 es peor que quedarse plano en 45 (−6,5 % frente a −8,4 %): en el valle el par queda por
   debajo del pico, así que se pierde alcance sin bajar el pico.
4. Hacer el radio **simétrico** (que vuelva a crecer) sólo compensa **pasada la horizontal**, donde la
   tensión empieza a bajar. En el banco el recorrido acaba en −3°.
5. Arrancar más grande (75 mm) da más alcance con la misma ganancia relativa. El límite es el tamaño
   del tambor, no el par: el pico de 75 → 53 pide 0,44 N·m en el motor, un 35 % del nominal sin contar
   la fricción.

## 6. Pendiente (paso de optimización)

- [ ] **Suavizar la unión** entre la bajada y el plano. Ahora tiene un pequeño tramo cóncavo que el
  cable puentearía, y suavizarlo costará algo de ganancia.
- [ ] Búsqueda fina dentro de 65–70 mm, en vez de esta rejilla gruesa.
- [ ] Comprobar el perfil final con el contacto exacto y los límites de convexidad y de ángulo de
  presión de `mecanismo.js`.
- [ ] Reescribir la parte de `resultados.tex` que usa la meseta (ver §1).
- [ ] Opcional: confirmar en qué unidades da el par el accionamiento (`par_x10`), para no depender de
  una escala ajustada.

## 7. Limitaciones

- El banco **no mide θ**: el ángulo de la barra sale del modelo de la pieza.
- La escala de par (u. → N·m) se **ajusta** con el modelo, así que los N·m absolutos tienen una
  incertidumbre de ~15 %. Los porcentajes frente al cilindro son cocientes y no dependen de ella.
- El brazo «par / célula» depende además de la calibración provisional de la célula (escala ~1,04).
  Lo que vale es su **tendencia**, no su valor absoluto.
- Los perfiles candidatos son **resultados de modelo** sobre una rejilla gruesa y con la unión sin
  suavizar.
- Ángulos de tambor: este documento los mide **sobre la espiral**, desde que el cable empieza a tirar.
  Para medirlos desde la posición B hay que sumar unos 9°.

## 8. Reproducir

```
python figuras/generar_campana.py      # antes: datos14.json (los 10 ensayos más recientes)
python figuras/analisis_tambor.py      # tambor14.json y n14_tambor.pdf (~1 min)
```

`analisis_tambor.py` añade al modelo un tipo de tambor `valle` (bajada logarítmica y después
lineal o plano) **sin modificar** `modelo_mecanismo.py`.

## 9. Para el artículo

El artículo **no debe nombrar el mecanismo** (nada de «capstan»): hay que hablar de *variable-gain
actuator*. La memoria del TFG sí puede nombrarlo.
