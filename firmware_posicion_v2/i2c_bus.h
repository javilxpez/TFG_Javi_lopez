#pragma once
#include <Arduino.h>

// ══ Bus I2C1 — un solo dueño, y por turnos ════════════════════════
// En estas dos líneas cuelgan dos chips con reglas incompatibles. El ZSC31014 pide
// la medida y la recoge ~2 ms más tarde, y entre esas dos operaciones no puede
// meterse nadie: su protocolo no tolera ni una condición de restart ajena, que le
// rompe la comunicación SIGUIENTE aunque ésta vaya dirigida a la NAU7802.
//
// Por eso el bus no se toca en crudo desde ningún sitio: se pide, se usa y se
// suelta. Quien no lo consigue pierde el turno y lo reintenta en la vuelta siguiente
// del loop; nadie espera con el bus ocupado, para no arrastrar al resto del firmware.
//
// Esto vive en su propia cabecera por dos motivos. Uno de fondo: el árbitro es
// autónomo — no sabe nada de las células, del servo ni del protocolo, sólo de quién
// tiene el bus cogido. Y uno práctico: el IDE de Arduino genera los prototipos de
// todas las funciones del .ino y los inserta por encima de las declaraciones del
// propio fichero, así que un tipo que aparece en una firma tiene que venir de fuera
// o el compilador lo ve antes de conocerlo. Los .h no los toca.

enum I2COwner : uint8_t {
  I2C_FREE = 0,   // libre
  I2C_LC1,        // ZSC31014 — retiene el bus durante toda la conversión
  I2C_LC2,        // NAU7802
  I2C_CFG         // escritura de EEPROM: exclusiva, con el sistema parado
};

static I2COwner i2cOwner  = I2C_FREE;
static uint32_t i2cDenied = 0;    // turnos perdidos por bus ocupado (diagnóstico)

// El bus vive en Core0 y sólo en Core0: Core1 no lo toca ni para leer. Si alguna vez
// se intenta desde allí, se rechaza aquí en vez de corromper una trama a medias.
static inline bool i2cAcquire(I2COwner who){
  if(rp2040.cpuid()!=0)                    { i2cDenied++; return false; }
  if(i2cOwner!=I2C_FREE && i2cOwner!=who)  { i2cDenied++; return false; }
  i2cOwner = who;
  return true;
}

static inline void i2cRelease(I2COwner who){ if(i2cOwner==who) i2cOwner = I2C_FREE; }
