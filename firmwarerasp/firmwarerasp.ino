// ═══════════════════════════════════════════════════════════
//  ZSC31014 Load Cell + Servo Motor — Raspberry Pi Pico 2
//  Binary telemetry + command protocol
//  Dual-core: Core0 = LC/telemetry/commands, Core1 = Modbus
// ═══════════════════════════════════════════════════════════

#include <Wire.h>
#include <ModbusMaster.h>
#include <SerialPIO.h>   // PIO-based UART: permite TX/RX en cualquier GPIO

// I2C1 en GP2 (SDA) / GP3 (SCL) — Wire1 no disponible en este core, usar TwoWire
static TwoWire I2C1Bus(i2c1, 2, 3);

// ── Pin Assignments ──────────────────────────────
#define LC_SDA_PIN   2    // GP2 (I2C1 SDA)
#define LC_SCL_PIN   3    // GP3 (I2C1 SCL)
#define LC_I2C_FREQ  100000

#define MB_TX_GPIO   14   // GP14 (DI → RS485, via SerialPIO)
#define MB_RX_GPIO   13   // GP13 (RO ← RS485, via SerialPIO)
#define MB_EN_GPIO   12   // GP12 (DE/RE direction control)

// ── Device Addresses ─────────────────────────────
#define ZSC31014_ADDR 0x28

// ── Load Cell Status Bits ────────────────────────
#define LC_STATUS_VALID   0x00
#define LC_STATUS_STALE   0x01
#define LC_STATUS_COMMAND 0x02

// ── Binary Protocol ──────────────────────────────
#define SYNC_0      0xAA
#define SYNC_1      0x55
#define PACKET_ID   0x02  // v2: load cell + motor

// Command IDs (Python → Pico)
#define CMD_START     0x01  // payload: uint8 mode (0=speed, 1=torque)
#define CMD_STOP      0x02  // no payload
#define CMD_SET_PARAM 0x03  // payload: uint8 param_id, int16_le value

// Param IDs for CMD_SET_PARAM
#define PARAM_TORQUE_REF     0x01  // torque x10 (e.g. 40 = 4.0%)
#define PARAM_SPEED_REF      0x02  // RPM
#define PARAM_TORQUE_LIM_POS 0x03
#define PARAM_TORQUE_LIM_NEG 0x04

// ── Modbus Register Addresses ────────────────────
#define MB_REG_CONTROL_MODE    0      // 1=speed, 2=torque
#define MB_REG_SPEED_REF       801    // Speed reference (RPM)
#define MB_REG_TORQUE_REF      833    // Torque reference (×10 %)
#define MB_REG_TORQUE_LIM_POS  839    // Torque limit positive
#define MB_REG_TORQUE_LIM_NEG  840    // Torque limit negative
#define MB_REG_SERVO_ENABLE    1041   // Servo ON/OFF (C04.11)
#define MB_REG_MON_RPM         16385  // Monitoring: motor RPM
#define MB_REG_MON_TORQUE      16387  // Monitoring: motor torque ×10

// ── Braking Resistor Configuration ───────────────
#define MB_REG_BRAKE_RES_SEL   16    // C00_10: resistor selection (1=external)
#define MB_REG_BRAKE_RES_POW   17    // C00_11: resistor power (W)
#define MB_REG_BRAKE_RES_OHM   18    // C00_12: resistor ohmic value (Ω)
#define MB_REG_BRAKE_RES_DISS  19    // C00_13: resistor dissipation (%)

#define BRAKE_RES_SEL_VAL      1     // External resistor
#define BRAKE_RES_POW_VAL      1000  // 1000 W
#define BRAKE_RES_OHM_VAL      235   // 235 Ω
#define BRAKE_RES_DISS_VAL     30    // Resistor dissipation (%)

// ── Full Parameter Register Map ──────────────────
// Register = group_number * 256 + hex(sub_parameter)
// Parameters already named above are skipped here.

// C00 — General Parameters (base 0x000 = 0)
// MB_C00_00  → MB_REG_CONTROL_MODE
#define MB_C00_01   1    // Motor rotating direction
#define MB_C00_02   2    // Pulses per revolution
#define MB_C00_04   4    // Auto-tuning mode
#define MB_C00_05   5    // Stiffness level
#define MB_C00_06   6    // Load inertia ratio
#define MB_C00_07   7    // Absolute mode
// MB_C00_10–13 → MB_REG_BRAKE_RES_*
#define MB_C00_14   20   // Brake enable switch
#define MB_C00_16   22   // Panel display
#define MB_C00_17   23   // Fully closed-loop disconnection detection
#define MB_C00_18   24   // Filter time of fully closed-loop disconnection detection
#define MB_C00_19   25   // Filter time of fully closed-loop hardware
#define MB_C00_1A   26   // Select ABZ port
#define MB_C00_1B   27   // Select OCZ port
#define MB_C00_20   32   // Select pulse input mode
#define MB_C00_21   33   // Select pulse effective edge
#define MB_C00_22   34   // Select pulse channel
#define MB_C00_24   36   // Low-speed pulse input filter time
#define MB_C00_25   37   // High-speed pulse input filter time
#define MB_C00_27   39   // Frequency division output phase
#define MB_C00_28   40   // Frequency division output pulses
#define MB_C00_2A   42   // Select pulse Z output polarity
#define MB_C00_30   48   // General user
#define MB_C00_31   49   // Super user

// C01 — Basic Gain Parameters (base 0x100 = 256)
#define MB_C01_00   256  // 1st position loop gain
#define MB_C01_01   257  // 1st speed loop gain
#define MB_C01_02   258  // 1st speed loop integral time parameter
#define MB_C01_03   259  // 1st torque reference filter cutoff frequency
#define MB_C01_08   264  // 2nd position loop gain
#define MB_C01_09   265  // 2nd speed loop gain
#define MB_C01_0A   266  // 2nd speed loop integral time parameter
#define MB_C01_0B   267  // 2nd torque reference filter cutoff frequency
#define MB_C01_10   272  // Speed feedback filter
#define MB_C01_11   273  // Cutoff frequency of speed feedback low-pass filter
#define MB_C01_12   274  // Speed feedback overlapping average filter time constant
#define MB_C01_13   275  // Speed feedforward source
#define MB_C01_14   276  // Speed feedforward percentage
#define MB_C01_15   277  // Speed feedforward filter cutoff frequency
#define MB_C01_16   278  // Torque feedforward source
#define MB_C01_17   279  // Torque feedforward percentage
#define MB_C01_18   280  // Torque feedforward filter cutoff frequency
#define MB_C01_1B   283  // PDFF control coefficient
#define MB_C01_1C   284  // Damping factor control coefficient
#define MB_C01_20   288  // Position reference overlapping average filter time constant A
#define MB_C01_21   289  // Position reference overlapping average filter time constant B
#define MB_C01_22   290  // Position reference low-pass filter time constant A
#define MB_C01_23   291  // Position reference low-pass filter time constant B
#define MB_C01_24   292  // 1st notch filter frequency of position reference
#define MB_C01_25   293  // 1st notch filter width of position reference
#define MB_C01_26   294  // 1st notch filter depth of position reference
#define MB_C01_27   295  // 2nd notch filter frequency of position reference
#define MB_C01_28   296  // 2nd notch filter width of position reference
#define MB_C01_29   297  // 2nd notch filter depth of position reference
#define MB_C01_2A   298  // Position reference pre-charge filter time constant
#define MB_C01_30   304  // Adaptive notch mode
#define MB_C01_31   305  // Adaptive notch test times
#define MB_C01_32   306  // Adaptive notch test frequency
#define MB_C01_33   307  // Adaptive notch test amplitude
#define MB_C01_34   308  // Adaptive notch target torque
#define MB_C01_35   309  // Adaptive notch test torque threshold
#define MB_C01_38   312  // Gain switchover mode
#define MB_C01_39   313  // Gain switchover time
#define MB_C01_3A   314  // Gain switchover threshold
#define MB_C01_3B   315  // Gain switchover loop width
#define MB_C01_3C   316  // Gain switchover position lock setting
#define MB_C01_3D   317  // Gain switchover position gain conversion time
#define MB_C01_40   320  // Frequency of the 1st notch
#define MB_C01_41   321  // Width level of the 1st notch
#define MB_C01_42   322  // Depth level of the 1st notch
#define MB_C01_43   323  // Frequency of the 2nd notch
#define MB_C01_44   324  // Width level of the 2nd notch
#define MB_C01_45   325  // Depth level of the 2nd notch
#define MB_C01_46   326  // Frequency of the 3rd notch
#define MB_C01_47   327  // Width level of the 3rd notch
#define MB_C01_48   328  // Depth level of the 3rd notch
#define MB_C01_49   329  // Frequency of the 4th notch
#define MB_C01_4A   330  // Width level of the 4th notch
#define MB_C01_4B   331  // Depth level of the 4th notch
#define MB_C01_4C   332  // Frequency of the 5th notch
#define MB_C01_4D   333  // Width level of the 5th notch
#define MB_C01_4E   334  // Depth level of the 5th notch

// C02 — Advanced Gain Parameters (base 0x200 = 512)
#define MB_C02_00   512  // Model tracking control
#define MB_C02_01   513  // Model tracking control gain
#define MB_C02_02   514  // Model tracking inertia correction coefficient
#define MB_C02_08   520  // Backlash compensation mode
#define MB_C02_09   521  // Number of backlash compensation pulses
#define MB_C02_0B   523  // Backlash compensation filter time constant
#define MB_C02_30   560  // Speed observer gain
#define MB_C02_31   561  // Speed observer inertia correction
#define MB_C02_32   562  // Speed observer speed feedback cutoff frequency
#define MB_C02_38   568  // Frequency for vibration suppression 1
#define MB_C02_39   569  // Inertia correction for vibration suppression 1
#define MB_C02_3A   570  // Low-pass filter correction for vibration suppression 1
#define MB_C02_3B   571  // Correction of high-pass filter 1 for vibration suppression 1
#define MB_C02_3C   572  // Frequency of high-pass filter 2 for vibration suppression 1
#define MB_C02_3D   573  // Ratio of compensation 1 for vibration suppression 1
#define MB_C02_3E   574  // Ratio of compensation 2 for vibration suppression 1
#define MB_C02_60   608  // Disturbance observer gain
#define MB_C02_61   609  // Disturbance observer inertia correction coefficient
#define MB_C02_62   610  // Disturbance observer low-pass cutoff frequency
#define MB_C02_63   611  // Disturbance observer compensation torque percentage
#define MB_C02_68   616  // Friction compensation switch and relevant setting
#define MB_C02_69   617  // Friction compensation speed threshold
#define MB_C02_6A   618  // Static friction compensation
#define MB_C02_6B   619  // Forward friction compensation of coulomb friction
#define MB_C02_6C   620  // Reverse friction compensation of coulomb friction
#define MB_C02_6D   621  // Viscous friction torque for rated speed
#define MB_C02_6E   622  // Friction compensation filter time
#define MB_C02_6F   623  // Friction compensation threshold for zero speed

// C03 — Instruction Parameters (base 0x300 = 768)
#define MB_C03_00   768  // Select positioning reference
#define MB_C03_02   770  // Group 1 numerator of electronic gear ratio
#define MB_C03_04   772  // Group 1 denominator of electronic gear ratio
#define MB_C03_06   774  // Group 2 numerator of electronic gear ratio
#define MB_C03_08   776  // Group 2 denominator of electronic gear ratio
#define MB_C03_0B   779  // Gear ratio switchover mode
#define MB_C03_0C   780  // Step size reference
#define MB_C03_0D   781  // Step size in speed reference
#define MB_C03_10   784  // Condition for position reach
#define MB_C03_11   785  // Positioning near threshold
#define MB_C03_12   786  // Position reach threshold
#define MB_C03_13   787  // Position reach filter time
#define MB_C03_14   788  // Position reach holding time
#define MB_C03_16   790  // Select position deviation clearance
#define MB_C03_20   800  // Select speed reference
// MB_C03_21 → MB_REG_SPEED_REF
#define MB_C03_22   802  // Acceleration rate
#define MB_C03_24   804  // Deceleration rate
#define MB_C03_26   806  // Select speed limiting
#define MB_C03_27   807  // Internal positive speed limit
#define MB_C03_28   808  // Internal negative speed limit
#define MB_C03_29   809  // External positive speed limit
#define MB_C03_2A   810  // External negative speed limit
#define MB_C03_2B   811  // Velocity window
#define MB_C03_2C   812  // Speed synchronization threshold
#define MB_C03_2D   813  // Speed rotation threshold
#define MB_C03_2E   814  // Zero speed output threshold
#define MB_C03_2F   815  // Zero clamp threshold
#define MB_C03_30   816  // Forward DI jog velocity
#define MB_C03_31   817  // Negative DI jog velocity
#define MB_C03_32   818  // Zero-speed output hysteresis
#define MB_C03_33   819  // Speed DO filter
#define MB_C03_34   820  // Speed S-curve enable
#define MB_C03_35   821  // Increasing acceleration of speed S-curve acceleration
#define MB_C03_36   822  // Increasing deceleration of speed S-curve acceleration
#define MB_C03_37   823  // Decreasing acceleration of speed S-curve deceleration
#define MB_C03_38   824  // Decreasing deceleration of speed S-curve deceleration
#define MB_C03_40   832  // Torque reference selection
// MB_C03_41 → MB_REG_TORQUE_REF
#define MB_C03_42   834  // Select torque limiting
#define MB_C03_43   835  // Internal positive torque limit
#define MB_C03_44   836  // Internal negative torque limit
#define MB_C03_45   837  // External positive torque limit
#define MB_C03_46   838  // External negative torque limit
// MB_C03_47 → MB_REG_TORQUE_LIM_POS
// MB_C03_48 → MB_REG_TORQUE_LIM_NEG
#define MB_C03_49   841  // Reference value for torque reach
#define MB_C03_4A   842  // Valid value for torque reached
#define MB_C03_4B   843  // Invalid value for torque reached

// C05 — Stop Parameters (base 0x500 = 1280)
#define MB_C05_00   1280  // Stop mode at S-ON off
#define MB_C05_01   1281  // Quick stop mode
#define MB_C05_02   1282  // Stop mode at overtravel
#define MB_C05_03   1283  // Stop mode at No. 1 fault
#define MB_C05_04   1284  // Stop mode at No. 2 fault
#define MB_C05_07   1287  // Stop speed threshold
#define MB_C05_08   1288  // Deceleration time for ramp to quick stop
#define MB_C05_0A   1290  // Deceleration time for ramp to slow stop
#define MB_C05_0C   1292  // Limit for stop at emergency-stop torque
#define MB_C05_10   1296  // Delay from brake close to motor de-energized
#define MB_C05_11   1297  // Speed threshold at brake closing
#define MB_C05_12   1298  // Maximum waiting time with S-ON off at brake closing
#define MB_C05_13   1299  // Delay from brake on to command received
#define MB_C05_14   1300  // Energizing delay of DB relay

// C06 — Protection Parameters (base 0x600 = 1536)
#define MB_C06_00   1536  // Threshold of position deviation excess
#define MB_C06_02   1538  // Threshold of excessive position reference frequency
#define MB_C06_03   1539  // Threshold of excessive speed
#define MB_C06_04   1540  // Input phase loss detection
#define MB_C06_05   1541  // Retentive at power failure
#define MB_C06_06   1542  // STO function shielding
#define MB_C06_07   1543  // Mechanical limit position
#define MB_C06_08   1544  // Mechanical PL
#define MB_C06_0A   1546  // Mechanical NL
#define MB_C06_0D   1549  // Three-phase 220V to two-phase alarm switch
#define MB_C06_0E   1550  // STO handling method
#define MB_C06_10   1552  // Drive overload protection threshold
#define MB_C06_11   1553  // Motor overload protection threshold
#define MB_C06_12   1554  // Motor locked-rotor detection
#define MB_C06_13   1555  // Motor locked-rotor detection time
#define MB_C06_14   1556  // Motor locked-rotor detection speed
#define MB_C06_15   1557  // Output phase loss detection
#define MB_C06_16   1558  // PTC enabling
#define MB_C06_17   1559  // Motor overload current coefficient
#define MB_C06_18   1560  // IGBT over-temperature protection threshold
#define MB_C06_19   1561  // Discharge conduit over-temperature protection threshold
#define MB_C06_1A   1562  // Encoder over-temperature protection threshold
#define MB_C06_1B   1563  // Encoder multi-turn overflow protection
#define MB_C06_1C   1564  // Drive high temperature warning threshold
#define MB_C06_1D   1565  // Output phase loss detection threshold
#define MB_C06_20   1568  // Protection from out of control (0=off, 1=on)
#define MB_C06_21   1569  // Torque threshold for protection from out of control
#define MB_C06_22   1570  // Speed threshold for protection from out of control
#define MB_C06_23   1571  // Speed filter time for protection from out of control
#define MB_C06_24   1572  // Detection time for protection from out of control
#define MB_C06_25   1573  // Motor over-temperature protection
#define MB_C06_26   1574  // Motor over-temperature protection threshold
#define MB_C06_27   1575  // STO 24V disconnection filter time
#define MB_C06_28   1576  // Servo OFF delay after STO triggered
#define MB_C06_29   1577  // SizeCDE electromagnetic shorting
#define MB_C06_2B   1579  // STO PWM buffer detection method
#define MB_C06_2C   1580  // Shielded power-off discharge

// C07 — Auto-tuning Parameters (base 0x700 = 1792)
#define MB_C07_00   1792  // Offline inertia auto-tuning mode setting
#define MB_C07_01   1793  // Offline inertia auto-tuning speed reference
#define MB_C07_02   1794  // Acceleration/deceleration time for offline inertia auto-tuning
#define MB_C07_03   1795  // Offline inertia auto-tuning target torque
#define MB_C07_04   1796  // Offline inertia auto-tuning revolutions
#define MB_C07_08   1800  // Online inertia auto-tuning mode setting
#define MB_C07_09   1801  // Online inertia auto-tuning parameter setting
#define MB_C07_28   1832  // Low-speed area threshold in friction identification reference
#define MB_C07_29   1833  // Number of target velocities in friction identification reference
#define MB_C07_2A   1834  // Maximum speed in friction identification reference
#define MB_C07_2B   1835  // Number of velocities in low-speed mode in friction identification
#define MB_C07_2C   1836  // Acceleration/deceleration time in friction identification
#define MB_C07_2D   1837  // Constant-velocity operation time in friction identification
#define MB_C07_2E   1838  // Stop time in friction identification reference
#define MB_C07_2F   1839  // Maximum running distance of friction identification

// C0A — Communication Parameters (base 0xA00 = 2560)
#define MB_C0A_00   2560  // Modbus communication station number
#define MB_C0A_01   2561  // Modbus communication baud rate
#define MB_C0A_02   2562  // Modbus communication format
#define MB_C0A_03   2563  // Modbus communication response time
#define MB_C0A_05   2565  // Select Modbus communication storage
#define MB_C0A_06   2566  // Modbus data format
#define MB_C0A_08   2568  // Commissioning software communication station ID
#define MB_C0A_09   2569  // Commissioning software communication baud rate
#define MB_C0A_0A   2570  // Commissioning software communication format
#define MB_C0A_0B   2571  // Commissioning software communication response time
#define MB_C0A_0D   2573  // Commissioning software communication storage
#define MB_C0A_0E   2574  // Commissioning software data format
#define MB_C0A_12   2578  // USB display switch

// C10 — Homing / Touch Probe Parameters (base 0x1000 = 4096)
#define MB_C10_00   4096  // Homing enable
#define MB_C10_01   4097  // Select homing mode
#define MB_C10_02   4098  // Initial homing speed
#define MB_C10_03   4099  // End homing speed
#define MB_C10_04   4100  // Homing acceleration time
#define MB_C10_06   4102  // Homing deceleration time
#define MB_C10_08   4104  // Homing timeout interval
#define MB_C10_0A   4106  // Homing offset mode
#define MB_C10_0B   4107  // Homing offset distance
#define MB_C10_10   4112  // Multi-turn absolute position offset (low 32 bits)
#define MB_C10_12   4114  // Multi-turn absolute position offset (high 32 bits)
#define MB_C10_14   4116  // Multi-turn revolutions data offset
#define MB_C10_15   4117  // Multi-turn overflow flag
#define MB_C10_16   4118  // Reference running mode in rotation mode
#define MB_C10_18   4120  // Numerator of electronic gear ratio in rotation mode
#define MB_C10_19   4121  // Denominator of electronic gear ratio in rotation mode
#define MB_C10_1A   4122  // Upper limit of mechanical absolute position in rotation mode (low)
#define MB_C10_1C   4124  // Upper limit of mechanical absolute position in rotation mode (high)
#define MB_C10_1E   4126  // Single-turn homing absolute value offset
#define MB_C10_20   4128  // Probe capture
#define MB_C10_21   4129  // Probe capture state released
#define MB_C10_22   4130  // Probe capture offset distance
#define MB_C10_24   4132  // Probe capture speed
#define MB_C10_25   4133  // Probe capture acceleration time
#define MB_C10_27   4135  // Probe capture deceleration time
#define MB_C10_2C   4140  // Filter time of probe capture pin
#define MB_C10_30   4144  // Torque limit of homing upon hit-and-stop
#define MB_C10_31   4145  // Speed for homing upon hit-and-stop
#define MB_C10_32   4146  // Number of times for homing upon hit-and-stop
#define MB_C10_33   4147  // Positioning threshold equivalent upon homing completed
#define MB_C10_34   4148  // Mode of homing upon hit-and-stop

// F31 —[Reset Parameters]
#define MB_F31_00   12544  // Fault reset        — clears active alarms after root cause is fixed
#define MB_F31_01   12545  // Program reset      — restarts internal firmware without power cycling
#define MB_F31_02   12546  // Parameter init     — ⚠ FACTORY RESET: restores all params to default (clears station ID & baud)
#define MB_F31_04   12548  // Fault record init  — clears saved fault history from drive memory
#define MB_F31_10   12560  // Encoder data reset — resets encoder position data (absolute encoders / after maintenance)

// ── Error Codes ─────────────────────────────────
#define ERR_NONE              0x00
#define ERR_LC_STALE_TIMEOUT  0x01  // Load cell stuck returning stale data >50ms
#define ERR_LC_I2C_FAIL       0x02  // I2C read returned no bytes
#define ERR_LC_NOT_FOUND      0x03  // Load cell not detected at startup
#define ERR_SERVO_CONFIG_FAIL 0x04  // Servo configuration write failed
#define ERR_SERVO_SCAN_FAIL   0x05  // No servo found on bus (full scan)

// ── Load Cell AFE Configuration ──────────────────
// Change these, reflash, and power-cycle ZSC31014 to apply.
// Gain: 000=1.5, 100=3, 001=6, 101=12, 010=24(default), 110=48, 011=96, 111=192
#define LC_PREAMP_GAIN      0b111
// A2D_Offset: 0x1-0xF (0x8=center/default; 0x0=reserved for temp, don't use)
#define LC_A2D_OFFSET       0x08
// Gain polarity: 0=negative, 1=positive(default)
#define LC_GAIN_POLARITY    1
// Nulling: 0=enabled(default, use for gain>=6), 1=disabled(for gain<6)
#define LC_DISABLE_NULLING  0

// ── State Machine ────────────────────────────────
enum State {
  STATE_IDLE,        // Waiting for Python START command
  STATE_SCANNING,    // Scanning Modbus bus
  STATE_CONFIGURING, // Setting up servo
  STATE_RUNNING,     // Main loop: read + telemetry + motor
  STATE_ERROR        // Something went wrong
};

enum LCPhase { LC_REQUEST, LC_WAIT };

struct ControlState {
  // System
  State    currentState    = STATE_IDLE;
  uint8_t  errorCode       = ERR_NONE;
  bool     modeTorque      = true;

  // Timing
  unsigned long lastActionTime    = 0;
  unsigned long lastTelemetryTime = 0;

  // Servo
  uint8_t  servoID           = 0;
  uint8_t  scanID            = 1;
  bool     servoConnected    = false;
  unsigned long scanStartTime      = 0;
  uint8_t  configureRetries        = 0;
  int16_t  motorRPM          = 0;
  int16_t  motorTorqueX10    = 0;  // torque * 10

  // Load cell
  bool     loadCellOK        = false;
  bool     lcConfigApplied   = false;
  LCPhase  lcPhase           = LC_REQUEST;
  unsigned long lcRequestTime = 0;
  uint16_t lastBridge        = 0;
  uint8_t  lastLCStatus      = 0;
  bool     lastLCValid       = false;

  // Servo tuning
  int16_t  stiffness         = 3;   // C00_05: stiffness level (0–31)
  int16_t  accelRate         = 3;   // C03_22: acceleration rate
  int16_t  decelRate         = 3;   // C03_24: deceleration rate

  // Torque & speed limits
  int16_t  torqueLimIntPos   = 500;  // C03_43: internal positive torque limit (×10 %)
  int16_t  torqueLimIntNeg   = 500;  // C03_44: internal negative torque limit (×10 %)
  int16_t  speedLimTorqPos   = 1000; // C03_47: positive speed limit in torque mode (RPM)
  int16_t  speedLimTorqNeg   = 1000; // C03_48: negative speed limit in torque mode (RPM)

  // Controller
  int16_t  zeroOffset        = 8210; // bridge value at zero load
  int16_t  baseRead          = 0;    // lastBridge - zeroOffset (raw)
  int16_t  deadband          = -5;   // below this baseRead, output is zero
  int16_t  thsld             = 320;  // added to scaled output
  int16_t  gain              = 9;    // baseRead multiplier
  int16_t  servoRef          = 0;    // last reference written to servo

  // Moving average filter on raw bridge reading
  static const uint8_t MA_SIZE = 12;
  uint16_t maBuf[12]         = {};
  uint8_t  maIdx             = 0;
  uint32_t maSum             = 0;
  uint16_t lastBridgeFilt    = 0;    // MA-filtered bridge value
};

static ControlState ctrl;

// ── Core1 Modbus worker (shared volatile state) ──
// Core0 writes the "input" fields; Core1 writes the "output" fields.
// Only one core touches Serial485/servo at a time:
//   - Core1 active while mbw.active == true (STATE_RUNNING)
//   - Core0 takes over after setting mbw.active = false and waiting for mbw.busy == false
struct MBWorker {
  volatile bool    active      = false; // Core0 → Core1: enable/disable
  volatile bool    busy        = false; // Core1 → Core0: currently inside a Modbus call
  volatile uint8_t slaveID     = 0;
  volatile bool    modeTorque  = true;
  volatile int16_t refValue    = 0;     // reference to write every cycle

  volatile bool    connected   = false; // last write succeeded
  volatile uint8_t failCount   = 0;     // consecutive write failures
  volatile int16_t motorRPM    = 0;
  volatile int16_t motorTorqueX10 = 0;
} mbw;

// Stop Core1 Modbus and wait until it finishes its current operation (max 100 ms)
static void stopCore1() {
  mbw.active = false;
  unsigned long t = millis();
  while (mbw.busy && millis() - t < 100) delayMicroseconds(100);
}

// ── Modbus ───────────────────────────────────────
// SerialPIO: UART por PIO, funciona en cualquier GPIO (necesario para GP14 TX)
static SerialPIO Serial485(MB_TX_GPIO, MB_RX_GPIO);

ModbusMaster servo;

void preTransmission()  { digitalWrite(MB_EN_GPIO, HIGH); }
void postTransmission() { delayMicroseconds(200); digitalWrite(MB_EN_GPIO, LOW); }

// ── Core1: Modbus write loop ──────────────────────
// Runs continuously while mbw.active, writing refValue and reading monitoring.
// Never called from Core0. Serial485 and servo are exclusively Core1's during RUNNING.
void setup1() {
  // Core1 starts after Core0 has already initialized Serial485 in servoInit()
}

void loop1() {
  if (!mbw.active || mbw.slaveID == 0) {
    delay(1);
    return;
  }

  mbw.busy = true;

  servo.begin(mbw.slaveID, Serial485);
  Serial485.setTimeout(20);

  uint8_t r = mbw.modeTorque
    ? servo.writeSingleRegister(MB_REG_TORQUE_REF, (uint16_t)mbw.refValue)
    : servo.writeSingleRegister(MB_REG_SPEED_REF,  (uint16_t)mbw.refValue);

  if (r == 0) {
    mbw.failCount  = 0;
    mbw.connected  = true;

    static uint8_t monCnt = 0;
    if (++monCnt >= 10) {
      monCnt = 0;
      if (servo.readHoldingRegisters(MB_REG_MON_RPM, 1) == 0)
        mbw.motorRPM = (int16_t)servo.getResponseBuffer(0);
      if (servo.readHoldingRegisters(MB_REG_MON_TORQUE, 1) == 0)
        mbw.motorTorqueX10 = (int16_t)servo.getResponseBuffer(0);
    }
  } else {
    if (++mbw.failCount >= 5) {
      mbw.connected      = false;
      mbw.motorRPM       = 0;
      mbw.motorTorqueX10 = 0;
    }
  }

  mbw.busy = false;
}

// ── CRC8 ─────────────────────────────────────────
static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// ═════════════════════════════════════════════════
//  TELEMETRY (Pico → Python)
// ═════════════════════════════════════════════════
// Packet v2 payload (19 bytes):
//   0     packet_id     uint8
//   1-4   t_ms          uint32 LE
//   5-6   bridge        uint16 LE
//   7     lc_status     uint8
//   8     lc_flags      uint8   (bit0=valid, bit1=servo_connected, bit2=config_ok; or error_code when state=ERROR)
//   9-10  rpm           int16 LE
//   11-12 torque_x10    int16 LE
//   13    servo_state   uint8
//   14    mode          uint8   (0=speed, 1=torque)
//   15-16 servo_ref     int16 LE  (last reference written to servo)
//   17-18 base_read     int16 LE  (bridge - zeroOffset)

static uint8_t txBuf[64];

void telemetrySend(uint32_t t, uint16_t bridge, uint8_t lcStatus, bool lcValid,
                   int16_t rpm, int16_t torqueX10,
                   uint8_t state, uint8_t mode) {
  uint8_t payloadLen = 19;
  uint8_t *p = txBuf;

  *p++ = PACKET_ID;
  *p++ = (t >>  0) & 0xFF;
  *p++ = (t >>  8) & 0xFF;
  *p++ = (t >> 16) & 0xFF;
  *p++ = (t >> 24) & 0xFF;
  *p++ = (bridge >> 0) & 0xFF;
  *p++ = (bridge >> 8) & 0xFF;
  *p++ = lcStatus;
  // lc_flags: bit0=valid, bit1=servo_connected, bit2=config_ok; or error_code when state == ERROR
  uint8_t flags;
  if (state == STATE_ERROR) {
    flags = ctrl.errorCode;
  } else {
    flags = (lcValid ? 0x01 : 0x00) | (ctrl.servoConnected ? 0x02 : 0x00) | (ctrl.lcConfigApplied ? 0x04 : 0x00);
  }
  *p++ = flags;
  *p++ = (rpm >>  0) & 0xFF;
  *p++ = (rpm >>  8) & 0xFF;
  *p++ = (torqueX10 >> 0) & 0xFF;
  *p++ = (torqueX10 >> 8) & 0xFF;
  *p++ = state;
  *p++ = mode;
  *p++ = (ctrl.servoRef >> 0) & 0xFF;
  *p++ = (ctrl.servoRef >> 8) & 0xFF;
  *p++ = (ctrl.baseRead >> 0) & 0xFF;
  *p++ = (ctrl.baseRead >> 8) & 0xFF;

  uint8_t frame[3 + payloadLen + 1];
  frame[0] = SYNC_0;
  frame[1] = SYNC_1;
  frame[2] = payloadLen;
  memcpy(&frame[3], txBuf, payloadLen);
  frame[3 + payloadLen] = crc8(txBuf, payloadLen);

  Serial.write(frame, sizeof(frame));
}

// ═════════════════════════════════════════════════
//  COMMAND RECEPTION (Python → Pico)
// ═════════════════════════════════════════════════
static uint8_t rxBuf[64];
static uint8_t rxPos = 0;
static enum { RX_SYNC0, RX_SYNC1, RX_LEN, RX_PAYLOAD } rxState = RX_SYNC0;
static uint8_t rxLen = 0;

void processCommand(const uint8_t *payload, uint8_t len) {
  if (len < 1) return;
  uint8_t cmd = payload[0];

  switch (cmd) {
    case CMD_START:
      if (ctrl.currentState == STATE_ERROR) break;  // refuse if halted
      if (len >= 2) {
        ctrl.modeTorque = (payload[1] != 0);
        ctrl.scanID = 1;
        ctrl.servoID = 0;
        ctrl.scanStartTime = millis();
        ctrl.configureRetries = 0;
        ctrl.currentState = STATE_SCANNING;
      }
      break;

    case CMD_STOP:
      // Stop Core1 first so we can safely use Serial485 from Core0
      if (mbw.active) stopCore1();
      if (ctrl.servoID > 0) {
        servo.begin(ctrl.servoID, Serial485);
        servo.writeSingleRegister(MB_REG_SERVO_ENABLE, 0);  // Servo OFF
      }
      ctrl.servoConnected = false;
      ctrl.motorRPM = 0;
      ctrl.motorTorqueX10 = 0;
      ctrl.currentState = STATE_IDLE;
      break;

    case CMD_SET_PARAM:
      // Only limit writes make sense here; ref is driven by the control loop in Core1
      if (len >= 4 && ctrl.servoID > 0) {
        uint8_t paramID = payload[1];
        int16_t value = (int16_t)(payload[2] | (payload[3] << 8));
        if (paramID == PARAM_TORQUE_LIM_POS || paramID == PARAM_TORQUE_LIM_NEG) {
          bool wasActive = mbw.active;
          if (wasActive) stopCore1();
          servo.begin(ctrl.servoID, Serial485);
          Serial485.setTimeout(100);
          if (paramID == PARAM_TORQUE_LIM_POS) servo.writeSingleRegister(MB_REG_TORQUE_LIM_POS, value);
          else                                  servo.writeSingleRegister(MB_REG_TORQUE_LIM_NEG, value);
          if (wasActive) { Serial485.setTimeout(20); mbw.active = true; }
          else           { Serial485.setTimeout(100); }
        }
      }
      break;
  }
}

void checkCommands() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    // Debug shortcut: 's' = CMD_START torque, 'x' = CMD_STOP
    if (b == 's' && ctrl.currentState != STATE_ERROR) {
      ctrl.modeTorque = true;
      ctrl.scanID = 1;
      ctrl.servoID = 0;
      ctrl.scanStartTime = millis();
      ctrl.configureRetries = 0;
      ctrl.currentState = STATE_SCANNING;
      Serial.println("# [s] START torque mode");
      continue;
    }
    if (b == 'x') {
      if (mbw.active) stopCore1();
      if (ctrl.servoID > 0) {
        servo.begin(ctrl.servoID, Serial485);
        servo.writeSingleRegister(MB_REG_SERVO_ENABLE, 0);
      }
      ctrl.servoConnected = false;
      ctrl.motorRPM = 0;
      ctrl.motorTorqueX10 = 0;
      ctrl.currentState = STATE_IDLE;
      Serial.println("# [x] STOP");
      continue;
    }

    switch (rxState) {
      case RX_SYNC0:
        if (b == SYNC_0) rxState = RX_SYNC1;
        break;
      case RX_SYNC1:
        rxState = (b == SYNC_1) ? RX_LEN : RX_SYNC0;
        break;
      case RX_LEN:
        rxLen = b;
        rxPos = 0;
        if (rxLen == 0 || rxLen > 60) { rxState = RX_SYNC0; break; }
        rxState = RX_PAYLOAD;
        break;
      case RX_PAYLOAD:
        rxBuf[rxPos++] = b;
        if (rxPos >= rxLen + 1) {  // payload + crc
          uint8_t crcRx = rxBuf[rxLen];
          uint8_t crcCalc = crc8(rxBuf, rxLen);
          if (crcRx == crcCalc) {
            processCommand(rxBuf, rxLen);
          }
          rxState = RX_SYNC0;
        }
        break;
    }
  }
}

// ═════════════════════════════════════════════════
//  LOAD CELL
// ═════════════════════════════════════════════════
bool loadCellInit() {
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  uint8_t err = I2C1Bus.endTransmission();
  if (err != 0) return false;

  // Discard initial stale read
  I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  while (I2C1Bus.available()) I2C1Bus.read();
  delay(5);
  return true;
}

void loadCellMeasurementRequest() {
  I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  while (I2C1Bus.available()) I2C1Bus.read();
}

bool loadCellFetch(uint16_t &bridgeRaw, uint8_t &status) {
  uint8_t count = I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)2);
  if (count < 2) return false;

  uint8_t msb = I2C1Bus.read();
  uint8_t lsb = I2C1Bus.read();

  status = (msb >> 6) & 0x03;
  bridgeRaw = ((uint16_t)(msb & 0x3F) << 8) | lsb;
  return true;
}

bool loadCellRead(uint16_t &bridgeRaw, uint8_t &status) {
  loadCellMeasurementRequest();
  delay(2);

  uint32_t start = millis();
  while (millis() - start < 50) {
    if (!loadCellFetch(bridgeRaw, status)) return false;
    if (status == LC_STATUS_VALID) return true;
    delayMicroseconds(500);
  }
  return false;
}

// ── Load Cell EEPROM Configuration ──────────────
// Offset_B values for A2D_Offset 0x0–0xF (EEPROM word 0x03)
static const uint16_t OFFSET_B_LUT[] = {
  0xE000, 0xE400, 0xE800, 0xEC00, 0xF000, 0xF400, 0xF800, 0xFC00,
  0x0000, 0x0400, 0x0800, 0x0C00, 0x1000, 0x1400, 0x1800, 0x1C00
};

static bool zscCommand(uint8_t cmd, uint16_t data) {
  I2C1Bus.beginTransmission(ZSC31014_ADDR);
  I2C1Bus.write(cmd);
  I2C1Bus.write((data >> 8) & 0xFF);
  I2C1Bus.write(data & 0xFF);
  return I2C1Bus.endTransmission() == 0;
}

static bool zscReadResponse(uint16_t &value) {
  if (I2C1Bus.requestFrom((uint8_t)ZSC31014_ADDR, (uint8_t)3) < 3) return false;
  uint8_t ack = I2C1Bus.read();
  uint8_t msb = I2C1Bus.read();
  uint8_t lsb = I2C1Bus.read();
  if (ack != 0x5A) return false;
  value = ((uint16_t)msb << 8) | lsb;
  return true;
}

// Try to enter command mode and configure gain/offset in EEPROM.
// Returns true if config was verified/updated, false if command mode unavailable.
// Command mode only available within 1.5ms of ZSC31014 power-on.
bool loadCellConfigureEEPROM() {
  // Enter command mode (Start_CM)
  if (!zscCommand(0xA0, 0x0000)) return false;
  delayMicroseconds(100);

  bool ok = false;
  uint16_t current = 0;

  // Read current B_Config (word 0x0F)
  if (!zscCommand(0x0F, 0x0000)) goto done;
  delayMicroseconds(100);
  if (!zscReadResponse(current)) goto done;

  {
    // Build desired B_Config, preserve bits [15:13]
    uint16_t desired = (current & 0xE000)
      | ((uint16_t)(LC_DISABLE_NULLING & 1) << 12)
      | (0b10 << 10)             // PreAmp_Mux = bridge
      | (1 << 9)                 // Bsink
      | (1 << 8)                 // LongInt
      | ((uint16_t)(LC_GAIN_POLARITY & 1) << 7)
      | ((uint16_t)(LC_PREAMP_GAIN & 0x07) << 4)
      | ((uint16_t)(LC_A2D_OFFSET & 0x0F));

    if (current != desired) {
      // Write B_Config (cmd = 0x40 + 0x0F)
      if (!zscCommand(0x4F, desired)) goto done;
      delay(15);
      // Write matching Offset_B (word 0x03, cmd = 0x40 + 0x03)
      if (!zscCommand(0x43, OFFSET_B_LUT[LC_A2D_OFFSET & 0x0F])) goto done;
      delay(15);
    }
  }
  ok = true;

done:
  // Exit command mode (Start_NOM — recalculates EEPROM checksum)
  zscCommand(0x80, 0x0000);
  delay(15);
  return ok;
}

// ═════════════════════════════════════════════════
//  SERVO MOTOR (Core0 only: scan + configure)
// ═════════════════════════════════════════════════
void servoInit() {
  pinMode(MB_EN_GPIO, OUTPUT);
  digitalWrite(MB_EN_GPIO, LOW);
  Serial485.begin(115200);
  Serial485.setTimeout(100); // 100 ms during scan; Core1 sets 20 ms for RUNNING
  servo.preTransmission(preTransmission);
  servo.postTransmission(postTransmission);
}

bool servoScanStep() {
  servo.begin(ctrl.scanID, Serial485);
  uint8_t result = servo.readHoldingRegisters(MB_REG_CONTROL_MODE, 1);
  if (result == servo.ku8MBSuccess) {
    ctrl.servoID = ctrl.scanID;
    return true;
  }
  ctrl.scanID++;
  if (ctrl.scanID > 247) ctrl.scanID = 1;
  return false;
}

static uint8_t servoWriteReg(uint16_t reg, uint16_t val, const char *name) {
  uint8_t r = servo.writeSingleRegister(reg, val);
  if (r != 0) Serial.printf("# CFG FAIL reg=%u (%s) val=%u code=0x%02X\n", reg, name, val, r);
  else        Serial.printf("# CFG OK   reg=%u (%s) val=%u\n", reg, name, val);
  return r;
}

bool servoConfigure(uint8_t id) {
  servo.begin(id, Serial485);
  Serial.printf("# servoConfigure ID=%u\n", id);

  // Stop servo first — config registers are locked while running
  uint8_t rOff = servo.writeSingleRegister(MB_REG_SERVO_ENABLE, 0);
  Serial.printf("# servo OFF: 0x%02X\n", rOff);
  delay(50);

  // Stiffness level (C00_05)
  servoWriteReg(MB_C00_05, ctrl.stiffness, "stiffness");

  // Acceleration / deceleration rates (C03_22 / C03_24)
  servoWriteReg(MB_C03_22, ctrl.accelRate, "accel");
  servoWriteReg(MB_C03_24, ctrl.decelRate, "decel");

  // Braking resistor configuration (C00_10–C00_13)
  servoWriteReg(MB_REG_BRAKE_RES_SEL,  BRAKE_RES_SEL_VAL,  "brk_sel");
  servoWriteReg(MB_REG_BRAKE_RES_POW,  BRAKE_RES_POW_VAL,  "brk_pow");
  servoWriteReg(MB_REG_BRAKE_RES_OHM,  BRAKE_RES_OHM_VAL,  "brk_ohm");
  servoWriteReg(MB_REG_BRAKE_RES_DISS, BRAKE_RES_DISS_VAL, "brk_diss");

  if (ctrl.modeTorque) {
    if (servoWriteReg(MB_REG_CONTROL_MODE,   2,                    "ctrl_mode") != 0) return false;
    if (servoWriteReg(MB_REG_TORQUE_REF,     0,                    "torq_ref")  != 0) return false;
    if (servoWriteReg(MB_C03_43, ctrl.torqueLimIntPos,             "tlim+")     != 0) return false;
    if (servoWriteReg(MB_C03_44, ctrl.torqueLimIntNeg,             "tlim-")     != 0) return false;
    if (servoWriteReg(MB_REG_TORQUE_LIM_POS, ctrl.speedLimTorqPos, "slimT+")   != 0) return false;
    if (servoWriteReg(MB_REG_TORQUE_LIM_NEG, ctrl.speedLimTorqNeg, "slimT-")   != 0) return false;
  } else {
    if (servoWriteReg(MB_REG_CONTROL_MODE, 1, "ctrl_mode") != 0) return false;
    if (servoWriteReg(MB_REG_SPEED_REF,    0, "spd_ref")   != 0) return false;
  }

  // Servo ON
  if (servoWriteReg(MB_REG_SERVO_ENABLE, 1, "servo_on") != 0) return false;
  Serial.println("# servoConfigure DONE");
  return true;
}

// ═════════════════════════════════════════════════
//  MAIN (Core0)
// ═════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  { unsigned long t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(10); }

  // Debug header (text, Python skips # lines)
  Serial.println("# ═══════════════════════════════════════");
  Serial.println("#  ZSC31014 + Servo — Raspberry Pi Pico 2");
  Serial.println("# ═══════════════════════════════════════");

  // Activate load cell power (EN on GP1)
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);
  delay(50);

  // Init I2C1 bus on GP2 (SDA) / GP3 (SCL)
  I2C1Bus.begin();
  I2C1Bus.setClock(LC_I2C_FREQ);

  // Try EEPROM config (needs power-on command mode window)
  ctrl.lcConfigApplied = loadCellConfigureEEPROM();
  Serial.printf("# LC Config: %s\n", ctrl.lcConfigApplied ? "OK" : "Power cycle needed");

  // Init load cell — required, error if missing
  ctrl.loadCellOK = loadCellInit();
  Serial.printf("# Load Cell: %s\n", ctrl.loadCellOK ? "OK" : "NOT FOUND");
  if (!ctrl.loadCellOK) {
    ctrl.errorCode = ERR_LC_NOT_FOUND;
    ctrl.currentState = STATE_ERROR;
    Serial.println("# ERROR: Load cell not found, system halted");
    Serial.flush();
    delay(50);
    return;
  }

  // Init Modbus hardware (Core0 owns Serial485 until STATE_RUNNING)
  servoInit();
  Serial.println("# Modbus: initialized");
  Serial.println("# Waiting for START command...");
  Serial.flush();
  delay(50);
}

void loop() {
  unsigned long now = millis();

  // Always check for commands from Python
  checkCommands();

  switch (ctrl.currentState) {

    case STATE_IDLE:
      if (now - ctrl.lastTelemetryTime >= 200) {
        uint16_t bridge; uint8_t lcSt;
        if (ctrl.loadCellOK && loadCellRead(bridge, lcSt))
          ctrl.lastBridge = bridge, ctrl.lastLCStatus = lcSt, ctrl.lastLCValid = (lcSt == LC_STATUS_VALID);
        telemetrySend(now, ctrl.lastBridge, ctrl.lastLCStatus, ctrl.lastLCValid,
                      0, 0, (uint8_t)ctrl.currentState, ctrl.modeTorque ? 1 : 0);
        ctrl.lastTelemetryTime = now;
      }
      break;

    case STATE_SCANNING:
      if (now - ctrl.scanStartTime > 10000) {  // 10 s — full scan at 100 ms/ID × 247 IDs ≈ 25 s worst case
        ctrl.errorCode = ERR_SERVO_SCAN_FAIL;
        ctrl.currentState = STATE_ERROR;
        Serial.println("# SCAN TIMEOUT — no servo found");
        break;
      }
      if (now - ctrl.lastActionTime >= 15) {
        if (servoScanStep()) {
          ctrl.currentState = STATE_CONFIGURING;
        }
        ctrl.lastActionTime = now;
      }
      if (now - ctrl.lastTelemetryTime >= 200) {
        uint16_t bridge; uint8_t lcSt;
        if (ctrl.loadCellOK && loadCellRead(bridge, lcSt))
          ctrl.lastBridge = bridge, ctrl.lastLCStatus = lcSt, ctrl.lastLCValid = (lcSt == LC_STATUS_VALID);
        telemetrySend(now, ctrl.lastBridge, ctrl.lastLCStatus, ctrl.lastLCValid,
                      0, 0, (uint8_t)ctrl.currentState, ctrl.modeTorque ? 1 : 0);
        ctrl.lastTelemetryTime = now;
      }
      break;

    case STATE_CONFIGURING:
      if (servoConfigure(ctrl.servoID)) {
        ctrl.configureRetries = 0;
        ctrl.errorCode = ERR_NONE;
        ctrl.servoConnected = true;

        // Pre-populate MA filter with real reading so servoRef starts at 0, not -73000
        {
          uint16_t initBridge = 0; uint8_t initSt = 0;
          if (ctrl.loadCellOK && loadCellRead(initBridge, initSt) && initSt == LC_STATUS_VALID) {
            for (uint8_t i = 0; i < ctrl.MA_SIZE; i++) ctrl.maBuf[i] = initBridge;
            ctrl.maSum    = (uint32_t)initBridge * ctrl.MA_SIZE;
            ctrl.maIdx    = 0;
            ctrl.lastBridgeFilt = initBridge;
            ctrl.lastBridge     = initBridge;
            ctrl.lastLCStatus   = initSt;
            ctrl.lastLCValid    = true;
          } else {
            ctrl.lastBridge = 0; ctrl.lastLCStatus = 0; ctrl.lastLCValid = false;
          }
        }

        // Hand Serial485 off to Core1 and enter RUNNING
        mbw.slaveID        = ctrl.servoID;
        mbw.modeTorque     = ctrl.modeTorque;
        mbw.refValue       = 0;
        mbw.failCount      = 0;
        mbw.connected      = true;
        mbw.motorRPM       = 0;
        mbw.motorTorqueX10 = 0;
        mbw.active         = true;   // Core1 takes over Serial485 from here

        ctrl.lcPhase = LC_REQUEST;
        ctrl.currentState = STATE_RUNNING;
      } else {
        ctrl.configureRetries++;
        if (ctrl.configureRetries >= 3) {
          ctrl.errorCode = ERR_SERVO_CONFIG_FAIL;
          ctrl.currentState = STATE_ERROR;
          Serial.printf("# CONFIG FAILED after 3 retries for ID=%u\n", ctrl.servoID);
        } else {
          // Retry the SAME ID (not +1) — servo was found here
          ctrl.scanID = ctrl.servoID;
          ctrl.servoID = 0;
          ctrl.scanStartTime = millis();
          ctrl.currentState = STATE_SCANNING;
          Serial.printf("# CONFIG retry %u/3 for ID=%u\n", ctrl.configureRetries, ctrl.scanID);
        }
      }
      break;

    case STATE_RUNNING:
      {
        // ── Load cell state machine (Core0 only, non-blocking) ──
        if (ctrl.loadCellOK) {
          if (ctrl.lcPhase == LC_REQUEST) {
            loadCellMeasurementRequest();
            ctrl.lcRequestTime = now;
            ctrl.lcPhase = LC_WAIT;
          } else if (now - ctrl.lcRequestTime >= 2) {
            uint16_t bridge;
            uint8_t status;
            if (loadCellFetch(bridge, status)) {
              if (status == LC_STATUS_VALID) {
                ctrl.lastBridge = bridge;
                ctrl.maSum -= ctrl.maBuf[ctrl.maIdx];
                ctrl.maBuf[ctrl.maIdx] = bridge;
                ctrl.maSum += bridge;
                ctrl.maIdx = (ctrl.maIdx + 1) % ctrl.MA_SIZE;
                ctrl.lastBridgeFilt = (uint16_t)(ctrl.maSum / ctrl.MA_SIZE);
                ctrl.lastLCStatus = status;
                ctrl.lastLCValid = true;
                ctrl.lcPhase = LC_REQUEST;
              } else if (now - ctrl.lcRequestTime > 50) {
                ctrl.errorCode = ERR_LC_STALE_TIMEOUT;
                goto error_stop;
              }
            } else {
              ctrl.errorCode = ERR_LC_I2C_FAIL;
              goto error_stop;
            }
          }
        }

        // ── Compute reference → Core1 writes it ──
        if (ctrl.lastLCValid) {
          ctrl.baseRead = (int16_t)ctrl.lastBridgeFilt - ctrl.zeroOffset;
          ctrl.servoRef = (ctrl.baseRead < ctrl.deadband)
                          ? ctrl.baseRead * ctrl.gain + ctrl.thsld : 0;
        } else {
          ctrl.servoRef = 0;
        }
        mbw.refValue   = ctrl.servoRef;
        mbw.modeTorque = ctrl.modeTorque;

        // ── Read monitoring results from Core1 ──
        ctrl.motorRPM       = mbw.motorRPM;
        ctrl.motorTorqueX10 = mbw.motorTorqueX10;
        ctrl.servoConnected = mbw.connected;

        // ── Detect servo disconnect → rescan ──
        if (!mbw.connected && mbw.failCount >= 5) {
          stopCore1();   // reclaim Serial485 before changing state
          ctrl.servoConnected = false;
          ctrl.motorRPM = 0;
          ctrl.motorTorqueX10 = 0;
          ctrl.scanID = ctrl.servoID;
          ctrl.servoID = 0;
          ctrl.scanStartTime = millis();
          ctrl.configureRetries = 0;
          Serial485.setTimeout(100);
          ctrl.currentState = STATE_SCANNING;
          break;
        }

        // ── Telemetry every 200 ms (Core0 is never blocked, so this fires on time) ──
        if (now - ctrl.lastTelemetryTime >= 200) {
          telemetrySend(now, ctrl.lastBridge, ctrl.lastLCStatus, ctrl.lastLCValid,
                        ctrl.motorRPM, ctrl.motorTorqueX10,
                        (uint8_t)ctrl.currentState, ctrl.modeTorque ? 1 : 0);
          ctrl.lastTelemetryTime = now;
        }
        break;

      error_stop:
        // Emergency stop: reclaim Serial485 from Core1, then send servo OFF
        stopCore1();
        servo.begin(ctrl.servoID, Serial485);
        servo.writeSingleRegister(MB_REG_SERVO_ENABLE, 0);
        ctrl.servoConnected = false;
        ctrl.motorRPM = 0;
        ctrl.motorTorqueX10 = 0;
        ctrl.servoRef = 0;
        ctrl.lastLCValid = false;
        telemetrySend(now, ctrl.lastBridge, ctrl.lastLCStatus, false,
                      0, 0, (uint8_t)STATE_ERROR, ctrl.modeTorque ? 1 : 0);
        ctrl.currentState = STATE_ERROR;
      }
      break;

    case STATE_ERROR:
      if (now - ctrl.lastTelemetryTime >= 1000) {
        telemetrySend(now, 0, 0, false, 0, 0, (uint8_t)ctrl.currentState, ctrl.modeTorque ? 1 : 0);
        ctrl.lastTelemetryTime = now;
      }
      break;
  }
}
