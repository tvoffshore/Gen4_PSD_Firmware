#
# Serial commands list v0.1

NOTES:

- ADR - slave address of the device, must contain 3 digits (for example 001, 033, 144). The **default address is 123** , the **broadcast address is 000.** The device may only handle SET or EXECUTE broadcast commands. If the board receives a GET broadcast command it won't respond with one exception: request of the slave address **ID 0: SlaveAddress**.
- \<CR\> - Carriage Return symbol '\r'
- Default values are applied for new sensors after the first firmware flashing. The next firmware updates DON'T reset the settings to default values.

# **ID 0: SlaveAddress** - Serial slave address

Set/Get the slave address for serial interfaces. This is the only serial command where the device responds to the broadcast.

**NOTE:** Please make sure there is only one device on the bus.

Default value: 123

Usage GET: !ADR:ADDR?\<CR\> or !000:ADDR?\<CR\>

Usage SET: !ADR:ADDR=123\<CR\> or !000:ADDR=123\<CR\>

# **ID 1: Date** - RTC Date

Set/Get the date in the format "yyyymmdd" (20220513 - 13 May 2022)

Usage GET: !ADR:DATE?\<CR\>

Usage SET: !ADR:DATE=20220513\<CR\>

# **ID 2: Time** - RTC Time

Set/Get the time in the format "hhmmss" (203230 - 20 hours 32 minutes and 30 seconds)

Usage GET: !ADR:TIME?\<CR\>

Usage SET: !ADR:TIME=203230\<CR\>

# **ID 3: PsdPoints** - PSD points count

Set/Get the PSD points count, which corresponds to the number of calculated bins. The minimum value is **1** , the maximum value is **10**.

**NOTE:** PSD segment size is equal to power of two of PSD points count.

Default value: 8

Usage GET: !ADR:PSDP?\<CR\>

Usage SET: !ADR:PSDP=8\<CR\>

# **ID 4: PsdCutoff** - PSD cutoff count

Set/Get PSD cutoff to store the results. It corresponds to the number of stored bins; the rest of the bins will be omitted. The minimum value is **1** , the maximum value is **1024**.

**NOTE:** PSD segment size is equal to power of two of PSD points count.

Default value: 128

Usage GET: !ADR:PSDC?\<CR\>

Usage SET: !ADR:PSDC=128\<CR\>

# **ID 5: DataTypeControl** - Control data type capabilities

Set/Get bitmask each bit of which corresponds to data types capabilities: 0 - disable data type, 1 - enable data type. Enabled data types will be taken/calculated and stored on SD.

**NOTE:** The current measurements will be restarted when data type capabilities are changed.

The bitmask value is according to the table below:

| | Bit 2 | Bit 1 | Bit 0 | Value |
| --- | --- | --- | --- | --- |
| Psd | | | X | 1 |
| Statistic | | X | | 2 |
| Statistic + Psd | | X | X | 3 |
| Raw | X | | | 4 |
| Raw + Psd | X | | X | 5 |
| Raw + Statistic | X | X | | 6 |
| Raw + Statistic + Psd | X | X | X | 7 |

Default value: 7 (Raw + Statistic + Psd)

Usage GET: !ADR:DTYP?\<CR\>

Usage SET: !ADR:DTYP=7\<CR\>

# **ID 6: MeasureControl** - Control measurement capabilities

Set/Get the bitmask each bit of which corresponds to sensors measurement capabilities: 0 - disable sensor, 1 - enable sensor. Enabled sensors will be used for measurements.

**NOTE:** The current measurements will be restarted when measurement capabilities are changed.

The bitmask value is according to the table below:

| | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 | Value |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ACC | | | | | | X | 1 |
| GYR | | | | | X | | 2 |
| GYR + ACC | | | | | X | X | 3 |
| ANG | | | | X | | | 4 |
| ANG+ACC | | | | X | | X | 5 |
| ANG + GYR | | | | X | X | | 6 |
| ANG + GYR + ACC | | | | X | X | X | 7 |
| AD1 | | | X | | | | 8 |
| AD1 + ACC | | | X | | | X | 9 |
| AD1 + GYR | | | X | | X | | 10 |
| AD1 + GYR + ACC | | | X | | X | X | 11 |
| AD1 + ANG | | | X | X | | | 12 |
| AD1 + ANG + ACC | | | X | X | | X | 13 |
| AD1 + ANG + GYR | | | X | X | X | | 14 |
| AD1 + ANG + GYR + ACC | | | X | X | X | X | 15 |
| AD2 | | X | | | | | 16 |
| AD2 + ACC | | X | | | | X | 17 |
| AD2 + GYR | | X | | | X | | 18 |
| AD2 + GYR + ACC | | X | | | X | X | 19 |
| AD2 + ANG | | X | | X | | | 20 |
| AD2 + ANG + ACC | | X | | X | | X | 21 |
| AD2 + ANG + GYR | | X | | X | X | | 22 |
| AD2 + ANG + GYR + ACC | | X | | X | X | X | 23 |
| AD2 + AD1 | | X | X | | | | 24 |
| AD2 + AD1 + ACC | | X | X | | | X | 25 |
| AD2 + AD1 + GYR | | X | X | | X | | 26 |
| AD2 + AD1 + GYR + ACC | | X | X | | X | X | 27 |
| AD2 + AD1 + ANG | | X | X | X | | | 28 |
| AD2 + AD1 + ANG + ACC | | X | X | X | | X | 29 |
| AD2 + AD1 + ANG + GYR | | X | X | X | X | | 30 |
| AD2 + AD1 + ANG + GYR + ACC | | X | X | X | X | X | 31 |
| ACC_RES | X | | | | | | 32 |
| ACC_RES + ACC | X | | | | | X | 33 |
| ACC_RES + GYR | X | | | | X | | 34 |
| ACC_RES + GYR + ACC | X | | | | X | X | 35 |
| ACC_RES + ANG | X | | | X | | | 36 |
| ACC_RES + ANG + ACC | X | | | X | | X | 37 |
| ACC_RES + ANG + GYR | X | | | X | X | | 38 |
| ACC_RES + ANG + GYR + ACC | X | | | X | X | X | 39 |
| ACC_RES + AD1 | X | | X | | | | 40 |
| ACC_RES + AD1 + ACC | X | | X | | | X | 41 |
| ACC_RES + AD1 + GYR | X | | X | | X | | 42 |
| ACC_RES + AD1 + GYR + ACC | X | | X | | X | X | 43 |
| ACC_RES + AD1 + ANG | X | | X | X | | | 44 |
| ACC_RES + AD1 + ANG + ACC | X | | X | X | | X | 45 |
| ACC_RES + AD1 + ANG + GYR | X | | X | X | X | | 46 |
| ACC_RES + AD1 + ANG + GYR + ACC | X | | X | X | X | X | 47 |
| ACC_RES + AD2 | X | X | | | | | 48 |
| ACC_RES + AD2 + ACC | X | X | | | | X | 49 |
| ACC_RES + AD2 + GYR | X | X | | | X | | 50 |
| ACC_RES + AD2 + GYR + ACC | X | X | | | X | X | 51 |
| ACC_RES + AD2 + ANG | X | X | | X | | | 52 |
| ACC_RES + AD2 + ANG + ACC | X | X | | X | | X | 53 |
| ACC_RES + AD2 + ANG + GYR | X | X | | X | X | | 54 |
| ACC_RES + AD2 + ANG + GYR + ACC | X | X | | X | X | X | 55 |
| ACC_RES + AD2 + AD1 | X | X | X | | | | 56 |
| ACC_RES + AD2 + AD1 + ACC | X | X | X | | | X | 57 |
| ACC_RES + AD2 + AD1 + GYR | X | X | X | | X | | 58 |
| ACC_RES + AD2 + AD1 + GYR + ACC | X | X | X | | X | X | 59 |
| ACC_RES + AD2 + AD1 + ANG | X | X | X | X | | | 60 |
| ACC_RES + AD2 + AD1 + ANG + ACC | X | X | X | X | | X | 61 |
| ACC_RES + AD2 + AD1 + ANG + GYR | X | X | X | X | X | | 62 |
| ACC_RES + AD2 + AD1 + ANG + GYR + ACC | X | X | X | X | X | X | 63 |

Default value: 63 (ACC_RES + AD2 + AD1 + ANG + GYR + ACC)

Usage GET: !ADR:MCTR?\<CR\>

Usage SET: !ADR:MCTR=63\<CR\>

# **ID 7: MeasureFrequency** - Measurements frequency

Set/Get the frequency of measurements - frequency with which the data will be sampled and read during measurements, Hz. The minimum value is **1Hz** , the maximum value is **100Hz**.

Default value: 10Hz

Usage GET: !ADR:MFRQ?\<CR\>

Usage SET: !ADR:MFRQ=10\<CR\>

# **ID 8: MeasureInterval** - Measurements interval

Set/Get the interval of measurements - the amount of time the measurements are taken before the pause interval (see **ID 9: PauseInterval**), Seconds. The minimum value is **1 sec**.

Default value: 600 seconds

Usage GET: !ADR:MINT?\<CR\>

Usage SET: !ADR:MINT=600\<CR\>

# **ID 9: PauseInterval** - Pause interval

Set/Get the interval of pause between measurements - the amount of time between the measurements intervals (see **ID 8: MeasureInterval**), Seconds. Set **interval to 0** makes measurements **infinite** (without pauses between measurements).

Default value: 300 seconds

Usage GET: !ADR:PINT?\<CR\>

Usage SET: !ADR:PINT=300\<CR\>

# **ID 10: AccelRange** - Accelerometer range

Set/Get accelerometer range (G).

Suitable values are given in the table below:

| Range | Value |
| --- | --- |
| 2G | 0 |
| 4G | 1 |
| 8G | 2 |
| 16G | 3 |

Default value: 0 (2G)

Usage GET: !ADR:ACRG?\<CR\>

Usage SET: !ADR:ACRG=0\<CR\>

# **ID 11: GyroRange** - Gyroscope range

Set/Get gyroscope range (DPS).

Suitable values are given in the table below:

| Range | Value |
| --- | --- |
| 125DPS | 0 |
| 250DPS | 1 |
| 500DPS | 2 |
| 1000DPS | 3 |
| 2000DPS | 4 |

Default value: 2 (+/- 500deg/sec)

Usage GET: !ADR:GYRG?\<CR\>

Usage SET: !ADR:GYRG=2\<CR\>

# **ID 12: AdcVoltage** - ADC sensors voltage

Set/Get voltage applied to both ADC sensors. The new voltage level will not be applied after the set command if ADC sensors disabled or sampling is disabled or paused. It will be applied automatically when ADC1 or ADC2 sampling starts OR you can apply it manually with **ID 34: ApplyAdcVoltage** command.

Default value: 10V

Range: 9.2-24V

Usage GET: !ADR:VSNS?\<CR\>

Usage SET: !ADR:VSNS=10\<CR\>

# **ID 13: GainSelect** - Gain selection configuration for ADC1

Set/Get the gain applied to the input of ADC1 sensor.

Suitable values are given in the table below:

| Gain | Value |
| --- | --- |
| 100 | 0 |
| 150 | 1 |
| 200 | 2 |
| 250 | 3 |

Default value: 0

Usage GET: !ADR:GNSL?\<CR\>

Usage SET: !ADR:GNSL=0\<CR\>

# **ID 14: InputTypeSelect** - Input type selection configuration for ADC2

Set/Get the input type of the ADC2 sensor.

Suitable values are given in the table below:

| Type | Value |
| --- | --- |
| IN2-/4-20mA | 0 |
| IN2+/0-5V | 1 |
| IN2 output | 2 |
| Ground | 3 |

Default value: 2

Usage GET: !ADR:ITSL?\<CR\>

Usage SET: !ADR:ITSL=2\<CR\>

# **ID 15: SerialSelect** - Serial interface selection

Set/Get the serial interface selected for serial commands handling.

**NOTE:** The serial interface will be selected immediately and the previous interface will become unavailable. USB interface is always available to control the device the same way as RS232/RS485 but with log messages sprinkled in command responses. To remove unnecessary log messages from the text the desired log level may be set with **ID 10: LogLevel** (0 to turn off log messages at all).

Suitable values are given in the table below:

| Serial | Value |
| --- | --- |
| RS232 | 0 |
| RS485 | 1 |

Default value: 0 (RS232)

Usage GET: !ADR:SERS?\<CR\>

Usage SET: !ADR:SERS=0\<CR\>

# **ID 16: LogLevel** - Log messages level configuration

Set/Get the maximum desired log messages level. The messages with level below this setting (higher value) will be hidden.

Suitable values are given in the table below:

| Log level | Value |
| --- | --- |
| None | 0 |
| Error | 1 |
| Warning | 2 |
| Info | 3 |
| Debug | 4 |
| Trace | 5 |

Default value: 3 (Info) for Release versions, 4 (Debug) for Debug versions

Usage GET: !ADR:LOGL?\<CR\>

Usage SET: !ADR:LOGL=Value\<CR\>

# **ID 17: BatteryStatus** - Battery status

Get the battery status (voltage in millivolts, level in percents).

Usage GET: !ADR:BATT?\<CR\>

# **ID 18: FwVersion** - Firmware version information

Get the firmware version information. The information consists of major and minor versions.

Usage GET: !ADR:FVER?\<CR\>
