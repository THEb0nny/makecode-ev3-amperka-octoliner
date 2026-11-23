/**
 * Channel indices for Octoliner line sensor (0 is far left, 7 is far right)
 */
enum OctolinerChannel {
    //% block="0 (far left)"
    Channel0 = 0,
    //% block="1"
    Channel1 = 1,
    //% block="2"
    Channel2 = 2,
    //% block="3 (center left)"
    Channel3 = 3,
    //% block="4 (center right)"
    Channel4 = 4,
    //% block="5"
    Channel5 = 5,
    //% block="6"
    Channel6 = 6,
    //% block="7 (far right)"
    Channel7 = 7
}

/**
 * Line sensor events
 */
enum OctolinerEvent {
    //% block="line lost"
    LineLost = 1,
    //% block="intersection (all sensors black)"
    Intersection = 2
}

//% color="#008080" weight=65 icon="\uf06e" block="Octoliner"
//% groups='["Line", "Channels", "Configuration"]'
namespace sensors {

    const OCTOLINER_I2C_ADDR = 42; // 7-bit I2C address (0x2A)
    const PIN_MAP = [4, 5, 6, 8, 7, 3, 2, 1]; // ADC pin mapping for channels 0..7 on STM32 MCU

    // Amperka GpioExpander / Octoliner I2C commands (i2cioCommands.h)
    const CMD_PORT_MODE_OUTPUT = 0x07;
    const CMD_DIGITAL_WRITE_HIGH = 0x09;
    const CMD_ANALOG_WRITE = 0x0B;
    const CMD_ANALOG_READ = 0x0C;
    const CMD_PWM_FREQ = 0x0D;

    //% fixedInstances
    export class OctolinerSensor extends internal.I2cSensor {
        private _initialized: boolean;
        private _sensitivity: number;
        private _lastPosition: number;

        constructor(port: number) {
            super(port);
            this._initialized = false;
            this._sensitivity = 208;
            this._lastPosition = 0;
            // Activate I2C port connection mode in EV3 kernel driver
            this._setMode(0);
        }

        _deviceType() {
            return DAL.DEVICE_TYPE_NXT_IIC;
        }

        _activated() {
            this._setMode(this.mode);
        }

        /**
         * Match the 16 null bytes reported by readI2cId() on EV3 for generic I2C devices
         */
        _i2cId(): string {
            let id = "";
            for (let i = 0; i < 16; i++) {
                id += String.fromCharCode(0);
            }
            return id;
        }

        /**
         * Return active state so getBytes() always reads from this port
         */
        isActive(): boolean {
            return true;
        }

        /**
         * Initializes the sensor (enables IR illumination and configures PWM frequency)
         */
        private ensureInit() {
            if (this._initialized) return;

            // Ensure EV3 kernel I2C port mode is active
            this._setMode(0);
            pause(30);

            // 1. Set PWM frequency (8000 Hz: 0x1F40)
            this.transaction(OCTOLINER_I2C_ADDR, [CMD_PWM_FREQ, 0x1F, 0x40], 0);
            pause(20);

            // 2. Turn on IR LED illumination pin (Pin 9)
            // Compatible with Octoliner v2 (OUTPUT HIGH) and v1 (PWM brightness):
            this.transaction(OCTOLINER_I2C_ADDR, [CMD_PORT_MODE_OUTPUT, 0x02, 0x00], 0);
            pause(20);
            this.transaction(OCTOLINER_I2C_ADDR, [CMD_DIGITAL_WRITE_HIGH, 0x02, 0x00], 0);
            pause(20);
            // In v1, IR LEDs are driven via PWM on pin 9 (65535 = full brightness)
            this.transaction(OCTOLINER_I2C_ADDR, [CMD_ANALOG_WRITE, 9, 0xFF, 0xFF], 0);
            pause(20);

            // 3. Set sensitivity via PWM on Pin 0 (0..65535)
            this.setSensitivity(this._sensitivity);
            pause(20);

            this._initialized = true;
        }

        /**
         * Sets sensor sensitivity (0 to 255). Default is 208.
         * Higher values increase sensor response to reflected IR light.
         * @param value sensitivity level from 0 to 255
         */
        //% blockId=octoliner_set_sensitivity
        //% block="set %this sensitivity to %value"
        //% block.loc.ru="установить чувствительность %this на %value"
        //% value.min=0 value.max=255 value.defl=208
        //% group="Configuration" weight=20
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public setSensitivity(value: number) {
            this._sensitivity = Math.max(0, Math.min(255, value));
            // Map 0..255 to 16-bit 0..65535: (val << 8) | val
            const val16 = (this._sensitivity << 8) | this._sensitivity;
            // ANALOG_WRITE command (0x0B): [command, pin(0), msb, lsb]
            this.transaction(OCTOLINER_I2C_ADDR, [CMD_ANALOG_WRITE, 0, (val16 >> 8) & 0xFF, val16 & 0xFF], 0);
            pause(20);
        }

        /**
         * Sets IR LED illumination brightness (0 to 255). Default is 255.
         * Particularly useful for Octoliner v1 which supports PWM brightness control.
         * @param value brightness level from 0 to 255
         */
        //% blockId=octoliner_set_brightness
        //% block="set %this brightness to %value"
        //% block.loc.ru="установить яркость %this на %value"
        //% value.min=0 value.max=255 value.defl=255
        //% group="Configuration" weight=15
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public setBrightness(value: number) {
            const b = Math.max(0, Math.min(255, value));
            const val16 = (b << 8) | b;
            this.transaction(OCTOLINER_I2C_ADDR, [CMD_ANALOG_WRITE, 9, (val16 >> 8) & 0xFF, val16 & 0xFF], 0);
            pause(20);
        }

        /**
         * Reads the raw analog value from a single channel (0 to 4095).
         * Darker surfaces reflect less IR light and produce higher values.
         * @param channel channel index from 0 to 7
         */
        //% blockId=octoliner_read_channel
        //% block="%this channel %channel value"
        //% block.loc.ru="значение канала %channel на %this"
        //% group="Channels" weight=50
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public readChannel(channel: OctolinerChannel): number {
            this.ensureInit();
            const pin = PIN_MAP[channel & 7];

            // Request ADC channel reading
            this.transaction(OCTOLINER_I2C_ADDR, [CMD_ANALOG_READ, pin], 2);
            pause(20); // Wait for transaction to complete in EV3 kernel timer interrupt

            const buf = this.getBytes();
            if (!buf || buf.length < 2) return -1;

            // If the I2C transfer NACKed, EV3 driver echoes back the TX buffer ([CMD_ANALOG_READ, pin])
            if (buf[0] === CMD_ANALOG_READ && buf[1] === pin) {
                return -1;
            }

            // 16-bit ADC reading (MSB first, LSB second):
            return (buf[0] << 8) | buf[1];
        }

        /**
         * Reads all 8 analog channels into an array.
         */
        public readAllChannels(): number[] {
            this.ensureInit();
            const result: number[] = [];
            for (let i = 0; i < 8; i++) {
                result.push(this.readChannel(i));
            }
            return result;
        }

        /**
         * Returns an 8-bit binary pattern representing the line state (1 = black/line, 0 = white/background).
         * Bit 7 corresponds to channel 0 (far left), bit 0 corresponds to channel 7 (far right).
         */
        //% blockId=octoliner_read_bitmask
        //% block="%this binary pattern"
        //% block.loc.ru="битовая маска датчиков %this"
        //% group="Line" weight=60
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public readBitmask(): number {
            const vals = this.readAllChannels();
            let minVal = 4095;
            let maxVal = 0;

            for (let i = 0; i < 8; i++) {
                if (vals[i] < minVal) minVal = vals[i];
                if (vals[i] > maxVal) maxVal = vals[i];
            }

            // Dynamic threshold midway between minimum and maximum readings
            const threshold = minVal + ((maxVal - minVal) >> 1);
            let mask = 0;
            for (let i = 0; i < 8; i++) {
                mask = (mask << 1) | (vals[i] > threshold ? 1 : 0);
            }
            return mask;
        }

        /**
         * Returns line position relative to sensor center, from -100 to +100.
         * 0 means line is directly centered.
         * Negative values indicate line is to the left, positive to the right.
         * If line is completely lost, returns the last known side (-100 or +100).
         */
        //% blockId=octoliner_line_position
        //% block="%this line position"
        //% block.loc.ru="положение линии на %this"
        //% group="Line" weight=100
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public linePosition(): number {
            const mask = this.readBitmask();

            if (mask === 0) {
                // Line lost: maintain the last detected side
                return this._lastPosition >= 0 ? 100 : -100;
            }

            let weightedSum = 0;
            let count = 0;

            // Channel weights centered around midpoint: from -3.5 (channel 0) to +3.5 (channel 7)
            for (let i = 0; i < 8; i++) {
                if ((mask & (1 << (7 - i))) !== 0) {
                    weightedSum += (i - 3.5);
                    count++;
                }
            }

            // Scale to -100 .. +100
            const pos = Math.round((weightedSum / (count * 3.5)) * 100);
            this._lastPosition = pos;
            return pos;
        }

        /**
         * Checks whether the sensor detects a line.
         */
        //% blockId=octoliner_is_on_line
        //% block="%this sees line"
        //% block.loc.ru="%this видит линию"
        //% group="Line" weight=80
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public isOnLine(): boolean {
            return this.readBitmask() !== 0;
        }

        /**
         * Checks whether the sensor detects an intersection (all 8 channels detect line).
         */
        //% blockId=octoliner_is_intersection
        //% block="%this sees intersection"
        //% block.loc.ru="%this видит перекресток"
        //% group="Line" weight=70
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public isIntersection(): boolean {
            return this.readBitmask() === 0xFF;
        }

        /**
         * Port number for this sensor instance (1, 2, 3, or 4).
         */
        public port(): number {
            return this._port + 1;
        }
    }

    //% whenUsed block="1" weight=95 fixedInstance jres=icons.port1
    export const octoliner1: OctolinerSensor = new OctolinerSensor(1);

    //% whenUsed block="2" weight=90 fixedInstance jres=icons.port2
    export const octoliner2: OctolinerSensor = new OctolinerSensor(2);

    //% whenUsed block="3" weight=90 fixedInstance jres=icons.port3
    export const octoliner3: OctolinerSensor = new OctolinerSensor(3);

    //% whenUsed block="4" weight=90 fixedInstance jres=icons.port4
    export const octoliner4: OctolinerSensor = new OctolinerSensor(4);
}
