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
        private _address: number;
        private _wireAddress: number;
        private _failCount: number;
        private _lastWakeTime: number;
        private _initAttemptTime: number;

        constructor(port: number) {
            super(port);
            this._initialized = false;
            this._sensitivity = 208;
            this._lastPosition = 0;
            this._address = OCTOLINER_I2C_ADDR;
            this._wireAddress = OCTOLINER_I2C_ADDR; // 42 (7-bit I2C address)
            this._failCount = 0;
            this._lastWakeTime = 0;
            this._initAttemptTime = 0;
            // Force hardware I2C mode activation:
            // Base I2cSensor constructor initializes realMode=0 and mode=0, so _setMode(0) is a no-op unless realMode is -1!
            this.realMode = -1;
            this._setMode(0);
        }

        _deviceType() {
            return DAL.DEVICE_TYPE_IIC_UNKNOWN;
        }

        _activated() {
            this.realMode = -1;
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
         * Wakes up the EV3 kernel I2C driver (d_iic.c).
         * Waits for kernel sensor autodetection to complete, then toggles port mode
         * to transition the driver out of IIC_SETUP_WAIT into IIC_WAITING.
         * Throttled to at most once every 3 seconds to avoid resetting the driver in rapid loops.
         */
        private wakeDriver() {
            const uptime = control.millis();
            if (uptime < 1200) {
                pause(1200 - uptime);
            }

            const now = control.millis();
            if (this._lastWakeTime !== 0 && (now - this._lastWakeTime) < 3000) {
                return;
            }
            this._lastWakeTime = now;

            // In d_iic.c, Device1Ioctl only transitions out of IIC_SETUP_WAIT when
            // IicPort[Port].Initialised == 1 AND Mode != (*pDevCon).Mode[Port].
            // Because Mode defaults to 0, setting mode 0 is a no-op unless we toggle to 1 and back.
            this.realMode = -1;
            this._setMode(1);
            pause(50);
            this.realMode = -1;
            this._setMode(0);
            pause(100);
        }

        /**
         * Reliably sends an I2C transaction to d_iic.c.
         * The EV3 d_iic kernel driver sets IIC_DATA_READY upon transfer completion,
         * but keeps IIC_WRITE_REQUEST set until an ioctl(IIC_SETUP) call clears it.
         * If a new transaction is sent while DATA_READY is pending, the first ioctl
         * call only clears the status and DROPS the new transaction!
         * Calling it twice guarantees that:
         * 1) any pending status is cleared (call 1),
         * 2) the new transaction is loaded and started (call 2).
         */
        private sendTransaction(addr: number, writeBuf: number[], readLen: number) {
            this.transaction(addr, writeBuf, readLen);
            this.transaction(addr, writeBuf, readLen);
        }

        /**
         * Checks if the received buffer represents a NACK or invalid response.
         */
        private isNack(buf: Buffer, pin: number): boolean {
            if (!buf || buf.length < 2) return true;
            // Disconnected or idle driver (all 0x00):
            if (buf[0] === 0 && buf[1] === 0) return true;
            // Echoed TX buffer (forward [CMD, pin] or reversed by getI2cBytes [pin, CMD]):
            if (buf[0] === CMD_ANALOG_READ && buf[1] === pin) return true;
            if (buf[1] === CMD_ANALOG_READ && buf[0] === pin) return true;
            // Floating bus or timeout (all 0xFF):
            if (buf[0] === 0xFF && buf[1] === 0xFF) return true;
            return false;
        }

        /**
         * Probes a specific wire address with a test read. Returns true if device ACKs.
         */
        private probeWireAddress(addr: number): boolean {
            this.sendTransaction(addr, [CMD_ANALOG_READ, 4], 2);
            pause(30);
            const buf = this.getBytes();
            return !this.isNack(buf, 4);
        }

        /**
         * Initializes sensor hardware: configures PWM frequency, enables IR illumination,
         * sets sensitivity, and verifies ACK communication.
         */
        private initHardware() {
            // 1. Set PWM frequency (8000 Hz: 0x1F40)
            this.sendTransaction(this._wireAddress, [CMD_PWM_FREQ, 0x1F, 0x40], 0);
            pause(25);

            // 2. Turn on IR LED illumination pin (Pin 9)
            // Compatible with Octoliner v2 (OUTPUT HIGH) and v1 (PWM brightness):
            this.sendTransaction(this._wireAddress, [CMD_PORT_MODE_OUTPUT, 0x02, 0x00], 0);
            pause(25);
            this.sendTransaction(this._wireAddress, [CMD_DIGITAL_WRITE_HIGH, 0x02, 0x00], 0);
            pause(25);
            // In v1, IR LEDs are driven via PWM on pin 9 (65535 = full brightness)
            this.sendTransaction(this._wireAddress, [CMD_ANALOG_WRITE, 9, 0xFF, 0xFF], 0);
            pause(25);

            // 3. Set sensitivity via PWM on Pin 0 (0..65535)
            const val16 = (this._sensitivity << 8) | this._sensitivity;
            this.sendTransaction(this._wireAddress, [CMD_ANALOG_WRITE, 0, (val16 >> 8) & 0xFF, val16 & 0xFF], 0);
            pause(25);

            // 4. Verify communication with a test read
            if (this.probeWireAddress(this._wireAddress)) {
                this._initialized = true;
                control.dmesg(`Octoliner init success on port ${this.port()} at wire addr ${this._wireAddress}`);
            } else {
                control.dmesg(`Octoliner init ACK failed on port ${this.port()}`);
            }
        }

        /**
         * Initializes the sensor if not already initialized.
         */
        private ensureInit() {
            if (this._initialized) return;

            const now = control.millis();
            if (this._initAttemptTime !== 0 && (now - this._initAttemptTime) < 1500) {
                return;
            }
            this._initAttemptTime = now;

            this.wakeDriver();
            control.dmesg(`Octoliner ensureInit on port ${this.port()}`);

            // Try candidate wire addresses in order of likelihood:
            // 42 (raw 7-bit), 66 (raw 0x42), 84 (42 shifted), 132 (0x42 shifted)
            const candidates = [42, 66, 84, 132];
            let found = false;
            for (let i = 0; i < candidates.length; i++) {
                if (this.probeWireAddress(candidates[i])) {
                    this._wireAddress = candidates[i];
                    found = true;
                    break;
                }
            }

            if (found) {
                this.initHardware();
            }
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
            this.ensureInit();
            this._sensitivity = Math.max(0, Math.min(255, value));
            // Map 0..255 to 16-bit 0..65535: (val << 8) | val
            const val16 = (this._sensitivity << 8) | this._sensitivity;
            // ANALOG_WRITE command (0x0B): [command, pin(0), msb, lsb]
            this.sendTransaction(this._wireAddress, [CMD_ANALOG_WRITE, 0, (val16 >> 8) & 0xFF, val16 & 0xFF], 0);
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
            this.ensureInit();
            const b = Math.max(0, Math.min(255, value));
            const val16 = (b << 8) | b;
            this.sendTransaction(this._wireAddress, [CMD_ANALOG_WRITE, 9, (val16 >> 8) & 0xFF, val16 & 0xFF], 0);
            pause(20);
        }

        /**
         * Sets custom 7-bit I2C address for the sensor (default is 42).
         * @param address 7-bit I2C address (1 to 127)
         */
        //% blockId=octoliner_set_address
        //% block="set %this I2C address to %address"
        //% block.loc.ru="установить I2C адрес %this на %address"
        //% address.min=1 address.max=127 address.defl=42
        //% group="Configuration" weight=10
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public setAddress(address: number) {
            this._address = address & 0x7F;
            this._wireAddress = this._address;
            this._initialized = false;
        }

        /**
         * Scans I2C addresses 1..127 on this port and returns the first responding address, or -1 if none found.
         */
        //% blockId=octoliner_scan_address
        //% block="%this scan I2C address"
        //% block.loc.ru="сканировать I2C адрес на %this"
        //% group="Configuration" weight=5
        //% this.fieldEditor="images" this.fieldOptions.columns="4" this.fieldOptions.width="300"
        public scanAddress(): number {
            this.wakeDriver();
            control.dmesg(`Octoliner scanning I2C addresses on port ${this.port()}`);

            // Test most likely addresses first:
            // 42 (raw 7-bit default), 66 (raw 0x42), 84 (42 shifted), 132 (0x42 shifted)
            const quickAddrs = [42, 66, 84, 132];
            for (let i = 0; i < quickAddrs.length; i++) {
                const wire = quickAddrs[i];
                if (this.probeWireAddress(wire)) {
                    this._wireAddress = wire;
                    this._address = wire > 127 ? (wire >> 1) : wire;
                    this.initHardware();
                    return this._address;
                }
            }

            // Scan all 7-bit addresses (1..127)
            for (let addr = 1; addr < 128; addr++) {
                // Try raw 7-bit unshifted address first
                if (this.probeWireAddress(addr)) {
                    control.dmesg(`Octoliner found responding raw address: ${addr}`);
                    this._address = addr;
                    this._wireAddress = addr;
                    this.initHardware();
                    return addr;
                }
                // Try 8-bit shifted address
                if (this.probeWireAddress(addr << 1)) {
                    control.dmesg(`Octoliner found responding 7-bit address: ${addr} (wire ${addr << 1})`);
                    this._address = addr;
                    this._wireAddress = addr << 1;
                    this.initHardware();
                    return addr;
                }
            }
            return -1;
        }

        /**
         * Returns diagnostic string showing wire address, init status, and last received raw buffer.
         */
        public debugInfo(): string {
            const buf = this.getBytes();
            const b0 = buf && buf.length > 0 ? buf[0] : -1;
            const b1 = buf && buf.length > 1 ? buf[1] : -1;
            const status = this._initialized ? "OK" : "NO";
            return `W:${this._wireAddress} ${status} [${b0},${b1}]`;
        }

        /**
         * Raw I2C probe: sends a single-byte command to the given wire address and
         * returns the two received bytes as a string "b0,b1" WITHOUT any NACK filtering.
         * Use for diagnosis to see exactly what the driver returns for each address.
         * @param wireAddr wire address (0–255) to probe
         * @param cmd command byte to send (default 0x00)
         */
        public rawProbe(wireAddr: number, cmd: number = 0): string {
            this.transaction(wireAddr, [cmd], 2);
            pause(80);
            this.transaction(wireAddr, [cmd], 2);
            pause(80);
            const buf = this.getBytes();
            const b0 = buf && buf.length > 0 ? buf[0] : -1;
            const b1 = buf && buf.length > 1 ? buf[1] : -1;
            return `${b0},${b1}`;
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
            this.sendTransaction(this._wireAddress, [CMD_ANALOG_READ, pin], 2);
            pause(25); // Wait for transaction to complete in EV3 kernel timer interrupt

            const buf = this.getBytes();
            if (this.isNack(buf, pin)) {
                this._failCount++;
                if (this._failCount > 10) {
                    this._initialized = false;
                    this._failCount = 0;
                }
                return -1;
            }

            this._failCount = 0;
            // getI2cBytes reverses the 2 bytes: buf[0] is LSB, buf[1] is MSB.
            // On STM32 ADC is 12-bit (0..4095).
            let val = (buf[1] << 8) | buf[0];
            if (val > 4095) {
                val = (buf[0] << 8) | buf[1];
            }
            return Math.min(4095, Math.max(0, val));
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
