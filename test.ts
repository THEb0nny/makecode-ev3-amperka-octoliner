// === OCTOLINER PORT 4 DIAGNOSTIC TEST ===
// Probes addresses on Port 4 and shows raw bytes on screen.

const sensor = sensors.octoliner4; // TARGET: Port 4!

// Wait for EV3 boot autodetect to settle
pause(2000);

brick.showString("OCTOLINER PORT 4", 1);
brick.showString("Step 1: Test LED 84", 2);
pause(1000);

// Try turning on LED at wire address 84 (42 shifted)
sensor.testLed(84);
pause(500);

// Priority addresses to test with raw probe:
// 84 = 42 << 1 (standard 7-bit 42 as 8-bit wire address)
// 42 = raw 7-bit 42 (unshifted)
// 132 = 66 << 1 (0x42 shifted)
// 66 = raw 0x42
// 85 = 84 | 1 (read address)
// 2 = NXT Ultrasonic standard wire address
const priority = [84, 42, 132, 66, 85, 2, 4, 8, 16, 32, 64, 128];

brick.showString("Step 2: Priority Addrs", 2);
pause(1000);

let found = -1;
let line = 3;

for (let i = 0; i < priority.length; i++) {
    const addr = priority[i];
    // Send [12, 4] = CMD_ANALOG_READ pin 4
    // If NACK: EV3 driver returns [12, 4] (echo) or [0, 0] if port idle
    // If ACK: EV3 driver returns actual ADC reading (NOT [12, 4])
    const res = sensor.rawProbe(addr, 12, 4);
    brick.showString(`A:${addr} [${res}]`, line);
    line++;
    if (line > 10) {
        pause(2000);
        line = 3;
    }

    const parts = res.split(",");
    const b0 = parseInt(parts[0]);
    const b1 = parseInt(parts[1]);
    const isEcho = (b0 === 12 && b1 === 4) || (b0 === 4 && b1 === 12);
    const isZero = (b0 === 0 && b1 === 0);
    const isFF = (b0 === 255 && b1 === 255);
    const isInvalid = (b0 < 0 || b1 < 0);

    if (!isEcho && !isZero && !isFF && !isInvalid) {
        found = addr;
        brick.showString(`FOUND: ${addr}!`, 2);
        sensor.testLed(addr);
        pause(3000);
        break;
    }
    pause(100);
}

if (found < 0) {
    brick.showString("Step 3: Deep Scan 1-127", 2);
    pause(1000);
    line = 3;
    for (let a = 1; a < 128; a++) {
        const wire = a << 1; // standard I2C wire address
        const res = sensor.rawProbe(wire, 12, 4);
        brick.showString(`W:${wire} [${res}]`, line);
        line++;
        if (line > 10) {
            pause(1500);
            line = 3;
        }

        const parts = res.split(",");
        const b0 = parseInt(parts[0]);
        const b1 = parseInt(parts[1]);
        const isEcho = (b0 === 12 && b1 === 4) || (b0 === 4 && b1 === 12);
        const isZero = (b0 === 0 && b1 === 0);
        const isFF = (b0 === 255 && b1 === 255);
        const isInvalid = (b0 < 0 || b1 < 0);

        if (!isEcho && !isZero && !isFF && !isInvalid) {
            found = wire;
            brick.showString(`FOUND W:${wire}!`, 2);
            sensor.testLed(wire);
            pause(3000);
            break;
        }
        pause(50);
    }
}

if (found >= 0) {
    brick.showString(`SUCCESS Addr=${found}`, 2);
    // Continuous reading loop to verify data
    while (true) {
        const res = sensor.rawProbe(found, 12, 4);
        brick.showString(`ADC: [${res}]`, 3);
        pause(300);
    }
} else {
    brick.showString("NO ACK ON ANY ADDR", 2);
    brick.showString("Check LED on board", 3);
}

while (true) {
    pause(5000);
}
