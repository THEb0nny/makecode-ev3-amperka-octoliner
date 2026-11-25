// === OCTOLINER PORT 4 SYNC I2C DIAGNOSTIC (v0.1.9) ===
// Uses synchronous Repeat=1 transactions via /dev/lms_iic and provides
// an electrical SCL=0V test to measure V_OL with a multimeter.

const sensor = sensors.octoliner4; // TARGET: Port 4

pause(2000);

brick.showString("OCTOLINER PORT 4", 1);
brick.showString("Test LED 84 & 42...", 2);
sensor.testLed(84);
sensor.testLed(42);
pause(500);

// Priority addresses:
// 84 = 42 << 1 (standard 8-bit write address for Octoliner)
// 42 = 0x2A raw
// 132 = 0x42 << 1
// 66 = 0x42 raw
const priority = [84, 42, 132, 66, 85, 2, 16, 32, 64, 128];

brick.showString("Priority Scan (R=1):", 2);
let found = -1;
let line = 3;

for (let i = 0; i < priority.length; i++) {
    const addr = priority[i];
    const res = sensor.rawProbe(addr, 12, 4);
    const ioctlRes = sensor.lastIoctlResult();
    brick.showString(`A:${addr} r:${ioctlRes} [${res}]`, line);
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
        brick.showString(`FOUND A:${addr}!`, 2);
        sensor.testLed(addr);
        pause(3000);
        break;
    }
    pause(50);
}

if (found < 0) {
    brick.showString("Deep Scan 1..127:", 2);
    line = 3;
    for (let a = 1; a < 128; a++) {
        const wire = a << 1;
        const res = sensor.rawProbe(wire, 12, 4);
        if (a % 16 === 0) {
            brick.showString(`Scanning ${wire}... [${res}]`, 3);
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
            brick.showString(`FOUND W:${wire} [${res}]`, 2);
            sensor.testLed(wire);
            pause(3000);
            break;
        }
    }
}

if (found >= 0) {
    brick.showString(`SUCCESS Addr=${found}`, 2);
    while (true) {
        const res = sensor.rawProbe(found, 12, 4);
        brick.showString(`ADC ch0: [${res}]`, 3);
        pause(200);
    }
} else {
    brick.showString("NO ACK ON ANY ADDR", 2);
    brick.showString(`Last: ${sensor.debugInfo()}`, 3);
    brick.showString("Press DOWN = SCL LOW", 5);
    brick.showString("Press UP   = SCL FLOAT", 6);
    brick.showString("Measure G-SCL voltage!", 7);

    let sclLow = false;
    while (true) {
        if (brick.buttonDown.isPressed()) {
            sclLow = true;
            sensor.driveSclLow(true);
            brick.showString("STATE: SCL = 0V (LOW) ", 9);
            pause(300);
        }
        if (brick.buttonUp.isPressed()) {
            sclLow = false;
            sensor.driveSclLow(false);
            brick.showString("STATE: SCL = FLOAT    ", 9);
            pause(300);
        }
        if (sclLow) {
            // Refresh pin low periodically in case d_iic touches it
            sensor.driveSclLow(true);
        }
        pause(100);
    }
}
