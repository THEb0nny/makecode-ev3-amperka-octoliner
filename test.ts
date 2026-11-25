// === OCTOLINER RAW I2C DIAGNOSTIC ===
// Scans wire addresses and shows raw buffer bytes WITHOUT any NACK filtering.
// This reveals whether the EV3 driver returns echoed TX bytes or a real response.

const sensor = sensors.octoliner1;

// Wait for EV3 I2C auto-detection to finish (~1 s after boot)
brick.pause(2000);

brick.showString("I2C SCAN", 1);
brick.showString("Port 4", 2);
brick.showString("Addr  B0  B1", 3);
brick.pause(1500);

// Priority addresses to test first: all interpretations of Octoliner default addr 42
// 42 = 0x2A (7-bit raw)
// 84 = 0x54 (42 << 1, write address)
// 85 = 0x55 (84 | 1, read address)
// 66 = 0x42 (alternative hex interpretation)
// 132 = 0x84 (66 << 1)
const priority = [42, 84, 85, 66, 132, 43, 86, 1, 2, 3, 16, 32, 48, 64, 80, 96, 112, 128];

let found = -1;
let line = 1;

for (let i = 0; i < priority.length; i++) {
    const addr = priority[i];
    const result = sensor.rawProbe(addr, 0x00);
    // Show on screen: "A:84 0,0"
    brick.showString(`A:${addr} [${result}]`, line);
    line++;
    if (line > 10) {
        line = 1;
        brick.pause(2000); // pause so user can read
    }
    // A non-zero, non-echo response means ACK
    // 0x00 sent as cmd, so echo would be [0,0] — NACK looks like [0,0]
    // But we also need to check if it's NOT [255,255] (floating)
    const parts = result.split(",");
    const b0 = parseInt(parts[0]);
    const b1 = parseInt(parts[1]);
    if (!((b0 === 0 && b1 === 0) || (b0 === 255 && b1 === 255) || (b0 === 0 && b1 === 255))) {
        found = addr;
        brick.showString(`FOUND: ${addr}`, line);
        break;
    }
    brick.pause(150);
}

if (found < 0) {
    // Deep scan: all addresses 1..127 raw and shifted
    brick.showString("Deep scan...", 1);
    brick.pause(500);
    line = 2;
    for (let addr = 1; addr < 128; addr++) {
        const result = sensor.rawProbe(addr, 0x0C); // CMD_ANALOG_READ=0x0C
        brick.showString(`A:${addr} [${result}]`, line);
        line++;
        if (line > 10) {
            line = 2;
            brick.pause(1500);
        }
        const parts = result.split(",");
        const b0 = parseInt(parts[0]);
        const b1 = parseInt(parts[1]);
        // If b0 != 0x0C (not an echo of cmd) AND not all-zero AND not all-FF
        if (!((b0 === 0x0C && b1 === addr) || (b0 === 0 && b1 === 0) || (b0 === 0xFF && b1 === 0xFF))) {
            found = addr;
            brick.showString(`FOUND: ${addr}`, line + 1);
            break;
        }
        pause(100);
    }
}

brick.pause(1000);
if (found >= 0) {
    brick.showString(`OK addr=${found}`, 1);
} else {
    brick.showString("NO DEVICE FOUND", 1);
    brick.showString("Check wiring!", 2);
}

// Keep result on screen
while (true) { brick.pause(5000); }
