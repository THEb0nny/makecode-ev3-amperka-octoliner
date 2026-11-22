// Test program for Amperka Octoliner line sensor
// Proportional line follower on Port 1

const baseSpeed = 40;
const kp = 0.6;

// Configure sensitivity
sensors.octoliner1.setSensitivity(208);

forever(function () {
    // Read deviation from center (-100 .. +100)
    let error = sensors.octoliner1.linePosition();
    let turn = error * kp;

    // Drive motors B and C
    motors.largeBC.tank(baseSpeed + turn, baseSpeed - turn);
});
