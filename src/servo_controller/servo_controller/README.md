Current design of the finger:

![](./yechan_four_servo_control/pictures/servo.png)

Each cell is controlled by an individual servo motor, with three key positions:

Neutral: ~+20° (resting position)

Pull: ~–70° (rotates clockwise to pull the tendon downward)

Push: ~+45° (rotates counterclockwise to push the green bar upward)

This gives each servo a ~90° range of motion between the pull and push extremes. These angle values are approximate and will require fine-tuning after the full finger is manufactured.

Control can be implemented using either ROS topics or actions, depending on your preferred interface.

![](./yechan_four_servo_control/pictures/degreeofrotation.png)

Note on Cell Reset Mechanism

Currently, the green reset bar is a single solid piece. This means all servos (and therefore all cells) must reset simultaneously by rotating counterclockwise, regardless of their previous state.

**Edit 2025-08-03:** 4 separate green bar for each cell

However, to support future designs where cells can be reset individually, each servo’s reset command should remain separate. This ensures flexibility if we later switch to a more modular reset mechanism.

