To tune PID😭

Start with these base values:
Kp = 0.12
Ki = 0.0002
Kd = 0.25
baseSpeed = 200

Adjust based on behavior:
If oscillating: reduce Kp, increase Kd.
If sluggish: increase Kp.
If steady-state errors: slightly increase Ki.

**If robot wiggles: reduce Kp by 0.02 increments.
If slow to correct: increase Kp by 0.02 increments.
If overshooting turns: increase Kd by 0.05 increments. **

For maximum speed:
Gradually increase baseSpeed until you see performance degradation.
Then fine-tune PID values to maintain control at higher speeds.

**Increase baseSpeed in steps of 10 up to 220.
At each step, verify the robot can handle sharp turns.
If turns become unstable, increase Kd slightly. **
 
