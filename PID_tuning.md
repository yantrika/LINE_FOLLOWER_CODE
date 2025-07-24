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

For maximum speed:
Gradually increase baseSpeed until you see performance degradation.
Then fine-tune PID values to maintain control at higher speeds.
This code should give you significantly better performance while maintaining reliability. The adaptive elements help handle both straightaways and sharp turns optimally.

