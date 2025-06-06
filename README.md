# PI Controller Tuning Quick Guide

## Quick Start Parameters

For most conveyor applications, start with these parameters in `main.c`:

```c
#define KP_GAIN 0.8f      // Proportional gain
#define KI_GAIN 0.3f      // Integral gain
```

## Step-by-Step Tuning Process

### Step 1: Test P-Only Controller
1. Set `KI_GAIN = 0.0f` (disable integral)
2. Start with `KP_GAIN = 0.5f`
3. Run the system and observe response on LCD
4. Gradually increase Kp until you see oscillations
5. Reduce Kp by 25-50% from oscillation point

### Step 2: Add Integral Action
1. Keep the Kp value from Step 1
2. Start with `KI_GAIN = 0.1f`
3. Gradually increase Ki until steady-state error is eliminated
4. Stop increasing Ki if oscillations appear

### Step 3: Fine Tuning
Adjust based on system behavior:

## Tuning Table

| **Problem** | **Solution** | **Parameter Change** |
|-------------|--------------|---------------------|
| Too slow to reach setpoint | Increase response speed | Increase Kp by 0.1-0.2 |
| Overshoots setpoint | Reduce aggressiveness | Decrease Kp by 0.1-0.2 |
| Oscillates around setpoint | Too much gain | Decrease Kp by 0.2-0.3 |
| Steady-state error | Need integral action | Increase Ki by 0.05-0.1 |
| Takes too long to eliminate error | Faster integral action | Increase Ki by 0.1-0.2 |
| Integral windup/instability | Too much integral | Decrease Ki by 0.1-0.2 |

## Performance Indicators

### Good Tuning Signs:
- ✅ Reaches setpoint in 3-5 seconds
- ✅ No overshoot or minimal overshoot (<10%)
- ✅ No oscillations in steady state
- ✅ Zero steady-state error
- ✅ Good disturbance rejection

### Poor Tuning Signs:
- ❌ Takes >10 seconds to reach setpoint
- ❌ Large overshoot (>20%)
- ❌ Continuous oscillations
- ❌ Persistent steady-state error
- ❌ Poor load disturbance response

## Load Testing

### Test Procedure:
1. Set system to 100 RPM
2. Let it stabilize for 30 seconds
3. Apply additional load (increase belt tension, add weight)
4. Observe how quickly it returns to 100 RPM
5. Remove load and observe recovery

### Expected Response:
- **Good tuning**: Returns to setpoint within 2-3 seconds
- **Poor tuning**: Takes >5 seconds or oscillates

## Emergency Procedures

### If System Becomes Unstable:
1. **Immediately set**: `KP_GAIN = 0.3f`, `KI_GAIN = 0.1f`
2. **Restart system**
3. **Begin tuning process again**

### Safe Parameter Ranges:
- **Kp range**: 0.1 - 2.0 (start conservatively)
- **Ki range**: 0.0 - 1.0 (start with 0.1)

## Example Tuning Sessions

### Session 1: Conservative System
```c
// Start
#define KP_GAIN 0.5f
#define KI_GAIN 0.0f
// Result: Slow but stable

// Add integral
#define KP_GAIN 0.5f
#define KI_GAIN 0.2f
// Result: Good performance

// Final optimization
#define KP_GAIN 0.6f
#define KI_GAIN 0.25f
// Result: Excellent response
```

### Session 2: Aggressive System
```c
// Start
#define KP_GAIN 1.0f
#define KI_GAIN 0.0f
// Result: Fast but oscillates

// Reduce proportional
#define KP_GAIN 0.7f
#define KI_GAIN 0.0f
// Result: Stable, add integral

// Add integral
#define KP_GAIN 0.7f
#define KI_GAIN 0.3f
// Result: Good performance
```

## Monitoring During Tuning

### Watch These Values on LCD:
- **Current RPM**: Should closely track setpoint
- **PWM%**: Should be steady in steady-state
- **Response time**: How long to reach setpoint after disturbance

### Use Rotary Encoder:
- Change setpoint from 100 to 120 RPM
- Observe settling behavior
- Change back to 100 RPM
- Both transitions should be smooth

## Advanced Tips

### For Heavy Loads:
- Start with higher Kp (0.8-1.2)
- Use moderate Ki (0.2-0.4)
- Monitor for saturation (PWM at 100%)

### For Light Loads:
- Use moderate Kp (0.5-0.8)
- Can use higher Ki (0.3-0.6)
- Watch for oscillations

### For Variable Loads:
- Use balanced parameters (Kp=0.7, Ki=0.3)
- Test with different load conditions
- Optimize for worst-case scenario

## Parameter Modification in Code

To change parameters during development:

1. **Edit main.c**:
   ```c
   #define KP_GAIN 0.8f    // Change this value
   #define KI_GAIN 0.3f    // Change this value
   ```

2. **Rebuild and flash**

3. **Test new parameters**

4. **Repeat until satisfied**

## Final Validation

### Complete System Test:
1. Start at 100 RPM setpoint
2. Use rotary encoder to change to 80 RPM
3. Change to 120 RPM
4. Return to 100 RPM
5. Apply manual load disturbance
6. Verify smooth operation in all cases

### Acceptable Performance Criteria:
- **Settling time**: <5 seconds
- **Overshoot**: <15%
- **Steady-state error**: <2 RPM
- **Load disturbance recovery**: <3 seconds

Once these criteria are met, your PI controller is properly tuned for your specific conveyor system.