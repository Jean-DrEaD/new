So with xdrive/ffbest the motor is strong — same PSU, same motor. The weakness is **in your firmware**. Let me find exactly where the torque is being lost.**Root cause: QI/DI are 7.2× too weak** after the formula correction.

The comment in config.h says the old code used `QI / (PWM_FREQ/3)` — the `/3` was a bug. The fix changed it to `QI / PWM_FREQ`. But `QI=500` was never scaled up to compensate. So the actual integrator gain went from `1200/(16000/3) = 0.225` down to `500/16000 = 0.031` — **7× weaker than xdrive**.

The integrator is what drives steady-state current. At standstill the proportional term contributes almost nothing (error is tiny once current starts flowing), so a weak integrator = weak holding torque.

**Fix in `config.h`, in `ONE_AXIS_USART_VARIANT`:**

```c
#define QP    0.8f
#define QI    3600.0f   // was 500 — 3600 = 1200*3 to match xdrive effective gain after formula fix
#define DP    0.7f
#define DI    3600.0f   // was 450 — same correction
#define CFG_CURR_FILT  0.35f   // keep
```

This is the only file that needs changing. Want me to generate the corrected `config.h`?