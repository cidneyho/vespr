Runs: 10. Errors in cm (heading in deg), pooled over every run.

**Moving**

| Error | Part | mean | RMS | 95 % | max |
|---|---|---|---|---|---|
| total | along | -8.7 | 13.8 | 26.4 | 32.7 |
| total | off_line | 12.8 | 13.7 | 21.4 | 27.4 |
| total | altitude | -5.1 | 5.7 | 10.2 | 14.3 |
| total | heading_deg | -3.18 | 4.44 | 8.82 | 9.39 |
| estimation | along | -2.6 | 9.8 | 19.0 | 22.1 |
| estimation | off_line | 11.8 | 12.9 | 20.8 | 24.9 |
| estimation | altitude | -4.9 | 4.9 | 6.6 | 7.3 |
| estimation | heading_deg | -3.20 | 4.45 | 8.82 | 9.46 |
| control | along | -6.1 | 8.2 | 15.8 | 18.5 |
| control | off_line | 1.9 | 2.5 | 5.6 | 9.9 |
| control | altitude | -0.3 | 2.6 | 6.1 | 9.8 |
| control | heading_deg | 0.02 | 0.31 | 0.41 | 4.39 |

**Turning**

| Error | Part | mean | RMS | 95 % | max |
|---|---|---|---|---|---|
| total | horizontal | 11.9 | 12.9 | 20.3 | 28.0 |
| total | altitude | -4.3 | 4.3 | 5.8 | 6.1 |
| total | heading_deg | -4.93 | 5.61 | 9.17 | 11.12 |
| estimation | horizontal | 12.5 | 13.5 | 21.3 | 26.4 |
| estimation | altitude | -4.9 | 5.0 | 6.9 | 7.0 |
| estimation | heading_deg | -4.86 | 5.93 | 9.41 | 9.94 |
| control | horizontal | 4.5 | 5.3 | 9.9 | 12.5 |
| control | altitude | 0.6 | 0.9 | 2.2 | 3.0 |
| control | heading_deg | -0.07 | 3.74 | 8.24 | 10.08 |

**Holding**

| Error | Part | mean | RMS | 95 % | max |
|---|---|---|---|---|---|
| total | horizontal | 15.0 | 16.6 | 27.4 | 35.6 |
| total | altitude | -4.6 | 4.8 | 6.1 | 14.5 |
| total | heading_deg | -4.56 | 5.71 | 9.39 | 14.72 |
| estimation | horizontal | 14.1 | 15.6 | 25.4 | 33.2 |
| estimation | altitude | -4.9 | 5.0 | 6.7 | 7.2 |
| estimation | heading_deg | -4.61 | 5.62 | 9.30 | 9.69 |
| control | horizontal | 4.3 | 6.3 | 15.6 | 20.8 |
| control | altitude | 0.3 | 1.3 | 2.6 | 11.3 |
| control | heading_deg | 0.05 | 1.34 | 2.76 | 10.19 |

**Settling: the actual position (mocap) vs the setpoint** (tolerance 5 cm). Mean / max over the runs.

| # | After | settle time s | not settled | overshoot cm | final error cm | wobble cm |
|---|---|---|---|---|---|---|
| 0 | up 0.9 m |  | 10 of 10 | 0.0 / 0.0 | 16.6 / 18.5 | 5.9 / 6.5 |
| 1 | north 1.0 m |  | 10 of 10 | 8.4 / 11.1 | 10.8 / 12.8 | 1.2 / 1.4 |
| 2 | east 1.0 m |  | 10 of 10 | 15.0 / 18.0 | 8.9 / 10.6 | 1.1 / 1.2 |
| 3 | turn +90 |  | 10 of 10 | 3.4 / 5.1 deg | 24.4 / 31.8 | 1.3 / 1.8 |
| 4 | south 1.0 m |  | 10 of 10 | 1.3 / 8.4 | 21.7 / 27.4 | 0.9 / 1.1 |
| 5 | west 1.0 m |  | 10 of 10 | 27.3 / 31.9 | 13.6 / 18.7 | 1.1 / 1.3 |
| 6 | up 0.5 m |  | 10 of 10 | 1.1 / 7.1 | 17.6 / 23.4 | 1.1 / 1.4 |
| 7 | down 0.5 m |  | 10 of 10 | 10.2 / 14.5 | 15.2 / 20.3 | 0.9 / 1.1 |
| 8 | turn -90 | 4.18 / 4.18 | 9 of 10 | 12.5 / 14.7 deg | 10.6 / 20.9 | 1.1 / 1.3 |

**Settling: PX4's estimate vs the setpoint** (tolerance 5 cm). Mean / max over the runs.

| # | After | settle time s | not settled | overshoot cm | final error cm | wobble cm |
|---|---|---|---|---|---|---|
| 0 | up 0.9 m | 4.74 / 4.74 | 9 of 10 | 0.0 / 0.0 | 9.3 / 11.3 | 5.7 / 6.1 |
| 1 | north 1.0 m | 2.00 / 2.12 | 0 of 10 | 17.3 / 20.4 | 2.1 / 3.6 | 0.9 / 1.1 |
| 2 | east 1.0 m | 1.98 / 2.04 | 0 of 10 | 17.4 / 19.6 | 1.8 / 2.6 | 0.7 / 0.9 |
| 3 | turn +90 | 2.27 / 2.34 | 0 of 10 | 7.8 / 9.6 deg | 2.2 / 3.8 | 1.2 / 1.6 |
| 4 | south 1.0 m | 1.84 / 1.90 | 0 of 10 | 16.4 / 20.7 | 2.0 / 2.5 | 0.5 / 0.7 |
| 5 | west 1.0 m | 2.01 / 2.06 | 0 of 10 | 17.7 / 19.8 | 1.6 / 2.1 | 0.8 / 1.0 |
| 6 | up 0.5 m | 0.35 / 0.92 | 0 of 10 | 5.6 / 11.3 | 0.8 / 1.3 | 1.2 / 1.6 |
| 7 | down 0.5 m | 0.36 / 0.66 | 0 of 10 | 5.2 / 8.1 | 1.1 / 1.6 | 1.0 / 1.1 |
| 8 | turn -90 | 2.09 / 2.30 | 0 of 10 | 7.5 / 10.2 deg | 2.0 / 2.8 | 1.1 / 1.4 |

