# Failing seeds, overnight sweep

Seeds 1800–201799 (200,000 seeds) on commit 2079703, run Nov 16 2026: 8 h on 2 cores, deterministic replay OK in every chunk, no coverage (THIN) warnings, 59–61% of simulated time airborne.

The `dst` runner prints only the first 3 failing seeds per property per run, and the sweep ran in 10,000-seed chunks. So a list below is complete only where it says so; the others are samples. Replay any seed with `cargo run --release -p ironloop-sim --bin dst -- --seed N --trace`. Seed numbers stay valid only while the scenario generator is unchanged.

| Property | Seeds failing | Listed | Complete |
| --- | --- | --- | --- |
| `estimate_tracks` | 2698 | 60 | no |
| `failsafe_lands_softly` | 1468 | 60 | no |
| `failsafe_descends` | 166 | 60 | no |
| `no_crash` | 63 | 48 | no |
| `crash_stops` | 61 | 50 | no |
| `failsafe_holds` | 49 | 39 | no |
| `arm_level` | 47 | 41 | no |
| `failsafe_disarms_on_touchdown` | 42 | 40 | no |
| `disarm_justified` | 41 | 36 | no |
| `no_flyaway` | 25 | 25 | yes |
| `failsafe_levels` | 24 | 24 | yes |
| `pilot_can_arm` | 17 | 17 | yes |
| `angle_stays_level` | 12 | 11 | no |
| `armed_above_idle` | 10 | 10 | yes |
| `motor_fault_true` | 7 | 7 | yes |
| `runaway_stops` | 6 | 6 | yes |
| `esc_fed` | 3 | 3 | yes |
| `motor_fault_reported` | 3 | 3 | yes |
| `rate_tracks` | 1 | 1 | yes |

## `estimate_tracks` (2698 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 1801 | 17.500 s | tilt estimate off by 17.8 deg | impact,imu-rotated,mode-switch,moved-at-boot,stick-slam |
| 1824 | 14.100 s | tilt estimate off by 15.7 deg | flip,mode-switch,stick-script,stick-slam |
| 1832 | 14.071 s | tilt estimate off by 25.8 deg | fc-stall,imu-dead-at-arm,inverted-hang,late-link,mode-switch,no-telemetry,punch,stick-slam,wind4 |
| 11814 | 20.900 s | tilt estimate off by 25.1 deg | arm-on-boot,mode-switch,punch,stick-slam |
| 12099 | 10.600 s | tilt estimate off by 36.3 deg | bent-prop,mode-switch,old-pack,stick-slam,wind2 |
| 12104 | 17.572 s | tilt estimate off by 16.9 deg | imu-dead-at-arm,moved-at-boot,old-pack,rc-loss,stick-slam,wind3 |
| 21813 | 9.600 s | tilt estimate off by 27.5 deg | fc-stall,imu-dead-at-arm,mode-switch,punch,stick-slam |
| 21845 | 11.200 s | tilt estimate off by 17.7 deg | imu-rotated,moved-at-boot,no-telemetry,old-pack,stick-slam,wind1 |
| 21846 | 8.600 s | tilt estimate off by 18.5 deg | mode-switch,punch,stick-slam |
| 31902 | 18.798 s | tilt estimate off by 16.9 deg | imu-rotated,kick,low-battery,no-telemetry,punch,rc-loss-airborne |
| 31911 | 20.096 s | tilt estimate off by 24.1 deg | clock-glitch,kick,low-battery,mode-switch,no-telemetry,stick-slam |
| 32008 | 11.200 s | tilt estimate off by 15.6 deg | inverted-hang,low-battery,mode-switch,rc-glitch,stick-script,stick-slam |
| 41904 | 23.916 s | tilt estimate off by 15.0 deg | imu-rotated,mode-switch,no-telemetry,punch,stick-script,wind1 |
| 41907 | 8.731 s | tilt estimate off by 18.0 deg | drop,impact,motor-degraded,moved-at-boot,no-telemetry |
| 42027 | 13.571 s | tilt estimate off by 16.2 deg | clock-wrap,desync,fc-stall,mode-switch,no-telemetry,old-pack,stick-slam |
| 51818 | 26.805 s | tilt estimate off by 15.0 deg | imu-rotated,punch,stick-slam,wind4 |
| 51857 | 8.900 s | tilt estimate off by 119.1 deg | bent-prop,impact,kick,mode-switch,on-its-side,rc-glitch,stick-slam |
| 51893 | 9.900 s | tilt estimate off by 15.4 deg | inverted-hang,mode-switch,moved-at-boot,no-telemetry,old-pack,punch,rc-loss,stick-slam |
| 61827 | 12.181 s | tilt estimate off by 27.9 deg | arm-on-boot,bent-prop,imu-rotated,late-link,stick-slam |
| 61886 | 11.000 s | tilt estimate off by 15.6 deg | flip,mode-switch,stick-slam,wind0 |
| 61972 | 12.316 s | tilt estimate off by 29.8 deg | arm-on-boot,desync,fc-stall,flip,late-link,stick-script,stick-slam |
| 71816 | 10.901 s | tilt estimate off by 17.3 deg | clock-wrap,event-drop,no-telemetry,stick-script,stick-slam |
| 71904 | 10.200 s | tilt estimate off by 15.9 deg | imu-rotated,mode-switch,stick-slam,wind3 |
| 72249 | 7.808 s | tilt estimate off by 15.0 deg | flip,imu-rotated,mode-switch,rc-loss-airborne,wind3 |
| 81867 | 10.400 s | tilt estimate off by 30.5 deg | clock-glitch,no-telemetry,rc-loss-airborne,stick-slam,wind-gust |
| 81913 | 11.500 s | tilt estimate off by 26.1 deg | jitter,kick,mode-switch,no-telemetry,old-pack,stick-slam,wind-gust |
| 81917 | 35.072 s | tilt estimate off by 17.0 deg | arm-on-boot,arm-throttle-up,bent-prop,clock-wrap,fc-stall,flip,imu-dropout,late-link,mode-switch,no-telemetry,stick-slam |
| 91867 | 35.920 s | tilt estimate off by 16.3 deg | desync,flip,imu-dead-at-arm,inverted-hang,kick,mode-switch,no-telemetry,punch,rc-loss-airborne,stick-slam |
| 92092 | 20.901 s | tilt estimate off by 49.1 deg | imu-dead-at-arm,imu-rotated,mode-switch,no-telemetry,rc-loss,stick-slam |
| 92176 | 11.521 s | tilt estimate off by 21.6 deg | arm-on-boot,clock-wrap,flip,imu-stuck,late-link,mode-switch,rc-glitch,wind2 |
| 101816 | 11.301 s | tilt estimate off by 20.9 deg | bent-prop,fc-stall,no-telemetry,stick-slam,wind1 |
| 101832 | 18.001 s | tilt estimate off by 19.3 deg | kick,mode-switch,punch,rc-loss-airborne,stick-slam,telemetry-loss |
| 101856 | 11.200 s | tilt estimate off by 19.6 deg | arm-throttle-up,clock-wrap,inverted-hang,on-its-side,rc-loss,stick-script,stick-slam,wind3 |
| 111802 | 15.000 s | tilt estimate off by 22.0 deg | bent-prop,imu-dead-at-arm,imu-stuck,inverted-hang,mode-switch,rc-glitch,rc-loss-airborne,stick-script,stick-slam,wind-gust |
| 111852 | 20.060 s | tilt estimate off by 87.0 deg | arm-on-boot,desync,mode-switch,no-telemetry,stick-script,stick-slam |
| 111913 | 12.100 s | tilt estimate off by 15.8 deg | bent-prop,clock-glitch,inverted-hang,kick,no-telemetry,rc-glitch,stick-slam |
| 121866 | 10.000 s | tilt estimate off by 26.4 deg | clock-wrap,imu-rotated,mode-switch,stick-slam |
| 121945 | 11.706 s | tilt estimate off by 23.4 deg | arm-on-boot,arm-throttle-up,desync,flip,late-link,low-battery,rc-loss-airborne,stick-script,stick-slam,wind4 |
| 122051 | 10.501 s | tilt estimate off by 98.0 deg | bent-prop,flip,impact,imu-garbage,imu-rotated,inverted-hang,stick-slam,wind2 |
| 131823 | 8.900 s | tilt estimate off by 33.5 deg | clock-wrap,imu-rotated,inverted-hang,no-telemetry,reboot-request,stick-slam,wind2 |
| 131841 | 66.000 s | tilt estimate off by 15.1 deg | bent-prop,inverted-hang,moved-at-boot,no-telemetry,old-pack,stick-script,stick-slam |
| 132104 | 21.700 s | tilt estimate off by 17.7 deg | arm-on-boot,arm-throttle-up,bent-prop,imu-rotated,mode-switch,moved-at-boot,no-telemetry,on-its-side,rc-loss-airborne,stick-slam |
| 141953 | 18.600 s | tilt estimate off by 20.4 deg | arm-on-boot,imu-rotated,mode-switch,reboot-request,stick-slam |
| 142033 | 14.590 s | tilt estimate off by 23.7 deg | clock-wrap,desync,impact,low-battery,mode-switch,motor-degraded,no-telemetry,punch,rc-loss-airborne,wind4 |
| 142134 | 17.339 s | tilt estimate off by 16.2 deg | clock-wrap,impact,late-link,mode-switch,stick-slam,wind-gust |
| 151888 | 25.925 s | tilt estimate off by 27.9 deg | arm-on-boot,arm-throttle-up,imu-rotated,kick,late-link,mode-switch,no-telemetry,stick-script,stick-slam |
| 152031 | 14.501 s | tilt estimate off by 25.5 deg | flip,low-battery,mode-switch,no-telemetry,stick-slam |
| 152097 | 24.800 s | tilt estimate off by 23.6 deg | arm-throttle-up,imu-rotated,mode-switch,old-pack,punch,rc-loss,stick-slam |
| 161973 | 26.600 s | tilt estimate off by 26.4 deg | bent-prop,event-drop,flip,impact,reboot-request,stick-slam,wind1 |
| 162076 | 19.401 s | tilt estimate off by 24.6 deg | clock-glitch,imu-dead-at-arm,imu-stuck,mode-switch,old-pack,rc-loss,stick-slam,wind1 |
| 162185 | 7.989 s | tilt estimate off by 19.5 deg | kick,punch,wind3 |
| 171820 | 21.800 s | tilt estimate off by 15.4 deg | mode-switch,no-telemetry,stick-slam,wind2 |
| 171841 | 10.300 s | tilt estimate off by 16.8 deg | arm-on-boot,desync,imu-rotated,low-battery,no-telemetry,rc-loss-airborne,stick-script,stick-slam |
| 171939 | 18.299 s | tilt estimate off by 22.8 deg | arm-on-boot,flip,imu-rotated,late-link,mode-switch,stick-slam |
| 181913 | 13.296 s | tilt estimate off by 15.0 deg | clock-glitch,jitter,kick,mode-switch,punch |
| 181963 | 10.500 s | tilt estimate off by 22.0 deg | bent-prop,event-drop,imu-dropout,imu-rotated,inverted-hang,mode-switch,no-telemetry,punch,reboot-request,stick-slam |
| 181979 | 33.700 s | tilt estimate off by 16.5 deg | flip,stick-script,stick-slam |
| 191800 | 10.300 s | tilt estimate off by 18.2 deg | old-pack,stick-script,stick-slam,wind3 |
| 191951 | 10.481 s | tilt estimate off by 15.6 deg | bent-prop,flip,imu-dead-at-arm,imu-stuck,jitter,late-link,rc-loss-airborne,stick-slam,wind4 |
| 191979 | 17.654 s | tilt estimate off by 19.0 deg | impact,imu-rotated,imu-stuck,mode-switch,no-telemetry,stick-slam,telemetry-loss,wind3 |

## `failsafe_lands_softly` (1468 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 1825 | 32.277 s | touched down at 3.1 m/s vertical | arm-on-boot,arm-throttle-up,drop,late-link,mode-switch,on-its-side,rc-loss-airborne,wind2 |
| 1979 | 68.250 s | touched down at 2.5 m/s vertical | imu-stuck,inverted-hang,mode-switch,no-telemetry,rc-loss-airborne,wind1 |
| 2182 | 36.086 s | touched down at 2.7 m/s vertical | bent-prop,mode-switch,no-telemetry,rc-loss-airborne,stick-slam |
| 12049 | 35.185 s | touched down at 2.8 m/s vertical | clock-wrap,low-battery,rc-loss-airborne,stick-slam,wind1 |
| 12067 | 28.323 s | touched down at 2.8 m/s vertical | event-drop,imu-dropout,no-telemetry,punch,rc-loss-airborne,wind0 |
| 12077 | 77.361 s | touched down at 2.8 m/s vertical | bent-prop,inverted-hang,mode-switch,no-telemetry,punch,rc-loss,rc-loss-airborne,stick-script,wind4 |
| 21913 | 33.563 s | touched down at 2.5 m/s vertical | arm-on-boot,flip,low-battery,mode-switch,rc-loss-airborne,wind1 |
| 21918 | 28.283 s | touched down at 2.6 m/s vertical | imu-garbage,mode-switch,rc-loss-airborne,stick-slam,wind0 |
| 22057 | 12.628 s | touched down at 3.5 m/s vertical | bent-prop,clock-wrap,event-drop,flip,imu-rotated,mode-switch,no-telemetry,rc-loss,stick-script,stick-slam,wind0 |
| 32145 | 72.870 s | touched down at 2.6 m/s vertical | desync,flip,imu-dead-at-arm,imu-rotated,imu-stuck,inverted-hang,jitter,moved-at-boot,rc-loss-airborne,wind2 |
| 32470 | 42.887 s | touched down at 2.6 m/s vertical | arm-throttle-up,flip,imu-garbage,mode-switch,rc-loss-airborne,stick-slam,wind0 |
| 32485 | 44.840 s | touched down at 2.6 m/s vertical | mode-switch,moved-at-boot,old-pack,punch,rc-loss-airborne,stick-script |
| 41842 | 50.302 s | touched down at 2.5 m/s vertical | arm-on-boot,imu-rotated,late-link,rc-loss-airborne,stick-script,stick-slam,wind1 |
| 41843 | 26.472 s | touched down at 2.6 m/s vertical | bent-prop,kick,mode-switch,rc-loss-airborne,wind1 |
| 42499 | 52.496 s | touched down at 2.9 m/s vertical | arm-on-boot,flip,mode-switch,no-telemetry,old-pack,rc-loss,rc-loss-airborne,stick-script,stick-slam |
| 51817 | 38.809 s | touched down at 2.6 m/s vertical | arm-throttle-up,imu-rotated,mode-switch,old-pack,rc-loss-airborne,stick-slam,wind0 |
| 52016 | 34.561 s | touched down at 2.6 m/s vertical | mode-switch,rc-loss-airborne,wind2 |
| 52038 | 36.844 s | touched down at 2.6 m/s vertical | drop,flip,imu-dead-at-arm,imu-rotated,late-link,rc-loss-airborne,reboot-request |
| 62024 | 33.790 s | touched down at 2.5 m/s vertical | clock-wrap,imu-dead-at-arm,mode-switch,rc-loss-airborne,stick-script,stick-slam |
| 62096 | 35.058 s | touched down at 2.5 m/s vertical | flip,imu-rotated,jitter,late-link,mode-switch,no-telemetry,old-pack,rc-loss-airborne,wind3 |
| 62245 | 52.468 s | touched down at 2.6 m/s vertical | arm-throttle-up,flip,imu-garbage,imu-rotated,rc-glitch,rc-loss-airborne,stick-script |
| 72006 | 75.592 s | touched down at 2.6 m/s vertical | flip,inverted-hang,mode-switch,rc-loss-airborne |
| 72007 | 32.664 s | touched down at 2.6 m/s vertical | imu-rotated,no-telemetry,old-pack,rc-loss-airborne,reboot-request,stick-slam,wind-gust |
| 72069 | 91.505 s | touched down at 2.7 m/s vertical | clock-wrap,flip,imu-rotated,inverted-hang,late-link,mode-switch,no-telemetry,punch,rc-loss-airborne,stick-script,stick-slam |
| 81836 | 29.175 s | touched down at 2.6 m/s vertical | bent-prop,imu-rotated,no-telemetry,rc-loss-airborne,stick-slam,wind1 |
| 81874 | 40.302 s | touched down at 3.0 m/s vertical | clock-wrap,mode-switch,moved-at-boot,no-telemetry,rc-loss-airborne,stick-script,wind3 |
| 81956 | 35.798 s | touched down at 2.7 m/s vertical | clock-glitch,clock-wrap,no-telemetry,rc-loss-airborne,stick-slam,wind0 |
| 91922 | 28.465 s | touched down at 2.6 m/s vertical | arm-on-boot,clock-wrap,late-link,old-pack,rc-loss-airborne |
| 92564 | 34.786 s | touched down at 2.5 m/s vertical | clock-wrap,imu-rotated,low-battery,mode-switch,no-telemetry,punch,rc-loss-airborne,wind4 |
| 92709 | 35.117 s | touched down at 2.8 m/s vertical | imu-garbage,imu-rotated,punch,rc-loss-airborne,wind2 |
| 102002 | 71.661 s | touched down at 2.5 m/s vertical | inverted-hang,mode-switch,no-telemetry,punch,rc-loss-airborne,wind1 |
| 102050 | 38.287 s | touched down at 2.7 m/s vertical | arm-throttle-up,bent-prop,impact,moved-at-boot,no-telemetry,old-pack,rc-loss-airborne,stick-slam |
| 102107 | 68.067 s | touched down at 2.8 m/s vertical | inverted-hang,no-telemetry,old-pack,rc-loss-airborne,stick-script |
| 112146 | 37.028 s | touched down at 2.5 m/s vertical | arm-throttle-up,mode-switch,rc-loss-airborne,stick-script |
| 112705 | 100.853 s | touched down at 2.7 m/s vertical | arm-on-boot,imu-rotated,inverted-hang,rc-loss-airborne |
| 112757 | 74.209 s | touched down at 2.6 m/s vertical | bent-prop,drop,flip,imu-garbage,inverted-hang,mode-switch,rc-loss,rc-loss-airborne,telemetry-loss |
| 121843 | 42.419 s | touched down at 2.6 m/s vertical | arm-throttle-up,clock-wrap,imu-rotated,mode-switch,no-telemetry,old-pack,rc-loss-airborne,stick-script,stick-slam |
| 122143 | 34.395 s | touched down at 2.5 m/s vertical | imu-rotated,mode-switch,no-telemetry,punch,rc-loss-airborne,stick-slam,wind2 |
| 122343 | 26.525 s | touched down at 2.7 m/s vertical | no-telemetry,old-pack,rc-loss-airborne |
| 131814 | 103.768 s | touched down at 2.8 m/s vertical | desync,flip,imu-garbage,inverted-hang,late-link,no-telemetry,old-pack,on-its-side,rc-loss-airborne,telemetry-loss,wind3 |
| 131827 | 34.333 s | touched down at 2.8 m/s vertical | arm-throttle-up,bent-prop,clock-wrap,late-link,no-telemetry,rc-loss-airborne,stick-slam,telemetry-loss |
| 132523 | 37.119 s | touched down at 2.6 m/s vertical | event-drop,imu-rotated,mode-switch,no-telemetry,rc-loss-airborne,stick-script,wind1 |
| 141992 | 31.122 s | touched down at 2.5 m/s vertical | bent-prop,rc-loss-airborne,stick-script,wind2 |
| 142066 | 35.120 s | touched down at 2.9 m/s vertical | flip,late-link,mode-switch,moved-at-boot,no-telemetry,old-pack,rc-loss-airborne,wind3 |
| 142160 | 60.072 s | touched down at 6.0 m/s vertical | arm-throttle-up,desync,flip,imu-rotated,inverted-hang,rc-glitch,rc-loss,stick-slam |
| 151845 | 65.206 s | touched down at 2.6 m/s vertical | bent-prop,imu-rotated,inverted-hang,moved-at-boot,no-telemetry,rc-loss-airborne,stick-slam |
| 152067 | 101.294 s | touched down at 2.5 m/s vertical | imu-rotated,inverted-hang,mode-switch,no-telemetry,old-pack,punch,rc-loss-airborne,stick-slam,wind1 |
| 152165 | 75.925 s | touched down at 3.1 m/s vertical | fc-stall,inverted-hang,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,stick-script |
| 161995 | 29.295 s | touched down at 2.5 m/s vertical | clock-wrap,kick,mode-switch,on-its-side,rc-loss-airborne |
| 162098 | 60.268 s | touched down at 2.6 m/s vertical | clock-wrap,desync,mode-switch,rc-loss-airborne,reboot-request,stick-script,stick-slam,wind0 |
| 162158 | 81.762 s | touched down at 2.7 m/s vertical | clock-wrap,flip,inverted-hang,kick,mode-switch,no-telemetry,rc-loss-airborne,stick-script,stick-slam |
| 172089 | 60.791 s | touched down at 2.7 m/s vertical | imu-dead-at-arm,inverted-hang,mode-switch,rc-loss-airborne,stick-slam,wind4 |
| 172255 | 56.924 s | touched down at 2.5 m/s vertical | imu-rotated,inverted-hang,mode-switch,old-pack,rc-loss-airborne |
| 172520 | 32.985 s | touched down at 2.9 m/s vertical | bent-prop,clock-wrap,mode-switch,no-telemetry,on-its-side,rc-loss-airborne |
| 181849 | 40.282 s | touched down at 2.5 m/s vertical | clock-glitch,imu-dropout,mode-switch,old-pack,punch,rc-loss,rc-loss-airborne |
| 182104 | 77.808 s | touched down at 2.6 m/s vertical | flip,impact,inverted-hang,no-telemetry,old-pack,rc-loss-airborne,stick-script |
| 182214 | 44.600 s | touched down at 2.5 m/s vertical | arm-on-boot,flip,no-telemetry,old-pack,rc-loss-airborne,stick-script |
| 191830 | 32.782 s | touched down at 2.5 m/s vertical | flip,mode-switch,old-pack,rc-loss,rc-loss-airborne |
| 191875 | 59.451 s | touched down at 2.5 m/s vertical | inverted-hang,mode-switch,no-telemetry,rc-loss-airborne,reboot-request |
| 191900 | 34.337 s | touched down at 2.5 m/s vertical | clock-glitch,desync,no-telemetry,punch,rc-loss-airborne |

## `failsafe_descends` (166 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 2055 | 21.055 s | sinking 0.06 m/s at 8.1 m | drop,mode-switch,moved-at-boot,no-telemetry,old-pack,rc-loss,stick-slam |
| 4193 | 18.887 s | sinking -0.71 m/s at 17.6 m | bent-prop,fc-stall,mode-switch,punch,rc-loss,stick-slam |
| 5109 | 34.137 s | sinking -0.13 m/s at 32.3 m | drop,flip,inverted-hang,mode-switch,rc-loss,stick-slam,wind4 |
| 13960 | 25.971 s | sinking -2.02 m/s at 56.0 m | drop,inverted-hang,kick,no-telemetry,rc-loss,stick-slam,wind1 |
| 14091 | 27.969 s | sinking -0.16 m/s at 53.0 m | inverted-hang,jitter,mode-switch,no-telemetry,punch,rc-loss,rc-loss-airborne,stick-slam,wind0 |
| 17959 | 32.994 s | sinking -0.48 m/s at 17.5 m | clock-wrap,flip,mode-switch,punch,rc-loss,rc-loss-airborne,wind1 |
| 25132 | 32.978 s | sinking -1.26 m/s at 38.0 m | inverted-hang,mode-switch,no-telemetry,rc-loss,stick-script,stick-slam,wind1 |
| 26275 | 23.676 s | sinking -0.67 m/s at 17.2 m | arm-throttle-up,imu-rotated,mode-switch,rc-loss,rc-loss-airborne |
| 26846 | 15.316 s | sinking -0.25 m/s at 19.1 m | clock-wrap,desync,flip,imu-rotated,inverted-hang,old-pack,rc-loss,wind-gust,wind1 |
| 31882 | 27.085 s | sinking 0.00 m/s at 21.8 m | arm-on-boot,arm-throttle-up,mode-switch,rc-loss,rc-loss-airborne,wind1 |
| 34192 | 20.556 s | sinking -0.86 m/s at 24.6 m | arm-on-boot,bent-prop,imu-rotated,inverted-hang,mode-switch,rc-loss,wind4 |
| 34667 | 44.451 s | sinking -1.28 m/s at 49.1 m | inverted-hang,rc-loss,rc-loss-airborne,stick-script,stick-slam |
| 42345 | 34.142 s | sinking -1.03 m/s at 20.4 m | arm-throttle-up,flip,imu-dropout,imu-rotated,inverted-hang,mode-switch,no-telemetry,punch,rc-loss |
| 43391 | 37.518 s | sinking -1.21 m/s at 19.7 m | flip,imu-dropout,imu-rotated,mode-switch,no-telemetry,old-pack,rc-loss,rc-loss-airborne,wind-gust |
| 44900 | 15.830 s | sinking -0.05 m/s at 13.0 m | flip,mode-switch,rc-loss,rc-loss-airborne,stick-script,wind1 |
| 51952 | 23.837 s | sinking -0.29 m/s at 9.8 m | arm-on-boot,flip,imu-dropout,imu-rotated,mode-switch,old-pack,rc-loss,stick-slam,wind1 |
| 53243 | 61.001 s | sinking -0.07 m/s at 38.5 m | arm-on-boot,desync,imu-rotated,inverted-hang,mode-switch,no-telemetry,rc-glitch,rc-loss |
| 53385 | 17.723 s | sinking -0.41 m/s at 15.0 m | mode-switch,old-pack,rc-loss,reboot-request,stick-script |
| 64870 | 19.575 s | sinking -0.08 m/s at 10.9 m | arm-throttle-up,flip,late-link,mode-switch,moved-at-boot,old-pack,rc-loss,wind1 |
| 67072 | 23.416 s | sinking -0.67 m/s at 21.4 m | moved-at-boot,rc-loss,rc-loss-airborne,reboot-request |
| 67503 | 25.127 s | sinking -0.99 m/s at 50.8 m | flip,imu-dropout,imu-rotated,inverted-hang,late-link,mode-switch,rc-loss,reboot-request,stick-slam,wind2 |
| 72496 | 49.005 s | sinking -0.77 m/s at 13.6 m | clock-wrap,imu-rotated,imu-stuck,inverted-hang,mode-switch,no-telemetry,rc-loss,wind1 |
| 73785 | 24.868 s | sinking -0.38 m/s at 15.4 m | bent-prop,clock-wrap,late-link,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,stick-slam |
| 75682 | 37.887 s | sinking -0.45 m/s at 25.3 m | inverted-hang,mode-switch,no-telemetry,punch,rc-loss,reboot-request,stick-script,wind1 |
| 82323 | 35.197 s | sinking -1.14 m/s at 22.2 m | arm-on-boot,event-drop,imu-stuck,mode-switch,no-telemetry,punch,rc-loss,rc-loss-airborne,wind2 |
| 83204 | 40.797 s | sinking -0.68 m/s at 18.4 m | arm-throttle-up,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,stick-slam |
| 85594 | 20.692 s | sinking -0.19 m/s at 15.8 m | imu-dead-at-arm,imu-rotated,mode-switch,old-pack,punch,rc-loss |
| 93575 | 32.052 s | sinking 0.07 m/s at 8.6 m | imu-rotated,mode-switch,punch,rc-loss,rc-loss-airborne |
| 93691 | 34.523 s | sinking -0.45 m/s at 12.4 m | bent-prop,clock-glitch,flip,imu-garbage,imu-rotated,imu-stuck,mode-switch,no-telemetry,rc-loss-airborne,stick-slam,wind0 |
| 93930 | 44.527 s | sinking -0.44 m/s at 15.7 m | imu-stuck,mode-switch,rc-loss,rc-loss-airborne,stick-script,stick-slam |
| 104297 | 29.923 s | sinking -0.05 m/s at 15.3 m | clock-wrap,imu-rotated,low-battery,mode-switch,rc-loss,stick-script,stick-slam,wind2 |
| 107992 | 35.926 s | sinking -1.49 m/s at 30.3 m | event-drop,flip,imu-rotated,inverted-hang,mode-switch,rc-loss,stick-slam |
| 108606 | 34.691 s | sinking -0.95 m/s at 49.7 m | inverted-hang,mode-switch,punch,rc-loss |
| 113163 | 21.533 s | sinking -0.29 m/s at 18.6 m | bent-prop,desync,inverted-hang,mode-switch,no-telemetry,rc-glitch,rc-loss |
| 113717 | 32.135 s | sinking -2.28 m/s at 57.8 m | bent-prop,drop,inverted-hang,moved-at-boot,old-pack,rc-loss,rc-loss-airborne |
| 115349 | 38.592 s | sinking -0.56 m/s at 34.6 m | arm-throttle-up,flip,imu-dead-at-arm,imu-garbage,imu-rotated,inverted-hang,mode-switch,no-telemetry,rc-loss |
| 122758 | 54.666 s | sinking -0.25 m/s at 18.3 m | flip,imu-rotated,no-telemetry,rc-loss,rc-loss-airborne,stick-script,wind0 |
| 126626 | 34.532 s | sinking -0.21 m/s at 10.6 m | imu-stuck,late-link,no-telemetry,old-pack,punch,rc-loss-airborne,stick-slam |
| 127755 | 23.498 s | sinking 0.04 m/s at 20.7 m | imu-rotated,inverted-hang,low-battery,mode-switch,rc-loss,stick-slam |
| 132310 | 27.270 s | sinking -0.76 m/s at 22.4 m | flip,inverted-hang,rc-loss,stick-script |
| 134837 | 32.589 s | sinking -2.06 m/s at 54.3 m | flip,imu-garbage,inverted-hang,no-telemetry,rc-loss |
| 136245 | 129.856 s | sinking -1.02 m/s at 13.5 m | bent-prop,imu-rotated,inverted-hang,late-link,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,wind4 |
| 142498 | 31.316 s | sinking -0.94 m/s at 26.5 m | clock-wrap,inverted-hang,mode-switch,rc-loss,stick-slam,wind0 |
| 142747 | 29.228 s | sinking -1.11 m/s at 26.1 m | imu-rotated,inverted-hang,mode-switch,old-pack,rc-loss,stick-slam,wind4 |
| 143065 | 46.321 s | sinking -0.22 m/s at 49.1 m | flip,impact,imu-rotated,inverted-hang,mode-switch,no-telemetry,old-pack,punch,rc-loss |
| 152392 | 24.601 s | sinking -1.90 m/s at 57.9 m | inverted-hang,mode-switch,no-telemetry,rc-loss,reboot-request,stick-script,wind0 |
| 152584 | 28.835 s | sinking -2.31 m/s at 61.8 m | inverted-hang,late-link,no-telemetry,rc-loss,stick-script,wind4 |
| 152826 | 21.892 s | sinking -1.12 m/s at 19.4 m | imu-dead-at-arm,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,stick-script,stick-slam,telemetry-loss,wind1 |
| 162499 | 42.495 s | sinking 0.03 m/s at 11.6 m | arm-throttle-up,bent-prop,clock-glitch,desync,flip,jitter,rc-loss-airborne,stick-slam,wind2 |
| 164789 | 34.090 s | sinking -1.53 m/s at 53.1 m | bent-prop,inverted-hang,mode-switch,punch,rc-loss,stick-slam |
| 166185 | 26.696 s | sinking -0.33 m/s at 13.9 m | bent-prop,no-telemetry,old-pack,rc-loss,rc-loss-airborne,telemetry-loss |
| 171911 | 28.769 s | sinking -0.36 m/s at 13.6 m | bent-prop,imu-dropout,mode-switch,moved-at-boot,punch,rc-loss,reboot-request,stick-script,wind3 |
| 172611 | 27.543 s | sinking -1.97 m/s at 56.1 m | arm-on-boot,imu-rotated,inverted-hang,mode-switch,rc-loss,stick-slam |
| 173333 | 23.528 s | sinking -0.51 m/s at 25.1 m | event-drop,flip,inverted-hang,mode-switch,no-telemetry,rc-loss,stick-slam,wind1 |
| 182248 | 42.127 s | sinking -0.14 m/s at 55.8 m | arm-on-boot,arm-throttle-up,flip,imu-rotated,inverted-hang,late-link,mode-switch,punch,rc-loss,wind1 |
| 183368 | 25.620 s | sinking -0.76 m/s at 19.7 m | bent-prop,drop,imu-stuck,rc-loss-airborne,wind1 |
| 187164 | 29.379 s | sinking -0.77 m/s at 22.5 m | bent-prop,imu-rotated,imu-stuck,mode-switch,rc-glitch,rc-loss,rc-loss-airborne |
| 192062 | 32.339 s | sinking -2.28 m/s at 58.4 m | clock-glitch,inverted-hang,mode-switch,no-telemetry,rc-loss,reboot-request,wind1 |
| 192938 | 25.984 s | sinking -0.63 m/s at 19.3 m | late-link,no-telemetry,rc-loss,rc-loss-airborne |
| 193931 | 34.362 s | sinking -1.48 m/s at 60.3 m | event-drop,inverted-hang,mode-switch,no-telemetry,rc-loss,stick-slam |

## `no_crash` (63 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 3323 | 17.582 s | touched down at 2.8 m/s | desync,imu-rotated,mode-switch,punch,wind3 |
| 4086 | 16.942 s | touched down at 9.9 m/s | clock-wrap,flip,impact,imu-rotated,kick,mode-switch,no-telemetry,old-pack,on-its-side,stick-script,wind4 |
| 11392 | 15.406 s | touched down at 5.1 m/s | drop,imu-rotated,kick,stick-script |
| 14034 | 25.154 s | touched down at 2.7 m/s | bent-prop,no-telemetry,punch,wind2 |
| 16987 | 20.008 s | touched down at 2.6 m/s | imu-rotated,low-battery,punch,wind1 |
| 22933 | 14.472 s | touched down at 2.5 m/s | bent-prop,desync,wind-gust,wind2 |
| 24692 | 18.659 s | touched down at 2.8 m/s | imu-garbage,imu-rotated,mode-switch,wind3 |
| 26055 | 17.476 s | touched down at 3.0 m/s | clock-glitch,clock-wrap,impact,moved-at-boot,on-its-side,punch,rc-glitch,wind4 |
| 33525 | 20.439 s | touched down at 3.1 m/s | arm-on-boot,arm-throttle-up,event-drop,punch,rc-glitch,wind2 |
| 36488 | 15.871 s | touched down at 6.4 m/s | desync,imu-dropout,imu-rotated,mode-switch,old-pack,stick-script,wind-gust,wind4 |
| 40382 | 12.090 s | touched down at 11.3 m/s | arm-throttle-up,imu-dropout,imu-garbage,imu-rotated,no-telemetry,rc-loss,stick-slam,wind2 |
| 41963 | 30.534 s | touched down at 2.7 m/s | flip,imu-dead-at-arm,mode-switch,stick-slam,wind4 |
| 43115 | 14.439 s | touched down at 8.8 m/s | bent-prop,impact,mode-switch,punch,stick-script,wind3 |
| 52665 | 25.607 s | touched down at 2.7 m/s | mode-switch,no-telemetry,stick-slam,wind4 |
| 56064 | 14.701 s | touched down at 2.6 m/s | bent-prop,imu-dropout,mode-switch,reboot-request,wind3 |
| 57354 | 17.201 s | touched down at 2.7 m/s | mode-switch,moved-at-boot,no-telemetry,punch,wind1 |
| 77845 | 21.237 s | touched down at 2.7 m/s | imu-garbage,mode-switch,moved-at-boot,no-telemetry,old-pack,on-its-side,rc-loss,wind0 |
| 78616 | 19.697 s | touched down at 3.7 m/s | arm-on-boot,fc-stall,impact,late-link,mode-switch,old-pack,punch,stick-script,stick-slam |
| 84958 | 13.763 s | touched down at 10.2 m/s | clock-wrap,mode-switch,stick-slam |
| 87275 | 27.058 s | touched down at 2.5 m/s | arm-on-boot,clock-glitch,late-link,punch,rc-loss,wind4 |
| 88770 | 60.898 s | touched down at 5.4 m/s | impact,imu-rotated,inverted-hang,mode-switch,moved-at-boot,no-telemetry,stick-script,wind1 |
| 93046 | 96.839 s | touched down at 8.7 m/s | arm-on-boot,desync,imu-rotated,inverted-hang,no-telemetry,stick-script,wind2 |
| 111114 | 16.962 s | touched down at 2.9 m/s | punch,wind4 |
| 111230 | 42.105 s | touched down at 7.1 m/s | event-drop,kick,reboot-request,stick-script,wind3 |
| 111709 | 20.345 s | touched down at 2.6 m/s | imu-dead-at-arm,punch |
| 112012 | 19.555 s | touched down at 2.8 m/s | imu-garbage,imu-rotated,mode-switch,moved-at-boot,no-telemetry,wind3 |
| 119000 | 20.423 s | touched down at 2.9 m/s | desync,imu-dead-at-arm,stick-script |
| 123370 | 19.774 s | touched down at 2.5 m/s | bent-prop,clock-wrap,mode-switch,no-telemetry,old-pack,punch,reboot-request,wind4 |
| 130640 | 19.281 s | touched down at 2.6 m/s | clock-wrap,mode-switch,no-telemetry |
| 131128 | 20.211 s | touched down at 3.4 m/s | desync,mode-switch,rc-loss-airborne,stick-script |
| 135660 | 21.009 s | touched down at 2.8 m/s | arm-on-boot,no-telemetry,stick-slam,wind3 |
| 135967 | 22.465 s | touched down at 2.9 m/s | desync,imu-stuck,no-telemetry,punch,stick-slam,telemetry-loss,wind4 |
| 137780 | 18.043 s | touched down at 2.5 m/s | jitter,stick-slam |
| 142380 | 11.935 s | touched down at 5.0 m/s | impact,inverted-hang,old-pack,stick-script,stick-slam,wind1 |
| 144044 | 18.108 s | touched down at 9.0 m/s | bent-prop,drop,event-drop,jitter,low-battery,mode-switch,no-telemetry,stick-slam,wind2 |
| 144561 | 17.394 s | touched down at 3.3 m/s | low-battery,mode-switch,moved-at-boot,old-pack,on-its-side,punch,reboot-request,wind4 |
| 159524 | 11.606 s | touched down at 4.7 m/s | bent-prop,kick,moved-at-boot,old-pack,stick-slam,wind1 |
| 160326 | 14.521 s | touched down at 11.3 m/s | clock-wrap,imu-dropout,imu-rotated,low-battery,mode-switch,punch,rc-glitch,stick-slam |
| 166323 | 14.085 s | touched down at 10.4 m/s | bent-prop,low-battery,mode-switch,stick-slam,wind3 |
| 166716 | 8.001 s | touched down at 10.5 m/s | arm-on-boot,imu-dropout,imu-stuck,low-battery,mode-switch,no-telemetry,punch,wind2 |
| 166977 | 20.169 s | touched down at 3.3 m/s | drop,flip,imu-dropout,imu-garbage,kick,mode-switch,old-pack,punch,stick-script,wind4 |
| 172128 | 28.459 s | touched down at 2.9 m/s | no-telemetry,stick-slam,wind3 |
| 175419 | 20.040 s | touched down at 3.2 m/s | clock-wrap,imu-garbage,low-battery,mode-switch,no-telemetry,rc-loss,wind3 |
| 181276 | 23.407 s | touched down at 2.6 m/s | bent-prop,mode-switch,no-telemetry,wind3 |
| 186058 | 18.674 s | touched down at 2.7 m/s | arm-throttle-up,bent-prop,impact,kick,late-link,no-telemetry,old-pack,stick-slam,telemetry-loss,wind0 |
| 188527 | 12.364 s | touched down at 3.3 m/s | clock-wrap,drop,impact,mode-switch,motor-degraded,stick-script,stick-slam |
| 199305 | 18.169 s | touched down at 4.4 m/s | bent-prop,impact,imu-rotated,inverted-hang,old-pack,stick-script,wind1 |
| 200739 | 21.427 s | touched down at 2.8 m/s | clock-wrap,imu-dead-at-arm,imu-rotated,moved-at-boot,punch,stick-slam |

## `crash_stops` (61 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 3337 | 53.390 s | armed, crashed at 156 deg for 4000 ms | event-drop,imu-rotated,inverted-hang,mode-switch,moved-at-boot,no-telemetry,on-its-side,punch,rc-loss,stick-slam,wind4 |
| 4388 | 45.221 s | armed, crashed at 90 deg for 4000 ms | inverted-hang,mode-switch,motor-dead,no-telemetry,rc-loss-airborne,stick-slam,wind1 |
| 5226 | 32.413 s | armed, crashed at 157 deg for 4000 ms | arm-on-boot,clock-wrap,flip,imu-rotated,inverted-hang,mode-switch,motor-dead,motor-degraded,no-telemetry,old-pack,punch,stick-script,wind-gust,wind3 |
| 14534 | 25.804 s | armed, crashed at 156 deg for 4000 ms | arm-on-boot,clock-glitch,imu-stuck,late-link,mode-switch,motor-dead,stick-script,wind1 |
| 16470 | 10.519 s | armed, crashed at 90 deg for 4000 ms | imu-misaligned,imu-rotated,low-battery,mode-switch,old-pack,punch,telemetry-loss,wind3 |
| 26843 | 12.679 s | armed, crashed at 90 deg for 4000 ms | desync,drop,imu-misaligned,imu-rotated,kick,mode-switch,motor-dead,punch,stick-script,wind4 |
| 29899 | 10.412 s | armed, crashed at 89 deg for 4000 ms | bent-prop,drop,event-drop,imu-misaligned,mode-switch,motor-dead,no-telemetry,rc-glitch |
| 36490 | 40.844 s | armed, crashed at 90 deg for 4000 ms | arm-throttle-up,fc-stall,mode-switch,motor-degraded,stick-script,wind3 |
| 42590 | 23.812 s | armed, crashed at 89 deg for 4000 ms | fc-stall,flip,mode-switch,motor-dead,rc-loss,stick-slam,wind4 |
| 42929 | 21.434 s | armed, crashed at 156 deg for 4000 ms | bent-prop,mode-switch,motor-dead,stick-script,stick-slam |
| 47612 | 42.564 s | armed, crashed at 156 deg for 4000 ms | clock-wrap,flip,imu-rotated,inverted-hang,low-battery,mode-switch,motor-dead |
| 58818 | 31.630 s | armed, crashed at 90 deg for 4000 ms | flip,mode-switch,motor-dead,no-telemetry,rc-loss,wind4 |
| 59078 | 11.189 s | armed, crashed at 90 deg for 4000 ms | arm-throttle-up,bent-prop,flip,imu-misaligned,inverted-hang,low-battery,mode-switch,no-telemetry,stick-slam |
| 59559 | 38.385 s | armed, crashed at 156 deg for 4000 ms | drop,imu-garbage,imu-rotated,mode-switch,motor-dead,stick-script,stick-slam,wind2 |
| 64370 | 98.940 s | armed, crashed at 157 deg for 4000 ms | arm-on-boot,inverted-hang,late-link,mode-switch,motor-dead,moved-at-boot,no-telemetry,punch,stick-script,stick-slam,wind4 |
| 70416 | 8.531 s | armed, crashed at 90 deg for 4000 ms | arm-on-boot,flip,imu-misaligned,mode-switch,old-pack |
| 72499 | 42.146 s | armed, crashed at 157 deg for 4000 ms | arm-on-boot,desync,inverted-hang,mode-switch,rc-loss |
| 76182 | 15.274 s | armed, crashed at 89 deg for 4000 ms | arm-throttle-up,bent-prop,imu-misaligned,inverted-hang,mode-switch,motor-dead,motor-degraded,no-telemetry,rc-loss-airborne,stick-slam,telemetry-loss,wind2 |
| 83872 | 18.740 s | armed, crashed at 156 deg for 4000 ms | bent-prop,drop,imu-rotated,motor-degraded,moved-at-boot,no-telemetry,old-pack,rc-glitch,stick-script,wind3 |
| 85077 | 41.715 s | armed, crashed at 90 deg for 4000 ms | flip,mode-switch,motor-dead,old-pack,stick-script,stick-slam,wind1 |
| 88013 | 13.336 s | armed, crashed at 90 deg for 4000 ms | bent-prop,impact,jitter,low-battery,mode-switch,motor-dead,stick-slam,wind4 |
| 93627 | 26.415 s | armed, crashed at 89 deg for 4000 ms | event-drop,imu-rotated,motor-degraded,no-telemetry,old-pack,stick-script,stick-slam,wind4 |
| 95038 | 90.654 s | armed, crashed at 156 deg for 4000 ms | flip,inverted-hang,motor-degraded,stick-script,stick-slam,telemetry-loss |
| 97141 | 12.493 s | armed, crashed at 90 deg for 4000 ms | arm-on-boot,flip,late-link,mode-switch,motor-dead,no-telemetry |
| 102958 | 33.115 s | armed, crashed at 90 deg for 4000 ms | arm-on-boot,imu-misaligned,kick,late-link,low-battery,mode-switch,motor-dead,moved-at-boot,no-telemetry,old-pack,stick-script,wind2 |
| 104904 | 10.416 s | armed, crashed at 90 deg for 4000 ms | bent-prop,clock-glitch,event-drop,imu-misaligned,low-battery,old-pack,punch,rc-loss-airborne,stick-slam |
| 111718 | 8.519 s | armed, crashed at 90 deg for 4000 ms | imu-misaligned,mode-switch,motor-degraded,old-pack,rc-glitch,rc-loss,wind1 |
| 115112 | 48.627 s | armed, crashed at 90 deg for 4000 ms | bent-prop,inverted-hang,jitter,motor-degraded,stick-script,wind4 |
| 115574 | 43.619 s | armed, crashed at 90 deg for 4000 ms | flip,imu-rotated,inverted-hang,mode-switch,motor-dead,rc-glitch |
| 116759 | 13.761 s | armed, crashed at 90 deg for 4000 ms | arm-on-boot,imu-misaligned,stick-slam,wind2 |
| 122030 | 10.817 s | armed, crashed at 90 deg for 4000 ms | arm-on-boot,flip,mode-switch,motors-swapped,rc-glitch,stick-script,stick-slam |
| 124407 | 18.224 s | armed, crashed at 90 deg for 4000 ms | arm-on-boot,inverted-hang,kick,mode-switch,moved-at-boot,old-pack,rc-glitch,stick-slam |
| 128326 | 32.157 s | armed, crashed at 156 deg for 4000 ms | inverted-hang,jitter,mode-switch,motor-dead,no-telemetry,stick-slam |
| 131901 | 21.757 s | armed, crashed at 156 deg for 4000 ms | imu-dropout,motor-dead,old-pack,stick-script,stick-slam,wind-gust |
| 138527 | 15.054 s | armed, crashed at 90 deg for 4000 ms | drop,imu-misaligned,imu-rotated,mode-switch,no-telemetry,old-pack,punch,rc-glitch,wind3 |
| 140293 | 9.836 s | armed, crashed at 90 deg for 4000 ms | clock-wrap,imu-misaligned,mode-switch,moved-at-boot,no-telemetry,old-pack,punch,rc-glitch |
| 141803 | 48.852 s | armed, crashed at 158 deg for 4000 ms | drop,flip,inverted-hang,motor-dead,no-telemetry,stick-slam,telemetry-loss |
| 145506 | 66.416 s | armed, crashed at 156 deg for 4000 ms | arm-on-boot,clock-wrap,imu-rotated,inverted-hang,jitter,late-link,mode-switch,motor-dead,no-telemetry,punch,reboot-request,stick-script |
| 148740 | 13.535 s | armed, crashed at 90 deg for 4000 ms | imu-misaligned,imu-rotated,mode-switch,no-telemetry,rc-loss-airborne,stick-slam |
| 153022 | 13.270 s | armed, crashed at 90 deg for 4000 ms | desync,drop,imu-misaligned,late-link,low-battery,old-pack,rc-glitch,stick-slam |
| 156420 | 68.376 s | armed, crashed at 157 deg for 4000 ms | bent-prop,inverted-hang,motor-dead,no-telemetry,rc-glitch,wind4 |
| 165886 | 8.152 s | armed, crashed at 90 deg for 4000 ms | clock-wrap,imu-misaligned,low-battery,old-pack,wind1 |
| 169463 | 8.069 s | armed, crashed at 90 deg for 4000 ms | imu-misaligned,low-battery,mode-switch,old-pack,rc-glitch,rc-loss-airborne,wind1 |
| 170135 | 19.848 s | armed, crashed at 156 deg for 4000 ms | desync,drop,fc-stall,imu-rotated,mode-switch,motor-dead,no-telemetry,old-pack,stick-script,stick-slam,wind0 |
| 173153 | 13.557 s | armed, crashed at 90 deg for 4000 ms | bent-prop,mode-switch,rc-glitch,stick-slam |
| 174094 | 13.064 s | armed, crashed at 90 deg for 4000 ms | clock-glitch,drop,imu-misaligned,imu-stuck,old-pack,rc-loss-airborne,stick-slam |
| 175920 | 58.830 s | armed, crashed at 157 deg for 4000 ms | arm-on-boot,flip,inverted-hang,motor-dead,no-telemetry,rc-glitch,rc-loss-airborne,reboot-request,stick-script,wind4 |
| 183622 | 18.125 s | armed, crashed at 89 deg for 4000 ms | clock-wrap,desync,drop,event-drop,imu-misaligned,imu-rotated,mode-switch,old-pack,punch,stick-slam,wind1 |
| 184869 | 27.220 s | armed, crashed at 156 deg for 4000 ms | late-link,mode-switch,motor-dead,punch,stick-script |
| 196216 | 14.686 s | armed, crashed at 156 deg for 4000 ms | arm-on-boot,flip,motor-dead,stick-script,telemetry-loss,wind2 |

## `failsafe_holds` (49 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 8825 | 38.584 s | vertical acceleration -6.4 m/s2 in the hold | imu-dead-at-arm,inverted-hang,mode-switch,rc-loss,stick-slam |
| 13490 | 36.159 s | vertical acceleration -4.9 m/s2 in the hold | inverted-hang,moved-at-boot,no-telemetry,rc-loss,rc-loss-airborne,wind1 |
| 23026 | 31.772 s | vertical acceleration -6.3 m/s2 in the hold | bent-prop,clock-wrap,imu-garbage,imu-rotated,inverted-hang,mode-switch,rc-loss,wind0 |
| 26266 | 33.289 s | vertical acceleration -5.7 m/s2 in the hold | desync,imu-garbage,inverted-hang,rc-loss,stick-script |
| 26308 | 32.201 s | vertical acceleration -4.9 m/s2 in the hold | clock-wrap,event-drop,flip,imu-dead-at-arm,imu-rotated,inverted-hang,mode-switch,moved-at-boot,punch,rc-loss,wind1 |
| 34739 | 36.258 s | vertical acceleration -6.8 m/s2 in the hold | clock-wrap,desync,inverted-hang,mode-switch,rc-loss,stick-slam,telemetry-loss,wind2 |
| 35760 | 28.819 s | vertical acceleration -6.3 m/s2 in the hold | desync,inverted-hang,moved-at-boot,on-its-side,rc-loss,stick-slam,wind1 |
| 44020 | 39.899 s | vertical acceleration -6.4 m/s2 in the hold | arm-throttle-up,imu-rotated,inverted-hang,jitter,mode-switch,rc-glitch,rc-loss,rc-loss-airborne,wind4 |
| 61143 | 39.311 s | vertical acceleration -5.2 m/s2 in the hold | arm-on-boot,flip,imu-rotated,inverted-hang,late-link,no-telemetry,rc-loss |
| 63853 | 30.205 s | vertical acceleration -3.9 m/s2 in the hold | arm-on-boot,arm-throttle-up,bent-prop,clock-wrap,imu-garbage,inverted-hang,mode-switch,motor-degraded,rc-loss,rc-loss-airborne,wind1 |
| 69402 | 45.076 s | vertical acceleration -6.5 m/s2 in the hold | flip,imu-garbage,imu-rotated,inverted-hang,mode-switch,punch,rc-loss,wind1 |
| 69881 | 30.476 s | vertical acceleration -5.6 m/s2 in the hold | imu-dropout,inverted-hang,mode-switch,no-telemetry,rc-loss |
| 72497 | 36.859 s | vertical acceleration -3.6 m/s2 in the hold | clock-wrap,flip,inverted-hang,low-battery,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,stick-slam |
| 74133 | 43.213 s | vertical acceleration -4.1 m/s2 in the hold | clock-glitch,inverted-hang,kick,mode-switch,rc-loss,stick-slam |
| 74261 | 30.198 s | vertical acceleration -6.4 m/s2 in the hold | clock-glitch,inverted-hang,mode-switch,rc-loss,stick-slam,wind2 |
| 84653 | 41.188 s | vertical acceleration -5.6 m/s2 in the hold | arm-on-boot,bent-prop,clock-wrap,drop,imu-stuck,inverted-hang,late-link,mode-switch,rc-loss,stick-slam,wind4 |
| 85744 | 9.922 s | vertical acceleration 5.7 m/s2 in the hold | clock-wrap,low-battery,mode-switch,no-telemetry,rc-loss,stick-slam,wind3 |
| 88924 | 38.011 s | vertical acceleration -3.5 m/s2 in the hold | flip,inverted-hang,rc-loss,wind1 |
| 101034 | 35.438 s | vertical acceleration -5.6 m/s2 in the hold | flip,inverted-hang,mode-switch,motor-degraded,punch,rc-loss,rc-loss-airborne,reboot-request,stick-slam |
| 103291 | 35.727 s | vertical acceleration -6.5 m/s2 in the hold | drop,inverted-hang,mode-switch,punch,rc-loss,stick-slam,wind3 |
| 129779 | 67.305 s | vertical acceleration -4.9 m/s2 in the hold | clock-wrap,imu-rotated,inverted-hang,rc-loss,wind-gust |
| 135250 | 10.620 s | vertical acceleration -12.0 m/s2 in the hold | arm-throttle-up,drop,mode-switch,moved-at-boot,no-telemetry,rc-loss,stick-slam,wind2 |
| 136245 | 29.152 s | vertical acceleration -6.8 m/s2 in the hold | bent-prop,imu-rotated,inverted-hang,late-link,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,wind4 |
| 140457 | 38.994 s | vertical acceleration -3.5 m/s2 in the hold | flip,imu-garbage,inverted-hang,mode-switch,no-telemetry,rc-loss,stick-slam,wind-gust |
| 142160 | 53.955 s | vertical acceleration -5.9 m/s2 in the hold | arm-throttle-up,desync,flip,imu-rotated,inverted-hang,rc-glitch,rc-loss,stick-slam |
| 143891 | 29.011 s | vertical acceleration -4.1 m/s2 in the hold | flip,inverted-hang,mode-switch,no-telemetry,old-pack,rc-loss,stick-slam |
| 145724 | 51.327 s | vertical acceleration -4.5 m/s2 in the hold | flip,inverted-hang,no-telemetry,rc-loss,stick-script,wind1 |
| 154961 | 33.051 s | vertical acceleration -6.2 m/s2 in the hold | arm-on-boot,inverted-hang,late-link,rc-loss,stick-slam |
| 161593 | 34.641 s | vertical acceleration -6.1 m/s2 in the hold | arm-on-boot,clock-wrap,flip,inverted-hang,mode-switch,no-telemetry,rc-loss,stick-script,wind2 |
| 165009 | 43.660 s | vertical acceleration -4.0 m/s2 in the hold | clock-glitch,imu-dead-at-arm,imu-rotated,inverted-hang,low-battery,mode-switch,no-telemetry,rc-loss,stick-script |
| 166029 | 29.457 s | vertical acceleration -5.1 m/s2 in the hold | arm-on-boot,clock-wrap,event-drop,imu-rotated,inverted-hang,no-telemetry,rc-loss,wind3 |
| 170274 | 37.614 s | vertical acceleration -6.6 m/s2 in the hold | imu-stuck,inverted-hang,mode-switch,punch,rc-loss,wind1 |
| 172984 | 33.456 s | vertical acceleration -5.7 m/s2 in the hold | clock-wrap,fc-stall,inverted-hang,moved-at-boot,no-telemetry,rc-loss,wind-gust |
| 173133 | 39.335 s | vertical acceleration -5.6 m/s2 in the hold | flip,impact,inverted-hang,rc-loss,rc-loss-airborne,stick-script |
| 173190 | 48.531 s | vertical acceleration -3.7 m/s2 in the hold | flip,inverted-hang,mode-switch,rc-glitch,rc-loss,stick-script |
| 182874 | 46.150 s | vertical acceleration -5.3 m/s2 in the hold | arm-throttle-up,flip,imu-dropout,imu-garbage,inverted-hang,mode-switch,no-telemetry,rc-loss,stick-slam,wind3 |
| 185410 | 37.407 s | vertical acceleration -5.8 m/s2 in the hold | inverted-hang,mode-switch,rc-loss,reboot-request |
| 188975 | 36.213 s | vertical acceleration -6.1 m/s2 in the hold | inverted-hang,mode-switch,no-telemetry,punch,rc-loss |
| 195138 | 32.093 s | vertical acceleration -5.5 m/s2 in the hold | drop,inverted-hang,mode-switch,no-telemetry,rc-glitch,rc-loss,stick-slam |

## `arm_level` (47 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 8233 | 4.203 s | true tilt 158 deg | arm-on-boot,drop,flip,imu-misaligned,mode-switch,no-telemetry |
| 8359 | 6.261 s | true tilt 131 deg | imu-dropout,imu-misaligned,imu-rotated,inverted-hang,jitter,late-link,mode-switch,no-telemetry,punch |
| 9308 | 14.502 s | true tilt 83 deg | desync,imu-misaligned,imu-rotated,mode-switch,moved-at-boot,rc-loss-airborne,stick-script,wind3 |
| 14996 | 38.504 s | true tilt 86 deg | arm-throttle-up,imu-misaligned,inverted-hang,stick-script,stick-slam,wind3 |
| 18478 | 5.101 s | true tilt 155 deg | imu-dropout,imu-misaligned,inverted-hang,mode-switch,no-telemetry,old-pack,punch,rc-loss-airborne,wind-gust |
| 23441 | 22.111 s | true tilt 70 deg | arm-on-boot,imu-dead-at-arm,imu-dropout,imu-rotated,moved-at-boot,no-telemetry,punch,stick-slam,wind3 |
| 31566 | 4.403 s | true tilt 96 deg | clock-wrap,imu-misaligned,imu-rotated,mode-switch,stick-slam |
| 32009 | 4.300 s | true tilt 131 deg | clock-wrap,event-drop,flip,impact,imu-misaligned,inverted-hang,old-pack,stick-slam,wind2 |
| 36284 | 10.301 s | true tilt 158 deg | flip,imu-misaligned,imu-rotated,mode-switch,old-pack,rc-glitch,rc-loss,stick-script |
| 37580 | 23.818 s | true tilt 132 deg | arm-on-boot,impact,imu-garbage,imu-stuck,late-link,mode-switch,moved-at-boot,no-telemetry,punch,stick-slam |
| 44228 | 6.302 s | true tilt 87 deg | imu-misaligned,imu-rotated,inverted-hang,mode-switch,rc-loss-airborne,stick-script,wind3 |
| 44540 | 4.804 s | true tilt 158 deg | imu-misaligned,inverted-hang,old-pack,rc-loss,stick-slam |
| 45133 | 11.201 s | true tilt 102 deg | imu-dead-at-arm,imu-stuck,mode-switch,stick-slam |
| 53059 | 5.102 s | true tilt 67 deg | flip,imu-stuck,inverted-hang,low-battery,mode-switch,motors-swapped,stick-slam |
| 56571 | 5.102 s | true tilt 90 deg | drop,imu-misaligned,inverted-hang,mode-switch,no-telemetry,stick-script |
| 71870 | 5.619 s | true tilt 63 deg | arm-throttle-up,imu-garbage,motors-swapped |
| 78547 | 7.203 s | true tilt 75 deg | imu-misaligned,low-battery,mode-switch,rc-loss-airborne,wind3 |
| 90504 | 10.501 s | true tilt 35 deg | arm-throttle-up,desync,impact,imu-dead-at-arm,imu-misaligned,kick,low-battery,old-pack |
| 92395 | 11.404 s | true tilt 47 deg | fc-stall,imu-garbage,imu-misaligned,mode-switch,no-telemetry,old-pack,stick-script,stick-slam |
| 94359 | 12.602 s | true tilt 89 deg | imu-garbage,imu-misaligned,imu-rotated,old-pack,wind-gust |
| 95930 | 4.301 s | true tilt 106 deg | clock-wrap,imu-dropout,imu-misaligned,mode-switch,motor-dead,no-telemetry,reboot-request,wind3 |
| 104022 | 4.402 s | true tilt 61 deg | drop,imu-dead-at-arm,inverted-hang,jitter,low-battery,mode-switch,motors-swapped |
| 105936 | 8.101 s | true tilt 38 deg | flip,imu-misaligned,imu-rotated,mode-switch,moved-at-boot,old-pack,rc-glitch,stick-script,stick-slam,wind-gust,wind2 |
| 108463 | 14.712 s | true tilt 38 deg | bent-prop,clock-wrap,imu-rotated,imu-stuck,low-battery,old-pack,wind3 |
| 120588 | 4.609 s | true tilt 43 deg | imu-misaligned,mode-switch,stick-slam |
| 128351 | 3.501 s | true tilt 33 deg | flip,imu-misaligned,imu-rotated,mode-switch,punch,stick-script,stick-slam,wind3 |
| 134810 | 4.206 s | true tilt 38 deg | imu-misaligned,no-telemetry,stick-slam,wind-gust,wind3 |
| 137490 | 7.303 s | true tilt 39 deg | clock-glitch,clock-wrap,event-drop,flip,mode-switch,motors-swapped,punch |
| 137760 | 14.403 s | true tilt 51 deg | drop,flip,imu-dropout,imu-misaligned,imu-rotated,mode-switch,old-pack,rc-loss-airborne,stick-slam |
| 143908 | 5.901 s | true tilt 38 deg | arm-throttle-up,imu-misaligned,low-battery,mode-switch,no-telemetry,stick-slam |
| 148985 | 4.705 s | true tilt 90 deg | arm-on-boot,imu-misaligned,mode-switch,on-its-side,punch,rc-loss-airborne,wind2 |
| 153888 | 10.801 s | true tilt 93 deg | impact,imu-garbage,imu-misaligned,moved-at-boot,punch,stick-slam,wind3 |
| 153949 | 16.103 s | true tilt 35 deg | arm-on-boot,imu-dropout,inverted-hang,kick,mode-switch,moved-at-boot,old-pack,punch,wind3 |
| 173070 | 4.704 s | true tilt 41 deg | flip,imu-misaligned,mode-switch,punch,stick-script,wind4 |
| 177478 | 15.504 s | true tilt 120 deg | imu-dropout,imu-stuck,motors-swapped,no-telemetry,rc-loss-airborne,wind4 |
| 178233 | 27.602 s | true tilt 40 deg | drop,impact,imu-misaligned,imu-rotated,inverted-hang,no-telemetry,stick-script,stick-slam,telemetry-loss |
| 183622 | 5.101 s | true tilt 78 deg | clock-wrap,desync,drop,event-drop,imu-misaligned,imu-rotated,mode-switch,old-pack,punch,stick-slam,wind1 |
| 191630 | 7.906 s | true tilt 46 deg | clock-glitch,impact,imu-misaligned,jitter,low-battery,mode-switch,old-pack,rc-loss-airborne,stick-slam,wind3 |
| 198554 | 29.804 s | true tilt 45 deg | arm-on-boot,clock-wrap,event-drop,imu-stuck,inverted-hang,mode-switch,no-telemetry,old-pack,punch,rc-loss,stick-slam,wind3 |
| 198607 | 14.204 s | true tilt 44 deg | desync,imu-dropout,imu-garbage,mode-switch,old-pack,stick-slam |
| 198944 | 20.715 s | true tilt 57 deg | bent-prop,flip,imu-stuck,jitter,no-telemetry,rc-loss |

## `failsafe_disarms_on_touchdown` (42 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 2541 | 16.960 s | armed on the ground 2000 ms after touchdown | imu-rotated,jitter,no-telemetry,rc-loss,reboot-request,stick-slam,wind3 |
| 13490 | 41.122 s | armed on the ground 2000 ms after touchdown | inverted-hang,moved-at-boot,no-telemetry,rc-loss,rc-loss-airborne,wind1 |
| 19172 | 32.032 s | armed on the ground 2000 ms after touchdown | flip,rc-loss-airborne,stick-slam |
| 21730 | 38.811 s | armed on the ground 2000 ms after touchdown | bent-prop,clock-glitch,fc-stall,impact,imu-rotated,no-telemetry,punch,rc-loss-airborne |
| 24578 | 14.367 s | armed on the ground 2000 ms after touchdown | imu-rotated,low-battery,mode-switch,moved-at-boot,old-pack,rc-loss,stick-script,telemetry-loss,wind-gust,wind2 |
| 25432 | 29.622 s | armed on the ground 2000 ms after touchdown | clock-wrap,kick,mode-switch,moved-at-boot,rc-loss-airborne,reboot-request,stick-slam,wind-gust,wind1 |
| 28425 | 27.485 s | armed on the ground 2000 ms after touchdown | bent-prop,drop,imu-rotated,motor-degraded,punch,rc-loss-airborne |
| 32991 | 16.531 s | armed on the ground 2000 ms after touchdown | clock-wrap,flip,mode-switch,punch,rc-loss |
| 34008 | 24.933 s | armed on the ground 2000 ms after touchdown | arm-throttle-up,bent-prop,impact,motor-dead,no-telemetry,punch,rc-loss,rc-loss-airborne,stick-slam,wind2 |
| 38599 | 22.886 s | armed on the ground 2000 ms after touchdown | arm-throttle-up,flip,mode-switch,no-telemetry,rc-loss,stick-script,stick-slam |
| 47598 | 44.150 s | armed on the ground 2000 ms after touchdown | flip,mode-switch,rc-loss-airborne,stick-script,wind0 |
| 60955 | 20.454 s | armed on the ground 2000 ms after touchdown | drop,imu-rotated,no-telemetry,rc-loss,stick-slam,wind-gust,wind4 |
| 62066 | 10.515 s | armed on the ground 2000 ms after touchdown | drop,kick,mode-switch,rc-loss,reboot-request,stick-slam,wind2 |
| 62098 | 26.515 s | armed on the ground 2000 ms after touchdown | imu-rotated,mode-switch,punch,rc-loss,rc-loss-airborne,stick-slam,telemetry-loss,wind2 |
| 70195 | 23.687 s | armed on the ground 2000 ms after touchdown | arm-throttle-up,drop,imu-dropout,rc-loss,stick-script,stick-slam |
| 73147 | 64.712 s | armed on the ground 2000 ms after touchdown | arm-on-boot,desync,drop,flip,imu-dead-at-arm,imu-rotated,inverted-hang,low-battery,motor-degraded,old-pack,rc-loss-airborne,stick-slam,wind3 |
| 74133 | 49.688 s | armed on the ground 2000 ms after touchdown | clock-glitch,inverted-hang,kick,mode-switch,rc-loss,stick-slam |
| 80328 | 23.746 s | armed on the ground 2000 ms after touchdown | imu-dropout,mode-switch,punch,rc-glitch,rc-loss,rc-loss-airborne,stick-slam,wind2 |
| 86105 | 10.893 s | armed on the ground 2000 ms after touchdown | imu-stuck,low-battery,mode-switch,no-telemetry,old-pack,rc-loss,stick-slam,telemetry-loss |
| 90270 | 58.015 s | armed on the ground 2000 ms after touchdown | desync,flip,inverted-hang,no-telemetry,old-pack,rc-loss-airborne,wind1 |
| 97981 | 48.437 s | armed on the ground 2000 ms after touchdown | flip,rc-loss,rc-loss-airborne,stick-script,stick-slam,wind4 |
| 107568 | 67.566 s | armed on the ground 2000 ms after touchdown | arm-throttle-up,drop,event-drop,flip,inverted-hang,mode-switch,motor-dead,no-telemetry,old-pack,rc-loss-airborne,stick-slam,wind1 |
| 113861 | 32.917 s | armed on the ground 2000 ms after touchdown | desync,mode-switch,rc-glitch,rc-loss-airborne,stick-slam |
| 133564 | 42.197 s | armed on the ground 2000 ms after touchdown | arm-on-boot,bent-prop,desync,mode-switch,rc-loss,rc-loss-airborne,stick-script,stick-slam,wind3 |
| 140568 | 25.897 s | armed on the ground 2000 ms after touchdown | bent-prop,flip,mode-switch,rc-loss,stick-slam,wind2 |
| 141101 | 15.933 s | armed on the ground 2000 ms after touchdown | desync,drop,imu-garbage,rc-loss,stick-slam |
| 144170 | 32.042 s | armed on the ground 2000 ms after touchdown | arm-throttle-up,clock-glitch,desync,imu-rotated,low-battery,mode-switch,no-telemetry,rc-loss-airborne,stick-slam,wind4 |
| 144393 | 27.835 s | armed on the ground 2000 ms after touchdown | clock-wrap,flip,mode-switch,rc-loss,stick-slam |
| 155057 | 23.180 s | armed on the ground 2000 ms after touchdown | flip,imu-stuck,kick,rc-loss,stick-slam |
| 162094 | 25.668 s | armed on the ground 2000 ms after touchdown | fc-stall,punch,rc-loss-airborne,wind-gust,wind4 |
| 166710 | 46.887 s | armed on the ground 2000 ms after touchdown | drop,flip,impact,mode-switch,no-telemetry,rc-loss-airborne,stick-script,stick-slam,wind2 |
| 170587 | 36.761 s | armed on the ground 2000 ms after touchdown | impact,low-battery,on-its-side,punch,rc-loss-airborne,stick-slam,wind-gust |
| 176852 | 9.787 s | armed on the ground 2000 ms after touchdown | clock-glitch,mode-switch,no-telemetry,punch,rc-loss,stick-slam,telemetry-loss,wind3 |
| 179132 | 39.712 s | armed on the ground 2000 ms after touchdown | arm-on-boot,mode-switch,punch,rc-loss-airborne,stick-slam |
| 182630 | 20.334 s | armed on the ground 2000 ms after touchdown | imu-garbage,imu-rotated,jitter,punch,rc-loss,rc-loss-airborne,stick-slam,wind3 |
| 183934 | 24.635 s | armed on the ground 2000 ms after touchdown | clock-glitch,low-battery,mode-switch,moved-at-boot,punch,rc-loss,stick-slam |
| 185132 | 31.487 s | armed on the ground 2000 ms after touchdown | arm-on-boot,desync,flip,kick,late-link,low-battery,mode-switch,no-telemetry,rc-loss,stick-slam,wind3 |
| 194657 | 66.397 s | armed on the ground 2000 ms after touchdown | drop,imu-garbage,inverted-hang,mode-switch,rc-glitch,rc-loss-airborne,stick-slam,wind-gust,wind4 |
| 195138 | 39.130 s | armed on the ground 2000 ms after touchdown | drop,inverted-hang,mode-switch,no-telemetry,rc-glitch,rc-loss,stick-slam |
| 196351 | 37.719 s | armed on the ground 2000 ms after touchdown | arm-on-boot,mode-switch,punch,rc-loss-airborne,reboot-request,stick-slam,wind3 |

## `disarm_justified` (41 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 6875 | 17.198 s | Some(RunawayTakeoff) at 0.1 m, tilt 19 deg, IMU fresh 0 ms ago | imu-dropout,mode-switch,no-telemetry,rc-loss,stick-slam,wind3 |
| 10031 | 13.535 s | Some(RunawayTakeoff) at 0.0 m, tilt 111 deg, IMU fresh 0 ms ago | arm-on-boot,clock-wrap,imu-dead-at-arm,imu-dropout,imu-rotated,late-link,mode-switch,motor-degraded,old-pack,stick-slam,wind-gust,wind1 |
| 11469 | 20.756 s | Some(RunawayTakeoff) at 0.1 m, tilt 129 deg, IMU fresh 0 ms ago | imu-dropout,imu-stuck,rc-loss-airborne,reboot-request,stick-slam |
| 18639 | 19.989 s | Some(RunawayTakeoff) at 0.0 m, tilt 50 deg, IMU fresh 0 ms ago | flip,imu-dropout,imu-rotated,moved-at-boot,no-telemetry,old-pack,rc-loss,stick-slam,wind3 |
| 19762 | 11.911 s | Some(RunawayTakeoff) at 0.1 m, tilt 135 deg, IMU fresh 0 ms ago | arm-on-boot,clock-wrap,fc-stall,impact,imu-stuck,mode-switch,moved-at-boot,rc-loss-airborne,stick-slam,wind2 |
| 21359 | 37.636 s | Some(RunawayTakeoff) at 0.2 m, tilt 65 deg, IMU fresh 0 ms ago | bent-prop,clock-wrap,flip,imu-rotated,imu-stuck,mode-switch,no-telemetry,punch,stick-script,wind1 |
| 28692 | 9.791 s | Some(ImuFailure) at 0.0 m, tilt 115 deg, IMU fresh 0 ms ago | arm-throttle-up,clock-wrap,imu-stuck,kick,moved-at-boot,no-telemetry,stick-slam,wind2 |
| 31828 | 11.180 s | Some(RunawayTakeoff) at 0.1 m, tilt 125 deg, IMU fresh 0 ms ago | arm-throttle-up,clock-wrap,fc-stall,flip,impact,imu-stuck,mode-switch,old-pack,rc-loss-airborne,wind2 |
| 33001 | 8.363 s | Some(RunawayTakeoff) at 0.1 m, tilt 154 deg, IMU fresh 0 ms ago | clock-wrap,imu-dropout,imu-rotated,stick-slam |
| 33962 | 17.925 s | Some(RunawayTakeoff) at 0.0 m, tilt 135 deg, IMU fresh 0 ms ago | clock-wrap,desync,late-link,mode-switch,punch,stick-slam,wind2 |
| 47563 | 7.701 s | Some(RunawayTakeoff) at 0.1 m, tilt 20 deg, IMU fresh 0 ms ago | flip,imu-garbage,imu-rotated,imu-stuck,kick,low-battery,no-telemetry,old-pack,stick-script,wind3 |
| 65521 | 13.622 s | Some(RunawayTakeoff) at 0.1 m, tilt 92 deg, IMU fresh 0 ms ago | clock-glitch,imu-dropout,mode-switch,no-telemetry,stick-slam |
| 71256 | 9.236 s | Some(RunawayTakeoff) at 0.1 m, tilt 131 deg, IMU fresh 0 ms ago | bent-prop,imu-stuck,stick-slam |
| 78912 | 20.375 s | Some(RunawayTakeoff) at 0.2 m, tilt 27 deg, IMU fresh 0 ms ago | fc-stall,imu-dropout,late-link,mode-switch,stick-slam,wind2 |
| 87358 | 11.219 s | Some(RunawayTakeoff) at 0.0 m, tilt 90 deg, IMU fresh 0 ms ago | fc-stall,imu-stuck,mode-switch,no-telemetry,stick-slam |
| 87407 | 11.944 s | Some(RunawayTakeoff) at 0.2 m, tilt 23 deg, IMU fresh 0 ms ago | clock-wrap,fc-stall,imu-dropout,imu-rotated,imu-stuck,mode-switch,no-telemetry,old-pack,stick-slam,wind0 |
| 102309 | 7.507 s | Some(RunawayTakeoff) at 0.1 m, tilt 12 deg, IMU fresh 0 ms ago | drop,imu-rotated,imu-stuck,inverted-hang,jitter,low-battery,old-pack,stick-script,stick-slam,wind2 |
| 106861 | 11.629 s | Some(RunawayTakeoff) at 0.1 m, tilt 36 deg, IMU fresh 0 ms ago | imu-stuck,jitter,low-battery,mode-switch,punch,stick-slam |
| 109288 | 21.524 s | Some(RunawayTakeoff) at 0.2 m, tilt 79 deg, IMU fresh 0 ms ago | flip,imu-garbage,imu-stuck,inverted-hang,jitter,mode-switch,no-telemetry,wind3 |
| 113398 | 6.881 s | Some(RunawayTakeoff) at 0.1 m, tilt 103 deg, IMU fresh 0 ms ago | flip,imu-dropout,mode-switch,punch,rc-loss-airborne,stick-slam,wind-gust |
| 114942 | 15.313 s | Some(RunawayTakeoff) at 0.1 m, tilt 110 deg, IMU fresh 0 ms ago | drop,imu-stuck,mode-switch,no-telemetry,wind2 |
| 115601 | 17.421 s | Some(RunawayTakeoff) at 0.0 m, tilt 82 deg, IMU fresh 0 ms ago | arm-on-boot,bent-prop,impact,imu-rotated,mode-switch,rc-loss,stick-slam |
| 122403 | 11.237 s | Some(RunawayTakeoff) at 0.0 m, tilt 42 deg, IMU fresh 0 ms ago | bent-prop,imu-rotated,imu-stuck,no-telemetry,stick-slam |
| 123274 | 15.859 s | Some(RunawayTakeoff) at 0.2 m, tilt 180 deg, IMU fresh 0 ms ago | clock-glitch,flip,imu-stuck,low-battery,mode-switch,reboot-request,stick-slam,wind0 |
| 130365 | 8.382 s | Some(RunawayTakeoff) at 0.2 m, tilt 88 deg, IMU fresh 0 ms ago | imu-dropout,imu-rotated,mode-switch,motor-degraded,punch,rc-loss,stick-slam |
| 137715 | 18.577 s | Some(RunawayTakeoff) at 0.1 m, tilt 12 deg, IMU fresh 0 ms ago | clock-wrap,imu-rotated,imu-stuck,mode-switch,stick-slam,wind2 |
| 140605 | 6.464 s | Some(RunawayTakeoff) at 0.1 m, tilt 33 deg, IMU fresh 0 ms ago | arm-on-boot,flip,imu-stuck,low-battery,no-telemetry,old-pack,stick-slam,wind2 |
| 158849 | 8.101 s | Some(RunawayTakeoff) at 0.2 m, tilt 170 deg, IMU fresh 0 ms ago | drop,imu-rotated,imu-stuck,mode-switch,old-pack,wind2 |
| 171896 | 10.102 s | Some(RunawayTakeoff) at 0.1 m, tilt 14 deg, IMU fresh 0 ms ago | clock-glitch,imu-stuck,no-telemetry,stick-slam,wind-gust,wind2 |
| 171940 | 12.393 s | Some(RunawayTakeoff) at 0.0 m, tilt 134 deg, IMU fresh 0 ms ago | bent-prop,imu-dropout,imu-rotated,rc-glitch,stick-slam,wind-gust |
| 178779 | 16.793 s | Some(RunawayTakeoff) at 0.2 m, tilt 135 deg, IMU fresh 0 ms ago | desync,imu-rotated,imu-stuck,mode-switch,no-telemetry,stick-script,stick-slam |
| 188981 | 13.726 s | Some(RunawayTakeoff) at 0.2 m, tilt 83 deg, IMU fresh 0 ms ago | clock-wrap,flip,imu-stuck,low-battery,mode-switch,motor-degraded,no-telemetry,stick-slam,telemetry-loss |
| 191372 | 9.336 s | Some(RunawayTakeoff) at 0.0 m, tilt 176 deg, IMU fresh 0 ms ago | bent-prop,clock-wrap,imu-stuck,mode-switch,old-pack,stick-slam,wind1 |
| 197192 | 9.732 s | Some(RunawayTakeoff) at 0.1 m, tilt 145 deg, IMU fresh 0 ms ago | bent-prop,clock-wrap,imu-stuck,mode-switch,old-pack,rc-loss-airborne,stick-slam |
| 198105 | 12.879 s | Some(RunawayTakeoff) at 0.1 m, tilt 94 deg, IMU fresh 0 ms ago | arm-on-boot,bent-prop,impact,late-link,no-telemetry,on-its-side,reboot-request,stick-slam,wind3 |
| 201706 | 19.009 s | Some(RunawayTakeoff) at 0.1 m, tilt 88 deg, IMU fresh 0 ms ago | arm-on-boot,arm-throttle-up,flip,imu-stuck,late-link,mode-switch,reboot-request,stick-slam,wind3 |

## `no_flyaway` (25 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 22334 | 106.499 s | alt 12.7 m, dist 120.0 m | flip,inverted-hang,jitter,kick,rc-glitch,rc-loss-airborne,stick-slam,wind4 |
| 23359 | 52.355 s | alt 60.0 m, dist 7.6 m | inverted-hang,mode-switch,no-telemetry,punch,wind1 |
| 32284 | 62.106 s | alt 60.0 m, dist 7.4 m | event-drop,inverted-hang,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,stick-script,wind2 |
| 50097 | 72.245 s | alt 60.0 m, dist 48.9 m | imu-rotated,inverted-hang,mode-switch,rc-loss,stick-script,telemetry-loss,wind4 |
| 61440 | 22.973 s | alt 60.0 m, dist 4.3 m | flip,inverted-hang,jitter,no-telemetry,rc-loss,stick-slam |
| 71703 | 105.313 s | alt 60.0 m, dist 6.8 m | fc-stall,inverted-hang,stick-script |
| 72226 | 96.099 s | alt 3.4 m, dist 120.0 m | flip,inverted-hang,jitter,mode-switch,no-telemetry,rc-loss-airborne,stick-script,wind4 |
| 98364 | 106.154 s | alt 4.4 m, dist 120.0 m | flip,inverted-hang,jitter,no-telemetry,rc-glitch,rc-loss,rc-loss-airborne,wind4 |
| 98753 | 30.865 s | alt 60.0 m, dist 2.0 m | imu-rotated,inverted-hang,mode-switch,rc-loss,stick-slam |
| 100128 | 85.966 s | alt 60.0 m, dist 23.3 m | flip,inverted-hang,stick-script,stick-slam,wind2 |
| 109130 | 88.331 s | alt 60.0 m, dist 23.5 m | fc-stall,inverted-hang,mode-switch,stick-slam,wind3 |
| 115709 | 77.191 s | alt 11.6 m, dist 120.0 m | flip,imu-dead-at-arm,imu-rotated,inverted-hang,mode-switch,no-telemetry,punch,rc-loss-airborne,stick-slam,wind4 |
| 120382 | 122.066 s | alt 7.7 m, dist 120.0 m | clock-glitch,event-drop,fc-stall,inverted-hang,mode-switch,rc-loss-airborne,stick-script,stick-slam,wind3 |
| 124768 | 78.657 s | alt 12.7 m, dist 120.0 m | flip,inverted-hang,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,stick-script,wind4 |
| 128240 | 90.182 s | alt 60.0 m, dist 2.0 m | arm-on-boot,flip,imu-rotated,inverted-hang,late-link,mode-switch,no-telemetry |
| 136245 | 110.593 s | alt 4.7 m, dist 120.0 m | bent-prop,imu-rotated,inverted-hang,late-link,mode-switch,no-telemetry,rc-loss,rc-loss-airborne,wind4 |
| 140008 | 24.713 s | alt 60.0 m, dist 4.6 m | fc-stall,inverted-hang,mode-switch,rc-loss,rc-loss-airborne,stick-slam |
| 148073 | 115.520 s | alt 60.0 m, dist 3.3 m | arm-throttle-up,flip,imu-dropout,imu-rotated,inverted-hang,rc-loss-airborne,stick-script,stick-slam |
| 152584 | 28.029 s | alt 60.0 m, dist 34.9 m | inverted-hang,late-link,no-telemetry,rc-loss,stick-script,wind4 |
| 154141 | 112.647 s | alt 60.0 m, dist 7.2 m | bent-prop,fc-stall,flip,imu-rotated,inverted-hang,jitter,mode-switch,rc-loss,stick-script |
| 157073 | 142.778 s | alt 2.3 m, dist 120.0 m | imu-rotated,inverted-hang,mode-switch,punch,rc-loss-airborne,stick-script,wind3 |
| 162610 | 119.224 s | alt 26.4 m, dist 120.0 m | bent-prop,imu-rotated,inverted-hang,no-telemetry,rc-loss-airborne,reboot-request,stick-script,stick-slam,wind4 |
| 179009 | 28.055 s | alt 60.0 m, dist 28.5 m | clock-wrap,fc-stall,inverted-hang,mode-switch,no-telemetry,rc-loss,wind3 |
| 193931 | 34.152 s | alt 60.0 m, dist 2.0 m | event-drop,inverted-hang,mode-switch,no-telemetry,rc-loss,stick-slam |
| 198739 | 49.488 s | alt 60.0 m, dist 0.5 m | imu-rotated,inverted-hang,mode-switch,moved-at-boot,no-telemetry,old-pack,stick-script |

## `failsafe_levels` (24 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 18892 | 15.906 s | tilt 33 deg 1000 ms into failsafe | impact,imu-dead-at-arm,imu-dropout,mode-switch,rc-loss,stick-script,stick-slam |
| 20265 | 10.294 s | tilt 88 deg 1000 ms into failsafe | imu-dropout,mode-switch,punch,rc-loss,stick-slam,wind1 |
| 22783 | 19.237 s | tilt 47 deg 1000 ms into failsafe | bent-prop,flip,imu-rotated,mode-switch,no-telemetry,rc-loss,stick-slam,wind2 |
| 25547 | 22.473 s | tilt 20 deg 2943 ms into failsafe | mode-switch,no-telemetry,old-pack,rc-loss,rc-loss-airborne,stick-slam,wind1 |
| 32400 | 13.397 s | tilt 20 deg 1595 ms into failsafe | arm-on-boot,bent-prop,imu-dropout,mode-switch,old-pack,rc-loss,stick-slam,wind4 |
| 34004 | 40.194 s | tilt 20 deg 7251 ms into failsafe | impact,imu-rotated,jitter,mode-switch,punch,rc-loss-airborne,stick-script,telemetry-loss,wind2 |
| 47928 | 8.637 s | tilt 27 deg 1000 ms into failsafe | fc-stall,kick,mode-switch,old-pack,rc-loss,stick-script,stick-slam,wind1 |
| 62807 | 20.880 s | tilt 22 deg 1000 ms into failsafe | drop,imu-rotated,mode-switch,on-its-side,rc-loss,reboot-request,stick-slam |
| 74133 | 43.314 s | tilt 22 deg 1000 ms into failsafe | clock-glitch,inverted-hang,kick,mode-switch,rc-loss,stick-slam |
| 81664 | 14.026 s | tilt 22 deg 1000 ms into failsafe | drop,event-drop,flip,impact,imu-rotated,rc-loss,stick-script,stick-slam |
| 82583 | 25.355 s | tilt 21 deg 1000 ms into failsafe | arm-throttle-up,drop,flip,mode-switch,no-telemetry,rc-loss,stick-slam |
| 85744 | 10.022 s | tilt 62 deg 1000 ms into failsafe | clock-wrap,low-battery,mode-switch,no-telemetry,rc-loss,stick-slam,wind3 |
| 101984 | 21.337 s | tilt 20 deg 1411 ms into failsafe | flip,imu-stuck,mode-switch,no-telemetry,rc-loss,stick-slam,wind3 |
| 126219 | 26.667 s | tilt 20 deg 4406 ms into failsafe | fc-stall,imu-stuck,rc-loss-airborne,stick-slam |
| 143965 | 28.913 s | tilt 20 deg 3044 ms into failsafe | arm-on-boot,desync,impact,imu-rotated,late-link,mode-switch,rc-loss-airborne,stick-slam |
| 144170 | 28.386 s | tilt 20 deg 3636 ms into failsafe | arm-throttle-up,clock-glitch,desync,imu-rotated,low-battery,mode-switch,no-telemetry,rc-loss-airborne,stick-slam,wind4 |
| 159523 | 27.525 s | tilt 23 deg 1666 ms into failsafe | flip,impact,mode-switch,no-telemetry,rc-loss-airborne,reboot-request |
| 164487 | 15.421 s | tilt 26 deg 1000 ms into failsafe | imu-rotated,imu-stuck,low-battery,mode-switch,rc-loss,wind-gust,wind4 |
| 166313 | 38.116 s | tilt 20 deg 4126 ms into failsafe | kick,late-link,no-telemetry,rc-loss-airborne,stick-slam,wind0 |
| 171378 | 13.382 s | tilt 38 deg 1000 ms into failsafe | flip,mode-switch,rc-loss,stick-slam |
| 172894 | 25.905 s | tilt 20 deg 1997 ms into failsafe | arm-on-boot,kick,no-telemetry,punch,rc-glitch,rc-loss-airborne,wind4 |
| 173788 | 21.011 s | tilt 53 deg 1000 ms into failsafe | clock-wrap,flip,mode-switch,no-telemetry,punch,rc-loss,stick-script,stick-slam |
| 187547 | 39.326 s | tilt 20 deg 10129 ms into failsafe | arm-on-boot,flip,kick,old-pack,rc-loss-airborne,wind-gust,wind4 |
| 201268 | 20.592 s | tilt 28 deg 1000 ms into failsafe | clock-wrap,flip,imu-rotated,rc-loss,rc-loss-airborne,stick-slam,wind1 |

## `pilot_can_arm` (17 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 33962 | 21.760 s | arm refused, blockers ArmBlockers(16) | clock-wrap,desync,late-link,mode-switch,punch,stick-slam,wind2 |
| 40332 | 16.905 s | arm refused, blockers ArmBlockers(16) | clock-glitch,imu-stuck,mode-switch,stick-slam,wind4 |
| 40944 | 12.690 s | arm refused, blockers ArmBlockers(16) | bent-prop,drop,imu-rotated,imu-stuck,late-link,low-battery,mode-switch,stick-script,wind4 |
| 56292 | 28.601 s | arm refused, blockers ArmBlockers(16) | imu-stuck,mode-switch,no-telemetry,stick-script,telemetry-loss |
| 70078 | 13.315 s | arm refused, blockers ArmBlockers(16) | bent-prop,imu-dead-at-arm,imu-dropout,imu-rotated,old-pack,on-its-side,stick-slam,wind3 |
| 73162 | 23.600 s | arm refused, blockers ArmBlockers(16) | clock-wrap,imu-rotated,imu-stuck,jitter,mode-switch,no-telemetry,old-pack,punch,stick-slam,wind4 |
| 78851 | 62.400 s | arm refused, blockers ArmBlockers(16) | event-drop,imu-stuck,inverted-hang,mode-switch,old-pack,stick-script,stick-slam,wind3 |
| 89471 | 17.006 s | arm refused, blockers ArmBlockers(16) | imu-dropout,kick,mode-switch,rc-glitch,stick-script,wind4 |
| 124883 | 21.187 s | arm refused, blockers ArmBlockers(16) | event-drop,flip,imu-rotated,imu-stuck,late-link,mode-switch,punch,reboot-request,stick-script,stick-slam,wind3 |
| 128081 | 54.706 s | arm refused, blockers ArmBlockers(16) | clock-glitch,imu-stuck,inverted-hang,mode-switch,stick-script,wind-gust,wind2 |
| 135126 | 51.603 s | arm refused, blockers ArmBlockers(16) | imu-rotated,imu-stuck,inverted-hang,rc-glitch,stick-slam,wind1 |
| 140605 | 7.705 s | arm refused, blockers ArmBlockers(16) | arm-on-boot,flip,imu-stuck,low-battery,no-telemetry,old-pack,stick-slam,wind2 |
| 149867 | 22.273 s | arm refused, blockers ArmBlockers(16) | flip,imu-stuck,late-link,mode-switch,no-telemetry,stick-slam,wind3 |
| 152386 | 37.002 s | arm refused, blockers ArmBlockers(16) | imu-dead-at-arm,imu-garbage,imu-stuck,inverted-hang,mode-switch,no-telemetry,punch,stick-script |
| 167287 | 25.679 s | arm refused, blockers ArmBlockers(16) | arm-on-boot,impact,imu-stuck,late-link,mode-switch,stick-script,stick-slam,wind3 |
| 174277 | 62.901 s | arm refused, blockers ArmBlockers(16) | imu-rotated,imu-stuck,inverted-hang,mode-switch,moved-at-boot,rc-loss,rc-loss-airborne,stick-script,stick-slam,wind1 |
| 194421 | 47.417 s | arm refused, blockers ArmBlockers(16) | bent-prop,event-drop,flip,imu-dropout,inverted-hang,mode-switch,no-telemetry,old-pack,stick-script |

## `angle_stays_level` (12 seeds, sample)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 20998 | 25.123 s | tilt 75 deg in angle mode | bent-prop,mode-switch,no-telemetry,punch,stick-slam,wind3 |
| 40382 | 11.814 s | tilt 99 deg in angle mode | arm-throttle-up,imu-dropout,imu-garbage,imu-rotated,no-telemetry,rc-loss,stick-slam,wind2 |
| 41444 | 19.877 s | tilt 75 deg in angle mode | arm-on-boot,clock-wrap,imu-stuck,late-link,mode-switch,no-telemetry,reboot-request,stick-slam,wind0 |
| 84958 | 12.872 s | tilt 75 deg in angle mode | clock-wrap,mode-switch,stick-slam |
| 144044 | 17.180 s | tilt 75 deg in angle mode | bent-prop,drop,event-drop,jitter,low-battery,mode-switch,no-telemetry,stick-slam,wind2 |
| 148237 | 16.955 s | tilt 75 deg in angle mode | mode-switch,stick-slam,wind1 |
| 149237 | 12.927 s | tilt 75 deg in angle mode | imu-rotated,mode-switch,no-telemetry,punch,stick-slam,wind3 |
| 160326 | 13.853 s | tilt 75 deg in angle mode | clock-wrap,imu-dropout,imu-rotated,low-battery,mode-switch,punch,rc-glitch,stick-slam |
| 166323 | 12.816 s | tilt 75 deg in angle mode | bent-prop,low-battery,mode-switch,stick-slam,wind3 |
| 166716 | 8.094 s | tilt 75 deg in angle mode | arm-on-boot,imu-dropout,imu-stuck,low-battery,mode-switch,no-telemetry,punch,wind2 |
| 167649 | 12.865 s | tilt 76 deg in angle mode | drop,imu-dropout,imu-rotated,mode-switch,old-pack,wind4 |

## `armed_above_idle` (10 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 3007 | 9.204 s | [0, 0, 0, 0] | drop,impact,imu-dropout,imu-misaligned,mode-switch,stick-script,wind1 |
| 43658 | 5.305 s | [0, 0, 0, 0] | imu-garbage,imu-rotated,mode-switch,motor-degraded,motors-swapped,old-pack,stick-script |
| 68801 | 7.712 s | [0, 0, 0, 0] | imu-garbage,imu-misaligned,kick,low-battery,mode-switch,punch,wind1 |
| 76589 | 8.900 s | [0, 0, 0, 0] | arm-throttle-up,drop,event-drop,imu-dropout,imu-rotated,no-telemetry,stick-slam |
| 78820 | 33.307 s | [0, 0, 0, 0] | flip,impact,imu-dropout,inverted-hang,low-battery,mode-switch,old-pack,rc-loss,stick-slam |
| 80484 | 64.314 s | [0, 0, 0, 0] | imu-garbage,inverted-hang,mode-switch,old-pack,rc-loss-airborne |
| 83668 | 32.430 s | [0, 0, 0, 0] | clock-wrap,imu-dropout,imu-garbage,no-telemetry,rc-loss-airborne |
| 133240 | 36.891 s | [0, 0, 0, 0] | event-drop,imu-garbage,mode-switch,no-telemetry,punch,rc-loss-airborne,wind2 |
| 140757 | 37.612 s | [0, 0, 0, 0] | arm-on-boot,fc-stall,flip,imu-dropout,imu-rotated,jitter,mode-switch,no-telemetry,old-pack,rc-loss-airborne,stick-slam |
| 159948 | 42.682 s | [0, 0, 0, 0] | arm-throttle-up,drop,imu-garbage,mode-switch,old-pack,punch,rc-loss-airborne,stick-script |

## `motor_fault_true` (7 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 15475 | 5.568 s | motor 0 reported faulty but healthy | bent-prop,desync,inverted-hang,motor-degraded,motors-swapped |
| 22305 | 9.765 s | motor 1 reported faulty but healthy | clock-wrap,desync,impact,imu-rotated,motors-swapped,stick-script,stick-slam,wind3 |
| 69489 | 5.782 s | motor 1 reported faulty but healthy | desync,event-drop,imu-misaligned,low-battery,mode-switch,motors-swapped,old-pack,stick-slam,wind1 |
| 105544 | 45.367 s | motor 1 reported faulty but healthy | arm-throttle-up,impact,inverted-hang,mode-switch,motor-dead,motors-swapped,moved-at-boot,punch,stick-slam,wind2 |
| 151348 | 8.665 s | motor 1 reported faulty but healthy | arm-on-boot,desync,event-drop,flip,late-link,mode-switch,motors-swapped,rc-loss-airborne,telemetry-loss |
| 159410 | 7.563 s | motor 1 reported faulty but healthy | inverted-hang,mode-switch,motor-dead,motors-swapped,stick-slam |
| 172689 | 10.247 s | motor 1 reported faulty but healthy | arm-on-boot,bent-prop,drop,fc-stall,late-link,mode-switch,motor-dead,motors-swapped,wind-gust,wind4 |

## `runaway_stops` (6 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 38877 | 10.604 s | still armed 5000 ms after throttle-up, alt 0.1 m | bent-prop,motors-swapped,no-telemetry,rc-loss-airborne,wind4 |
| 43263 | 7.504 s | still armed 5000 ms after throttle-up, alt 0.0 m | fc-stall,imu-misaligned,motor-degraded,motors-swapped,punch,rc-loss |
| 48686 | 9.309 s | still armed 5000 ms after throttle-up, alt 0.0 m | imu-rotated,mode-switch,motors-swapped,old-pack,reboot-request,stick-script,stick-slam,wind3 |
| 126531 | 8.627 s | still armed 5000 ms after throttle-up, alt 0.0 m | imu-rotated,jitter,mode-switch,motors-swapped,no-telemetry,stick-slam,wind0 |
| 152050 | 19.823 s | still armed 5000 ms after throttle-up, alt 0.0 m | arm-throttle-up,flip,mode-switch,motors-swapped,no-telemetry,old-pack,stick-slam,wind4 |
| 199260 | 10.856 s | still armed 5000 ms after throttle-up, alt 0.0 m | arm-throttle-up,clock-wrap,drop,inverted-hang,late-link,motors-swapped,old-pack,stick-slam,wind1 |

## `esc_fed` (3 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 3007 | 9.204 s | 10 ms without a motor command | drop,impact,imu-dropout,imu-misaligned,mode-switch,stick-script,wind1 |
| 78820 | 33.307 s | 12 ms without a motor command | flip,impact,imu-dropout,inverted-hang,low-battery,mode-switch,old-pack,rc-loss,stick-slam |
| 140757 | 37.612 s | 12 ms without a motor command | arm-on-boot,fc-stall,flip,imu-dropout,imu-rotated,jitter,mode-switch,no-telemetry,old-pack,rc-loss-airborne,stick-slam |

## `motor_fault_reported` (3 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 105544 | 45.508 s | motor 0 stalled 200 ms | arm-throttle-up,impact,inverted-hang,mode-switch,motor-dead,motors-swapped,moved-at-boot,punch,stick-slam,wind2 |
| 159410 | 7.703 s | motor 0 stalled 200 ms | inverted-hang,mode-switch,motor-dead,motors-swapped,stick-slam |
| 172689 | 10.386 s | motor 0 stalled 200 ms | arm-on-boot,bent-prop,drop,fc-stall,late-link,mode-switch,motor-dead,motors-swapped,wind-gust,wind4 |

## `rate_tracks` (1 seeds)

| Seed | At | Detail | Workload |
| --- | --- | --- | --- |
| 38210 | 12.000 s | pitch rate off by 42% RMS | imu-rotated,jitter,mode-switch,no-telemetry,punch,rc-glitch,rc-loss,stick-script,stick-slam |
