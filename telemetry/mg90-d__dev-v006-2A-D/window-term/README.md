# Window term captures (mg90-d, board 2A-D)

One servo, 5 V, free horn unless said. Each capture's `meta.json` carries its board, plant stamp and drive limits.

| directory | what | firmware | set |
|---|---|---|---|
| `onsets-A/ol-{1,2,3}` | OpenLoop onsets from rest, 25 / 27 / 29 % duty, both ways, 300 ms windows | b7b51ee0, `g_window_q016` 0 | set of record 324e3267 |
| `onsets-A-brake/ol-{1,2,3}` | the same onsets, each preceded by a brake | b7b51ee0, `g_window_q016` 0 | as above |
| `g1/ol-1` | the same onsets with the reading the firmware takes (`i_tick_counts`, TEL chunk 0x240+12) beside the raw sample | 1a35457f, `g_window_q016` 1178 | friction 40 / 728, speed ceiling 8050 |
| `g3/c{300,800,1500,4025}` | Velocity cruises, 300 to 4025 c/s, both ways, 1.2 s legs, the integrator's held current | 1a35457f, `g_window_q016` 1178 | friction 40 / 728 |
| `g3b/c{300,800,1500,4025}` | the same cruises on the refit friction line | 1a35457f, `g_window_q016` 1178 | friction 43 / 947 |
