# Elephant race report

A 56 mph lorry overtakes a 55 mph lorry. Demand is 1400 veh/h/lane on a 20 km road, simulated for 780 s. Baselines use the same seed with both lorries kept in lane 1.

| Metric | 3-lane baseline | 3-lane overtake | 2-lane baseline | 2-lane overtake |
|---|---|---|---|---|
| Overtake duration (s) | 0 | 496 | 0 | 436 |
| Overtaking rate past lorries (veh/h) | 300 | 225 | 146 | 9 |
| Max cars held at lorry speed | 3 | 39 | 2 | 53 |
| Max platoon length (m) | 274 | 1811 | 148 | 1838 |
| Platoon growth rate (km/h) | 1.1 | 8.2 | 1.2 | 12.6 |
| Vehicle-minutes stuck | 4 | 102 | 8 | 136 |
| Drivers held up | 8 | 66 | 4 | 58 |
| Total time spent (veh-h) | 97.56 | 97.63 | 64.88 | 65.17 |
| Mean car speed (km/h) | 96.1 | 95.9 | 94.8 | 94.0 |
| Lane changes | 1574 | 1529 | 596 | 545 |
| Moves into the outside lane | 410 | 417 | 295 | 274 |
| Hard-braking events (< -3 m/s²) | 37 | 37 | 13 | 14 |

## Speed-gap sweep (3 lanes)

| Gap (mph) | Overtake (s) | Finished | Extra veh-min stuck | Extra delay (veh-min) | Overtaking-rate drop |
|---|---|---|---|---|---|
| 0.5 | 630 | no | 56 | 0.8 | 11% |
| 1.0 | 496 | yes | 98 | 4.5 | 25% |
| 2.0 | 204 | yes | 48 | 6.0 | 33% |
| 3.0 | 132 | yes | 39 | 4.6 | 33% |
| 5.0 | 74 | yes | 27 | 0.1 | 29% |
