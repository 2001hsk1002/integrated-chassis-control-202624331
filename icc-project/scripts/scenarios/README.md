# scripts/scenarios — ICC 표준 시나리오 빌더

ICC (Integrated Chassis Control) 검증을 위한 표준 시나리오 정의 모듈. 각 시나리오는 driver/brake 입력 + 노면/마찰 + 평가 KPI를 단일 struct로 반환한다.

## 사용

```matlab
SIM = struct('dt', 0.001);
scenario = scenario_dispatcher('A1', SIM);              % dry (default)
scenario = scenario_dispatcher('B1', SIM, 'wet');       % weather variant
scenario = scenario_dispatcher('B3', SIM, 'dry');       % split-μ만 dry 의미 있음
```

반환 struct 필드:

| 필드 | 형식 | 설명 |
|---|---|---|
| `id, name, refStandard` | char | 시나리오 메타 |
| `tEnd, vx0` | scalar | 시뮬레이션 종료시간 [s], 초기속도 [m/s] |
| `steerDriver(t)` | function | 노면휠 조향각 [rad] |
| `brakeCmd(t)` | function | per-wheel brake torque [Nm] (4×1) |
| `z_road(t, wheel)` | function | per-wheel 노면 elevation [m] |
| `mu_wheel(t, wheel)` | function | per-wheel μ |
| `kpis` | cell | 평가 KPI 이름 |
| `refPath` (옵션) | N×2 | 목표 경로 [x,y] |
| `weather` | struct | dispatcher가 자동 채움 (name/precip/intensity/mu_scale) |
| `hd` | struct | HD scenario 자산 (objects/banking/laneMaterial/vehicleCatalog) |

## 시나리오 ID 표

### Category A — 횡방향 (Lateral / Handling)

| ID | 시나리오 | 표준 | 파일 |
|---|---|---|---|
| **A1** | DLC @ 80 km/h | ISO 3888-1:2018 | [scn_A1_dlc_80.m](scn_A1_dlc_80.m) |
| **A2** | Severe DLC (Moose) @ 60 km/h | ISO 3888-2:2011 | [scn_A2_dlc_severe.m](scn_A2_dlc_severe.m) |
| **A3** | Step Steer | ISO 7401:2011 | [scn_A3_step_steer.m](scn_A3_step_steer.m) |
| **A4** | Steady-State Circular R=50 m | ISO 4138:2021 | [scn_A4_ss_circular.m](scn_A4_ss_circular.m) |
| **A5** | Sine with Dwell | FMVSS 126 / ISO 19365 | [scn_A5_sine_with_dwell.m](scn_A5_sine_with_dwell.m) |
| **A6** | Sinusoidal Steer Sweep 0.1→1.1 Hz | ISO 7401:2011 §6.2 | [scn_A6_sine_sweep.m](scn_A6_sine_sweep.m) |
| **A7** | Brake-in-Turn | ISO 7975:2019 | [scn_A7_brake_in_turn.m](scn_A7_brake_in_turn.m) |

### Category B — 종방향 (Longitudinal / ABS·TC)

| ID | 시나리오 | 표준 | 파일 |
|---|---|---|---|
| **B1** | 직진 제동 (고-μ, 100 km/h→0) | ISO 21994:2007, UN-R 13H | [scn_B1_straight_brake.m](scn_B1_straight_brake.m) |
| **B2** | 직진 제동 (저-μ μ≈0.3, 80 km/h→0) | ISO 21994:2007 | [scn_B2_straight_brake_low_mu.m](scn_B2_straight_brake_low_mu.m) |
| **B3** | Split-μ 제동 (좌 1.0 / 우 0.3) | ISO 14512:1999 | [scn_B3_split_mu.m](scn_B3_split_mu.m) |

### Category C — 수직 (Vertical / CDC·Active Susp.)

| ID | 시나리오 | 표준 | 파일 |
|---|---|---|---|
| **C1** | Single Bump (cosine, 80 mm) | ISO 8608 / OEM | [scn_C1_single_bump.m](scn_C1_single_bump.m) |
| **C2** | 수직 sweep 0.1→25 Hz | OEM ride sweep | [scn_C2_sweep.m](scn_C2_sweep.m) |

### Category D — 통합 (Integration — ICC 핵심)

| ID | 시나리오 | 표준 | 파일 |
|---|---|---|---|
| **D1** | DLC + 0.3 g 제동 | OEM combined / ISO 3888-1+21994 파생 | [scn_D1_dlc_brake.m](scn_D1_dlc_brake.m) |

## Weather variant

| variant | precipitation | intensity | mu_scale | 적용 시나리오 |
|---|---|---|---|---|
| `dry` (default) | none | 0.0 | 1.0 | 전체 |
| `wet` | rain | 0.6 | 0.7 | A1/A2/A3/A4/A5/A6/A7/B1/D1 |
| `snow` | snow | 0.5 | 0.3 | B1 (드물게) |

`mu_wheel` 은 dispatcher에서 `mu_scale`로 자동 wrapping. B3 split-μ는 dry로 고정 (자체적으로 비대칭 μ 정의).

## HD scenario export

[hd/scn_export_hd.m](hd/scn_export_hd.m) — OpenSCENARIO 1.3 + OpenDRIVE 1.6 + OpenCRG + osc2cm 변환까지 한 번에. CarMaker 15 HD scenario import 호환.

자세한 내용은 `docs/model_calibration_report.md` §15–§17 참조.

## 테스트

- [test_roundtrip.m](test_roundtrip.m) — export → import 정합성 (A1/B1/C1/B3)
- [test_hd_export.m](test_hd_export.m) — HD asset bundle 산출 + osc2cm 검증 (A1–D1 × weather)
