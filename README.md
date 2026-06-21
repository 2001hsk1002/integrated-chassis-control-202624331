# Integrated Chassis Control (ICC) — 자동제어 기말 프로젝트

학부 3-4학년 자동제어 과목용 기말 프로젝트. **차량 동역학 plant + 표준 시험 시나리오 + KPI 측정 인프라**가 제공되며, 학생은 **횡/종/수직 통합 샤시 제어기**를 설계·튜닝해 베이스라인 대비 성능 개선을 정량적으로 입증한다.

## 30초 onboarding

1. **본인 fork 생성**: 본 repo 페이지 우측 상단 **"Use this template"** → "Create a new repository" → 본인 GitHub 계정에 `integrated-chassis-control-<본인학번>` 등으로 생성. (GitHub Classroom 은 2026-05 부로 신규 사용 중단 — 학생이 직접 fork)
2. **무엇을 만드나**: [icc-project/scripts/control/](icc-project/scripts/control/) 의 `ctrl_lateral.m` / `ctrl_longitudinal.m` / `ctrl_vertical.m` / `ctrl_coordinator.m` 4개의 `%% TODO` 부분을 본인 설계로 채운다 — AFS·ESC·ABS·CDC·Coordinator.
3. **어떻게 검증하나**: 로컬에서 `run('scripts/run_icc_benchmark.m')` 로 P1 시나리오 6종을 Controller `off` vs `on` 자동 비교, KPI 개선율 표 출력.
4. **무엇을 제출하나**: 채워진 `ctrl_*.m` + 보고서 ([docs/report_template.md](icc-project/docs/report_template.md) 기반) + `student_info.m` (학번/이름 기입) + 본인 PC 에서 `run('scripts/grade.m')` 실행 후 생성된 **`icc-project/grade_report.json`**.
5. **어떻게 채점되나**: 본인 PC (Campus license MATLAB) 에서 `run('scripts/grade.m')` 실행 → `grade_report.json` 생성 → ctrl_*.m + grade_report.json 함께 commit/push. 본인 fork 의 GitHub Actions ([.github/workflows/classroom.yml](.github/workflows/classroom.yml)) 가 schema + `ctrl_signature` (SHA256) 검증 + KPI 임계 통과 여부 확인 → Actions 의 **Summary** 탭에 점수표 게시. 강의자/TA 가 표본 5-10% 를 본인 PC 에서 재실행해 정확성 확인.

6. **fork URL 제출**: 강의에서 안내된 양식 (Google Form 또는 staff repo 의 issue) 에 본인 fork URL 제출. 마감일 commit hash 기준으로 채점.

자세한 과제 명세: **[icc-project/ASSIGNMENT.md](icc-project/ASSIGNMENT.md)**
환경 세팅: **[icc-project/GETTING_STARTED.md](icc-project/GETTING_STARTED.md)**
자주 묻는 문제: **[icc-project/TROUBLESHOOTING.md](icc-project/TROUBLESHOOTING.md)**

## 무엇이 제공되는가

| 영역 | 파일 |
|---|---|
| Plant 모델 4종 | [scripts/plant/](icc-project/scripts/plant/) — bicycle / 3DOF / 7DOF / 14DOF, [선택 가능 적분기](icc-project/config/sim_params.m) `ode45`(default)/`ode23`/`ode15s`/`rk4`/`euler` |
| 표준 시나리오 13종 | [scripts/scenarios/](icc-project/scripts/scenarios/) — ISO 3888/4138/7401/14512/21994, FMVSS 126 |
| Driver model | [scripts/driver/](icc-project/scripts/driver/) — Stanley / Pure Pursuit / steering robot |
| KPI 라이브러리 | [scripts/utils/kpi/](icc-project/scripts/utils/kpi/) — yaw response, brake, ABS slip, sine-with-dwell, LTR, tire utilization, lateral path deviation, ride RMS |
| Runner + Benchmark | [run_icc_scenario.m](icc-project/scripts/run_icc_scenario.m) / [run_icc_benchmark.m](icc-project/scripts/run_icc_benchmark.m) |
| 검증 보고서 | [docs/model_calibration_report.md](icc-project/docs/model_calibration_report.md) — plant calibration vs CarMaker BMW_5 |

## 무엇을 만들어야 하는가

| 파일 | 설계 대상 |
|---|---|
| [ctrl_lateral.m](icc-project/scripts/control/ctrl_lateral.m) | AFS (active front steer) + ESC (yaw moment via differential brake) — yaw rate 추종 + slip angle 제한 |
| [ctrl_longitudinal.m](icc-project/scripts/control/ctrl_longitudinal.m) | 속도 추종 + ABS (slip ratio limiter) |
| [ctrl_vertical.m](icc-project/scripts/control/ctrl_vertical.m) | CDC (skyhook 또는 hybrid damping) |
| [ctrl_coordinator.m](icc-project/scripts/control/ctrl_coordinator.m) | 횡·종·수직 명령을 actuator(δ, brake torque ×4, damping ×4)로 분배 |

학부 3-4학년 자동제어 과목 범위에 맞춰 **PID / Compensator / Pole placement / LQR** 등 강의에서 배운 기법 자유 선택.

## Quick start

```bash
git clone https://github.com/<your-org>/integrated-chassis-control.git
cd integrated-chassis-control
# MATLAB R2023b+ 권장 (Automated Driving Toolbox, Vehicle Dynamics Blockset 권장하지만 필수 아님)
matlab -batch "run('icc-project/scripts/utils/init_project.m'); run('icc-project/scripts/run_icc_benchmark.m')"
```

기본 베이스라인 (제어기 PID 기본 게인) 으로 P1 6 시나리오가 돌면 환경 정상.

## For instructors

GitHub Classroom 설정 + 채점 운영: **[CLASSROOM_SETUP.md](CLASSROOM_SETUP.md)**

## License

MIT. 자세한 내용은 [LICENSE](LICENSE).

## Acknowledgments

차량 파라미터는 IPG Automotive CarMaker BMW_5 demo 데이터셋 기반. ISO/FMVSS/UN-R 시험 표준 참조.
