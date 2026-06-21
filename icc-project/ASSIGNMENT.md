# Term Project — Integrated Chassis Control 설계

**과목**: 자동제어 (학부 3-4학년)
**난이도**: ★★★★☆
**소요 시간**: 4–6주
**팀**: 개인 또는 2인 1팀
**제출**: GitHub PR (마감일 23:59 KST, 시간 기준은 GitHub 서버 timestamp)

---

## 1. 과제 목적

차량 동역학 plant (BMW_5 14DOF) 와 표준 시험 시나리오가 미리 제공되는 환경에서, **횡·종·수직 통합 샤시 제어기**를 직접 설계해 베이스라인 (제어기 OFF) 대비 핸들링 안정성 / 제동 거리 / 승차감을 정량적으로 개선하라.

학습 목표:
1. 자동제어 강의에서 배운 PID / LQR / Pole placement / SMC 등을 **실제 차량 동역학 모델**에 적용
2. 다중 입출력 (MIMO) 시스템의 actuator allocation (Coordinator) 설계
3. ISO/FMVSS 표준 시나리오와 KPI 의 의미 이해
4. 베이스라인 정량 비교에 기반한 제어 성능 평가

---

## 2. 제공되는 것

| 영역 | 학생이 손대지 않음 (제공 그대로 사용) |
|---|---|
| Plant 모델 | bicycle, 3DOF, 7DOF, 14DOF |
| 시나리오 | A1, A2, A3, A4, A5, A6, A7, B1, B2, B3, C1, C2, D1 |
| Driver model | Stanley, Pure Pursuit, steering robot |
| KPI 라이브러리 | 14종 KPI 함수 + 임계값 |
| Runner / Benchmark | run_icc_scenario.m, run_icc_benchmark.m |

학생이 **수정 가능한 파일**:
- `scripts/control/ctrl_lateral.m`
- `scripts/control/ctrl_longitudinal.m`
- `scripts/control/ctrl_vertical.m`
- `scripts/control/ctrl_coordinator.m`
- `config/sim_params.m` 중 `CTRL.*` 와 `LIM.*` 항목만 (다른 항목 수정 시 채점 무효)
  - 예외: `SIM.solver` (plant 적분기 선택) 는 자유 변경 가능 — `'ode45'`(default)/`'ode23'`/`'ode15s'`/`'rk4'`/`'euler'` 중 선택. 채점 시 사용된 solver 가 그대로 적용됨. ode45 는 adaptive step + 변경 가능한 `SIM.solver_RelTol`/`AbsTol` 사용. 비교는 [scripts/utils/compare_solver.m](scripts/utils/compare_solver.m) 로 가능
- `scripts/student_info.m` (학번/이름 기입)
- `docs/report.md` (보고서)

**그 외 파일 수정은 채점 무효**. 수정해야 할 경우 사전에 GitHub Issue로 문의.

---

## 3. 설계 요구사항

### 3.1 ctrl_lateral (AFS + ESC) — 핵심

**입력**: yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt
**출력**: `deltaAdd.steerAngle` [rad], `deltaAdd.yawMoment` [Nm]

요구사항:
1. yaw rate 추종 — driver 입력 위에 보조 조향 (AFS) 인가
2. slip angle 절댓값이 임계 (예 3°) 를 넘으면 ESC 요 모멘트 인가 (driver와 반대 방향)
3. 속도 의존성 — 고속에서 게인 적응 (예: gain scheduling, LPV)
4. **금지**: hard-coded scenario-specific 처리 (예: "A1 이면 X, A7 이면 Y") — 일반화된 제어기 설계

### 3.2 ctrl_longitudinal (속도 + ABS)

**입력**: vxRef, vx, ax, ctrlState, CTRL, LIM, dt
**출력**: `forceCmd.Fx_total` [N], `forceCmd.brakeRatio`

요구사항:
1. 속도 추종 (실제 시나리오는 vxRef 변동 없음 — 의미는 cruise/decel transition)
2. ABS — 휠 슬립 비 (slip ratio) `|κ| > 0.12` 일 때 brake torque 감소
3. 저크 제한 (`LIM.MAX_JERK`)

### 3.3 ctrl_vertical (CDC / Active Damping)

**입력**: 4개 휠의 sprung/unsprung 가속도, suspension travel
**출력**: per-wheel 감쇠 계수 [Ns/m]

요구사항:
1. Skyhook 또는 hybrid (skyhook + groundhook) 알고리즘 권장
2. 감쇠 범위: `CTRL.VER.cMin ≤ c ≤ CTRL.VER.cMax` (sim_params 정의)
3. 빈도 분리 — body bounce (1–2 Hz) 와 wheel hop (10–15 Hz) 다른 전략

### 3.4 ctrl_coordinator (Actuator Allocation)

**입력**: latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM
**출력**: `actuatorCmd.{steerAngle, brakeTorque(4x1), dampingCoeff(4x1)}`

요구사항:
1. ESC yaw moment → 4-wheel brake 차동 분배 — track 반거리와 lever arm 고려
2. 종방향 제동 + ESC 차동 동시 작동 시 마찰원 제한 고려 (옵션, 가점)
3. WLS (weighted least squares) 또는 simple split 모두 가능

---

## 4. 제출물 (Deliverables)

```
your-fork/
├── icc-project/
│   ├── scripts/
│   │   ├── student_info.m        ← 학번/이름 기입 (필수)
│   │   └── control/
│   │       ├── ctrl_lateral.m      ← 채워서 제출
│   │       ├── ctrl_longitudinal.m
│   │       ├── ctrl_vertical.m
│   │       └── ctrl_coordinator.m
│   ├── config/
│   │   └── sim_params.m            ← CTRL.*, LIM.* 만 수정 허용
│   ├── grade_report.json           ← 본인 PC 에서 grade.m 실행 산출물 (필수 첨부)
│   └── docs/
│       └── report.md               ← 8-12 페이지 분량
```

### 4.1 코드 요구사항

- 모든 `ctrl_*.m` 가 `run_icc_benchmark.m` 에서 오류 없이 실행
- `grade.m` 가 종료 코드 0 반환
- 주석은 한국어 또는 영어 자유, 다만 함수 헤더 doc-string 필수

### 4.1.1 제출 워크플로 (필수)

본 과제는 GitHub Actions 가 MATLAB 을 직접 실행하지 **않습니다** (campus license 제약). 학생이 본인 PC 에서 채점 결과를 생성해 함께 push 해야 합니다.

```bash
# 1. ctrl_*.m 작성/수정 완료 후
# 2. 본인 PC MATLAB Command Window:
cd icc-project
run('scripts/grade.m')
# → icc-project/grade_report.json 생성됨

# 3. git add + commit + push (grade_report.json 포함 필수)
git add icc-project/scripts/control/ctrl_*.m \
        icc-project/scripts/student_info.m \
        icc-project/grade_report.json \
        icc-project/docs/report.md
git commit -m "Submit final controllers"
git push origin main
```

**무결성 검증**: `grade_report.json` 은 `ctrl_signature` (4개 ctrl_*.m 의 SHA256) 를 포함. GitHub Actions 가 학생 fork 의 실제 ctrl_*.m 파일과 hash 가 일치하는지 자동 검증. **`grade_report.json` 만 수동 편집해 점수를 올리려는 시도는 자동으로 차단됨** + 학칙 위반.

**TA 표본 재실행**: 강의자/TA 가 학기말에 전체 학생 fork 의 5-10% (만점 케이스는 100%) 를 본인 PC 에서 grade.m 재실행해 학생이 보고한 score 와 ±1점 이내 일치하는지 검증.

### 4.2 보고서 (`docs/report.md`)

[docs/report_template.md](docs/report_template.md) 시작점 활용.

필수 섹션:
1. **설계 개요** — 어떤 제어기법을 선택했고 그 이유
2. **수학적 모델링** — 사용한 plant 단순화 (bicycle? 3DOF?), 시스템 방정식, 가정
3. **제어기 설계** — gain 계산 과정 (PID tuning rule? LQR Q/R weight choice? pole placement?)
4. **시뮬레이션 결과** — P1 시나리오 6종에 대한 baseline vs designed controller KPI 표 + 핵심 plot
5. **분석 + 한계** — 어디서 잘 됐고, 어디서 부족한가, 왜 그런가
6. **참고문헌** — 인용 표준/논문/교과서

PDF가 아닌 Markdown 으로 제출. 수식은 LaTeX 문법.

---

## 5. 채점 매트릭스 (100점)

### 5.1 정량 (자동 채점, 70점)

각 시나리오에 대해 designed controller가 베이스라인 (제어기 OFF) 대비 KPI 를 얼마나 개선했는지 자동 측정. 두 가지 채점 view:

**(A) 시나리오 단위 binary view** — 본인 fork 의 GitHub Actions Summary 탭에 표시:

| 시나리오 | 합격 조건 (시나리오 합 ≥ 임계) | 점수 (binary) |
|---|---|---|
| **A3** Step Steer | KPI 합 ≥ 9/12 | 12 |
| **A1** ISO 3888-1 DLC | KPI 합 ≥ 10/15 | 15 |
| **A4** SS Circular | KPI 합 ≥ 6/10 | 10 |
| **A7** Brake-in-Turn | KPI 합 ≥ 10/15 | 15 |
| **B1** Straight Brake | KPI 합 ≥ 6/10 | 10 |
| **D1** DLC + Brake | KPI 합 ≥ 5/8 | 8 |

시나리오 합이 임계 미달이면 0점, 도달하면 만점 (binary). 본인 fork 의 Actions → 최근 run → Summary 탭에서 시나리오별 ✅/❌ 와 점수 확인.

**(B) 상세 KPI breakdown (PR 코멘트 + grade_report.json)** — 부분 점수 + 진단:

| 시나리오 | KPI | 만점 조건 | KPI 점수 |
|---|---|---|---|
| A3 | yawRateOvershoot ≤ 10% | 베이스라인 개선 시 만점 | 4 |
| A3 | yawRateRiseTime ≤ 0.3 s | | 4 |
| A3 | yawRateSettling ≤ 0.8 s | | 4 |
| A1 | sideSlipMax ≤ 3° | | 6 |
| A1 | LTR_max ≤ 0.6 | | 5 |
| A1 | lateralDevMax ≤ 0.7 m | | 4 |
| A4 | understeerGradient 0.003 ± 80% | | 5 |
| A4 | sideSlipMax ≤ 2° | | 5 |
| A7 | sideSlipMax ≤ 5° | | 8 |
| A7 | LTR_max ≤ 0.7 | | 7 |
| B1 | stoppingDistance ≤ 40 m | | 5 |
| B1 | absSlipRMS ≤ 0.10 | | 5 |
| D1 | sideSlipMax ≤ 4° | | 4 |
| D1 | LTR_max ≤ 0.6 | | 2 |
| D1 | lateralDevMax ≤ 1.0 m | | 2 |

각 KPI 는 임계 ±tol% 범위에서 선형 감점. 베이스라인 (Controller=off) 보다 악화 시 0점.

총 70점 (시나리오 합 view = KPI breakdown 합). View (A) binary 와 grade.m breakdown 은 일관됨 — binary view 는 시나리오 단위 합이 threshold 넘는 학생만 점수 주는 보수적 view.

**제출 코드가 run-time error 발생** → 0점 (단, GitHub Issue 로 사전 신고 시 부분 인정 가능).

### 5.2 정성 (수동 채점, 30점)

| 항목 | 만점 | 채점 기준 |
|---|---|---|
| 수학적 모델링의 깊이 | 8 | 가정의 정당성, 방정식의 유도 |
| 제어기법 선택의 정당성 | 8 | 왜 PID? 왜 LQR? trade-off 분석 |
| Gain 계산 과정 | 6 | tuning rule, pole placement, simulation iteration |
| 결과 분석 + 한계 인식 | 5 | "왜 A4 에서 잘 안 됐는가" 같은 솔직한 자기비판 |
| 보고서 가독성 + 인용 | 3 | 그림 캡션, 단위 표기, 표준 인용 |

### 5.3 가산점 (최대 +10)

- **+3**: A2 Severe DLC 또는 A5 FMVSS 126 sine-with-dwell 추가 통과
- **+3**: ctrl_coordinator 에서 마찰원 제한 + WLS allocation 구현
- **+2**: C1 single bump 또는 C2 sweep 에서 CDC 효능 입증
- **+2**: gain scheduling 또는 LPV 등 비선형 기법 적용 및 효능 입증

**상한 100점** (가산점 합산 후에도).

### 5.4 감점

- **−10**: 표절 의심 (Moss 검사 통과 못함)
- **−5**: 제출 형식 위반 (student_info.m 미기입, 허용 외 파일 수정)
- **−3**: 보고서 페이지 수 미달 (필수 섹션 6개 중 1개라도 누락)

---

## 6. 일정

| 주차 | 활동 |
|---|---|
| 1주차 | 환경 세팅, 베이스라인 benchmark 실행, 코드베이스 파악 |
| 2주차 | ctrl_lateral 1차 설계 (A3, A1 검증) |
| 3주차 | ctrl_longitudinal + ABS 설계 (B1 검증) |
| 4주차 | ctrl_coordinator + ESC (A7, D1 검증) |
| 5주차 | ctrl_vertical (옵션), gain 통합 튜닝 |
| 6주차 | 보고서 작성, 최종 제출 |

---

## 7. 협업 규칙

- 2인 1팀 시 commit 분포가 명백히 한쪽으로 치우치면 감점 (`git shortlog -sn` 으로 확인)
- 다른 팀과의 코드 공유 금지
- ChatGPT/Claude 등 AI 도구 사용 허용하나 **사용 사실 + 사용 범위를 보고서에 명시** (예: "PID gain tuning aid 로 활용")
- GitHub Issue 로 질문 환영 (모든 학생이 볼 수 있는 공개 채널)

---

## 8. FAQ + Troubleshooting

자주 묻는 문제는 [TROUBLESHOOTING.md](TROUBLESHOOTING.md) 참조.

환경 세팅 단계별 안내: [GETTING_STARTED.md](GETTING_STARTED.md).
