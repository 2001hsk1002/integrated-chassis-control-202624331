# Getting Started — ICC Term Project

5분 안에 베이스라인 simulation 이 돌아가도록 환경 세팅.

## 1. 요구사항

- **MATLAB R2023b 이상** (R2024b 권장)
- 권장 toolbox (없어도 동작하지만 일부 기능 제한):
  - Control System Toolbox (LQR, pole placement 등 사용 시)
  - Automated Driving Toolbox (시나리오 OpenSCENARIO export 시)
  - Vehicle Dynamics Blockset (Simulink 비교 시)
- Python 3.10+ (보고서 docx 빌드 시; 보고서가 .md 라면 생략 가능)
- Git

CarMaker 라이선스는 **불필요**. CarMaker 관련 스크립트는 자동 skip.

## 2. 본인 fork 생성 + Clone

GitHub Classroom 은 2026-05 부로 신규 사용 중단 ([gh.io/classroom-sunset](https://gh.io/classroom-sunset)). 본 template repo 의 우측 상단 **"Use this template"** → "Create a new repository" 로 본인 GitHub 계정에 fork 생성.

- Owner: 본인 계정
- Repository name: `integrated-chassis-control-<본인학번>` 권장 (예: `integrated-chassis-control-2026123456`)
- Visibility: **Private** (학생끼리 코드 노출 방지)

```bash
# 생성한 본인 fork clone
git clone https://github.com/<your-github-id>/integrated-chassis-control-<your-student-id>.git
cd integrated-chassis-control-<your-student-id>
```

학기말에 강의자에게 본인 fork URL 을 제출 (방법은 강의 공지 — Google Form 또는 staff repo 의 issue).

## 3. MATLAB path 초기화

MATLAB 을 `icc-project/` 디렉터리에서 열고:

```matlab
run('scripts/utils/init_project.m')
```

기대 출력:
```
=== ICC Project Initialization ===
Project root: ...
[sim_params] Vehicle params loaded from CarMaker BMW_5 ...
[sim_params] Parameters loaded. Plant=bicycle, VehicleSet=bmw5_cm15
[kpi_thresholds] Thresholds loaded.
경고: [carmaker_paths] CarMaker MATLAB path not found: ...  ← 무시 OK
=== Initialization Complete ===
```

CarMaker 경고는 무시. 학생용으로는 필요 없음.

## 4. 베이스라인 benchmark

```matlab
run('scripts/run_icc_benchmark.m')
```

P1 6 시나리오 (A1, A3, A4, A7, B1, D1) 가 각각 Controller `off` / `on` 두 번 실행되며 KPI 비교표 출력. 약 1–2 분 소요.

베이스라인 (제공된 PID 기본 게인) 의 예상 결과:
- A1 sideSlipMax: 약 1.6° (controller on)
- A7 sideSlipMax: 약 2.6° (controller on; off 면 46° 스핀아웃)
- D1 sideSlipMax: 약 1.8°

이 숫자가 비슷하게 나오면 환경 OK.

## 5. 학번/이름 기입

```matlab
edit scripts/student_info.m
```

학번/이름 입력 후 저장. 자동 채점이 이 파일을 참조한다.

## 6. 첫 수정 — Hello, Controller

`scripts/control/ctrl_lateral.m` 의 PID gain (`CTRL.LAT.Kp` 등) 을 sim_params.m 에서 바꿔보고 benchmark 재실행:

```matlab
edit config/sim_params.m   % CTRL.LAT.Kp = 2.0  로 변경
run('scripts/utils/init_project.m')  % 재로드
run('scripts/run_icc_benchmark.m')
```

KPI 가 달라지면 환경 정상.

## 7. 단일 시나리오 디버그

전체 benchmark 가 아니라 한 시나리오만:

```matlab
[result, kpi] = run_icc_scenario('A1', '14dof', 'Controller', 'on', 'SavePlot', false);
plot(result.x_pos, result.y_pos, 'b-', result.scenario.refPath(:,1), result.scenario.refPath(:,2), 'r--');
legend('vehicle', 'refPath'); axis equal;
```

이런 방식으로 trajectory 시각화 + 가설 검증.

## 8. 채점 (필수)

본 과제는 GitHub Actions 가 MATLAB 을 **직접 실행하지 않습니다** (campus license 가 외부 CI runner 에 자동 적용 불가). 학생이 본인 PC (Campus license MATLAB) 에서 채점기를 실행하고 그 산출물을 함께 push 합니다.

```matlab
cd icc-project
run('scripts/grade.m')
```

콘솔에 점수표 출력 + `icc-project/grade_report.json` 저장됨. **이 JSON 을 반드시 commit 해야 합니다** — GitHub Actions 가 이 파일의 `ctrl_signature` (4개 ctrl_*.m 의 SHA256) 와 실제 ctrl_*.m 의 hash 가 일치하는지 검증.

`grade_report.json` 의 주요 필드:

| 필드 | 의미 |
|---|---|
| `quantitative.score` | 정량 점수 (≤ 70) |
| `breakdown[]` | 시나리오별 KPI + 점수 |
| `ctrl_signature` | ctrl_*.m 4개의 SHA256 (자동 무결성 검증) |
| `matlab_version` | 학생 PC 의 MATLAB 버전 |
| `solver_used` | `SIM.solver` (ode45/rk4/...) |
| `timestamp` | grade.m 실행 시각 |

## 9. Submit

```bash
git add icc-project/scripts/control/*.m \
        icc-project/scripts/student_info.m \
        icc-project/grade_report.json \
        icc-project/docs/report.md
git commit -m "Final submission: <student_id>"
git push origin main
```

GitHub Actions 가:
1. `grade_report.json` 존재 + schema 검증
2. `ctrl_signature` 가 실제 ctrl_*.m hash 와 일치하는지 자동 검증
3. 시나리오별 점수가 임계 넘는지 binary 평가 → 본인 fork 의 Actions 탭 → 최근 run → **Summary** 탭에서 점수표 확인
4. PR 인 경우 KPI breakdown 코멘트 자동 작성

**주의**: ctrl_*.m 을 수정한 후에는 반드시 `grade.m` 을 다시 실행해 `grade_report.json` 을 재생성해야 합니다. 아니면 `ctrl_signature` 가 mismatch 되어 Actions 가 fail 합니다.

**학기말 검증**: 강의자/TA 가 학생 fork 의 5-10% 표본 (만점 케이스 100%) 을 본인 PC 에서 grade.m 재실행 → 보고된 score 와 ±1점 일치 확인.

---

## 다음 단계

- 과제 명세 자세히 보기 → [ASSIGNMENT.md](ASSIGNMENT.md)
- 자주 묻는 문제 → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- 보고서 시작점 → [docs/report_template.md](docs/report_template.md)
