# Troubleshooting

자주 발생하는 문제와 해결법.

## 설치 / 초기화

### Q1. `init_project.m` 에서 "carmaker_paths CarMaker MATLAB path not found" 경고
**A**: 무시. CarMaker 라이선스가 없어도 학생 과제는 모두 동작.

### Q2. `'scenario_dispatcher' 을(를) 인식할 수 없는 함수 또는 변수입니다`
**A**: MATLAB current directory 가 `icc-project/` 인지 확인. `init_project.m` 을 먼저 실행했는지 확인.

### Q3. `Error using export ... 'OpenSCENARIOVersion' value must be 1.0 or 1.1`
**A**: MATLAB R2023b 미만에서 발생. R2024b 권장. 또는 [scn_export_osc.m](scripts/scenarios/export/scn_export_osc.m) 의 export 부분만 회피하면 됨 (학생 과제 핵심에는 영향 없음).

### Q4. `Error: Unable to perform assignment because the indices on the left side are not compatible`
**A**: `ctrl_*.m` 의 출력 shape 가 맞지 않음. `deltaAdd.steerAngle` 은 scalar [rad], `actuatorCmd.brakeTorque` 는 4×1 column vector.

---

## 제어기 설계

### Q5. `lateralDevMax` 가 비현실적으로 큼 (수십 m)
**A**: 옛 KPI 버그는 수정됨 ([kpi_lateral_path_deviation.m:24](scripts/utils/kpi/kpi_lateral_path_deviation.m#L24)). 그래도 큰 값이 나오면 차량이 path 밖으로 이탈 (스핀아웃 등). `result.x_pos`/`result.y_pos` 시각화로 확인.

### Q6. `yawRateOvershoot` 가 수만 % 로 나옴
**A**: `r_ss` (정상상태 yaw rate) 가 거의 0인 시나리오 (예: A1 DLC 는 net yaw 거의 0) 에서 정규화 문제 — KPI 자체의 한계. A3 step steer 에서만 의미 있는 KPI.

### Q7. PID gain 을 키웠는데 sideSlipMax 가 더 커짐
**A**: AFS 가 yaw rate ref 를 너무 공격적으로 추종하려고 슬립 한계를 넘어섬. β-limiter 게인 (`BETA_GAIN`, `BETA_THRESHOLD` in ctrl_lateral.m) 도 함께 조정 필요. 또는 yaw rate ref 자체를 friction-limited 로 saturate.

### Q8. A7 brake-in-turn 베이스라인이 스핀아웃 (sideSlipMax > 40°)
**A**: 정상. 이게 ESC 가 풀어야 할 문제. 제어기 ON 시 5° 이하로 떨어뜨리는 게 목표.

### Q9. ABS 가 작동 안 함 (`absSlipRMS` 가 변화 없음)
**A**: 현재 `ctrl_longitudinal.m` 은 슬립 기반 모듈레이션 없음. 학생이 ABS 로직 추가 필요 — wheel slip ratio `κ` 측정 후 `|κ| > 0.12` 일 때 brake torque 감소.

### Q10. C1 single bump 에서 ride RMS 가 controller on/off 동일
**A**: 현재 `ctrl_vertical.m` 은 stub. 학생이 skyhook 등 구현 필요. 또는 `actCmd.dampingCoeff` 가 plant 의 14DOF damper 입력으로 잘 전달되는지 확인.

---

## 시뮬레이션 / KPI

### Q11. benchmark 가 시나리오마다 결과가 다름 (재현 불가)
**A**: 단일 plant 는 deterministic. 다른 결과가 나온다면 (a) gain 을 사이에 바꿨거나, (b) `init_project.m` 을 재실행 안 함. `clear all` 후 `init_project` → `run_icc_benchmark` 순서.

### Q12. A4 (정상선회) 가 8초가 너무 짧음
**A**: A4 의 tEnd 는 12 s. 정상상태 도달까지 약 8s + hold 4s. 시뮬레이션이 짧게 끝나면 plant 가 발산 (sideSlip 발산) 한 경우. controller 가 너무 공격적인지 확인.

### Q13. CarMaker 데이터셋 없으면 plant 가 안 돌아가나
**A**: 아니. `init_project.m` 이 CarMaker 데이터 없으면 hardcoded BMW_5 fallback 으로 대체. 학생 과제에는 영향 없음.

---

## Git / 제출

### Q14. push 했는데 GitHub Actions 가 안 돌아감
**A**: (a) PR을 생성했는지 확인. (b) Actions 탭에서 workflow 가 disabled 인지 확인 (Fork 한 repo 는 첫 push 시 수동 activate 필요).

### Q15. Actions 가 "MATLAB license issue" 로 실패
**A**: 강의용 학교 GitHub Education 계정 사용 권장. 학교 라이선스 없으면 로컬 grade.m 실행 결과 (`grade_report.json`) 를 PR 에 첨부.

### Q16. `student_info.m` 안 채우면 어떻게 됨?
**A**: 채점은 진행되지만 -5점 감점 + 채점 시트 매칭 불가 가능성.

### Q17. 다른 팀 코드 의도치 않게 봤는데 어떻게 함
**A**: 보고서에 명시적 기재 (예: "X팀 PR review 중 우연히 ctrl_lateral 일부 봄"). 평가자 판단으로 감점 없을 수 있음. 묵비가 더 위험.

---

## 보고서

### Q18. report.md 에 plot 어떻게 넣나
**A**: MATLAB 에서 `saveas(gcf, 'docs/figures/myplot.png')` 로 저장 후 `![설명](figures/myplot.png)` 으로 embed. `docs/figures/` 디렉터리 자동 생성됨.

### Q19. 수식은 어떻게 쓰나
**A**: MathJax 호환 LaTeX. inline: `$\delta = K_p e$`, display: `$$r = \frac{V_x}{L}\,\delta$$`

### Q20. 보고서 길이 제한
**A**: 8–12 페이지 (figure 포함). 절대값 아니라 가독성 중시 — 너무 짧으면 분석 부족, 너무 길면 군더더기.

---

## 그 외

자료에 없는 문제는 **GitHub Issue** 로 (모든 학생이 답을 볼 수 있도록). 개인 메일로 보내지 마세요.
