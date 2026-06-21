# 제출 PR

## 학생 정보
- **학번**: <!-- 필수 -->
- **이름**: <!-- 필수 -->
- **팀**: 개인 / 2인 1팀 (팀이면 팀원 학번/이름)

## 체크리스트
- [ ] `scripts/student_info.m` 에 학번/이름 기입
- [ ] `scripts/control/ctrl_lateral.m` 본인 설계로 채움
- [ ] `scripts/control/ctrl_longitudinal.m` 본인 설계로 채움
- [ ] `scripts/control/ctrl_vertical.m` 본인 설계로 채움 (optional, 가산점)
- [ ] `scripts/control/ctrl_coordinator.m` 본인 설계로 채움
- [ ] `docs/report.md` 보고서 작성 완료 (8-12 페이지)
- [ ] 로컬에서 `run('scripts/grade.m')` 실행 후 점수 만족
- [ ] **`icc-project/grade_report.json` 첨부** (grade.m 실행 산출물 — ctrl_signature 자동 검증)
- [ ] ctrl_*.m 마지막 수정 이후 grade.m 재실행 완료 (signature mismatch 방지)
- [ ] AI 도구 사용 시 `student_info.m` 의 `ai_usage` 필드에 기재
- [ ] `config/sim_params.m` 의 CTRL.* / LIM.* (그리고 `SIM.solver` 토글) 만 수정 — 다른 항목 변경 시 채점 무효

## 설계 요약
<!-- 본인 설계의 핵심을 2-3 줄로 -->
- 횡방향:
- 종방향:
- 수직 (있다면):
- Coordinator:

## 베이스라인 대비 자체 측정 결과
<!-- 로컬 grade.m 결과 핵심만 -->
| 시나리오 | 핵심 KPI | OFF | ON | 개선 |
|---|---|---|---|---|
| A1 | sideSlipMax | x.xx° | x.xx° | -xx% |
| A7 | sideSlipMax | x.xx° | x.xx° | -xx% |

## 알려진 한계
<!-- 어디서 잘 안 됐는지 정직하게 -->

## 참고문헌
<!-- 핵심 인용만 — 보고서에 자세히 -->
