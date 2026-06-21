---
name: 질문
about: 환경 세팅 / 과제 명세 / 디버깅 관련 질문
labels: question
---

## 어떤 단계에서 발생했나요?
- [ ] 환경 세팅 (init_project.m, MATLAB 경로)
- [ ] 베이스라인 benchmark 실행
- [ ] 제어기 설계 / 디버깅
- [ ] 보고서 작성
- [ ] grade.m 또는 GitHub Actions
- [ ] 그 외

## 무엇을 했나요?
<!-- 재현 가능한 명령 시퀀스 -->
```matlab
% 예시
run('scripts/utils/init_project.m')
[r, k] = run_icc_scenario('A1', '14dof', 'Controller', 'on');
```

## 어떤 에러가 발생했나요?
```
% 에러 메시지 + 스택 트레이스 (Korean MATLAB 메시지도 OK)
```

## 어떤 결과를 기대했나요?
<!-- "sideSlipMax 가 3° 이하로 나와야 하는데 실제 5° 나옴" 같은 식 -->

## 환경
- MATLAB version:
- OS:
- 본인 fork URL (선택, 공개 가능한 경우):

## 이미 시도한 것
- [ ] TROUBLESHOOTING.md 확인
- [ ] 기존 Issue 검색
- [ ] init_project.m 재실행 + clear all
