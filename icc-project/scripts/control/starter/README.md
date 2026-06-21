# Starter templates — ctrl_*.m skeletons

학생이 채워야 할 4개 제어기 함수의 빈 스켈레톤.

## 사용법

```bash
# 본인 fork 에서, 시작 시 한 번만:
cp scripts/control/starter/ctrl_lateral_starter.m       scripts/control/ctrl_lateral.m
cp scripts/control/starter/ctrl_longitudinal_starter.m  scripts/control/ctrl_longitudinal.m
cp scripts/control/starter/ctrl_vertical_starter.m      scripts/control/ctrl_vertical.m
cp scripts/control/starter/ctrl_coordinator_starter.m   scripts/control/ctrl_coordinator.m
```

이렇게 하면 기존 reference 구현이 starter (빈 함수) 로 덮어쓰이며, 본인 설계를 작성한다.

(주의: 단순히 cp 하지 말고 본인 코드로 채워야 함. starter 그대로 두면 controller off 동등 → 0점)

## 채워야 할 4개 함수

| 함수 | 설계 대상 |
|---|---|
| [ctrl_lateral_starter.m](ctrl_lateral_starter.m) | AFS + ESC (yaw rate 추종 + β-limiter) |
| [ctrl_longitudinal_starter.m](ctrl_longitudinal_starter.m) | 속도 추종 + ABS |
| [ctrl_vertical_starter.m](ctrl_vertical_starter.m) | CDC (skyhook 등) |
| [ctrl_coordinator_starter.m](ctrl_coordinator_starter.m) | Actuator allocation (yaw moment → 4-wheel brake 차동) |

각 함수의 docstring 에 입력/출력 + 요구사항 + 힌트가 자세히 적혀있음.

## 참조 자료

- 과제 명세: [../../../../ASSIGNMENT.md](../../../ASSIGNMENT.md)
- 환경 세팅: [../../../../GETTING_STARTED.md](../../../GETTING_STARTED.md)
- KPI 정의: [../../../../docs/icc_test_protocol.md](../../../docs/icc_test_protocol.md) §2
- 차량 파라미터: [../../../../config/sim_params.m](../../../config/sim_params.m)
- Bicycle Model 도우미: [../../control/calc_bicycle_model.m](../calc_bicycle_model.m), [calc_ref_yaw_rate.m](../calc_ref_yaw_rate.m)
