# [학번-이름] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: YYYY-MM-DD
**팀**: 개인 / 2인 1팀 (팀원: ...)

---

## 1. 설계 개요 (1 페이지)

1-2 문단으로:
- 이 과제에서 무엇을 달성하려고 했는가
- 어떤 제어기법을 선택했는가 (PID? LQR? SMC? gain scheduling?)
- 왜 그 기법을 선택했는가 — 강의 내용·논문·교과서 인용으로 정당화

각 제어기 한 줄 요약:
- **ctrl_lateral**: <기법> 으로 yaw rate 추종 + <기법> 으로 β-limiter
- **ctrl_longitudinal**: <기법>
- **ctrl_vertical**: <기법>
- **ctrl_coordinator**: <yaw moment → brake 분배 방식>

---

## 2. 수학적 모델링 (1-2 페이지)

### 2.1 사용한 plant 단순화
어떤 모델을 제어 설계에 사용했는가? (bicycle? 3DOF?) 학생은 14DOF plant 위에 검증하지만, **제어기 설계** 자체는 보통 더 단순한 모델 위에서 한다.

### 2.2 State-space 표현
$$\dot{x} = Ax + Bu, \quad y = Cx + Du$$

상태 변수, 입력, 출력 정의 + A, B 행렬 표현. Bicycle Model 사용 시:
$$x = [v_y, r]^T, \quad u = \delta$$
$$\dot{v}_y = -(C_f + C_r)/(mV_x)\,v_y + ((l_r C_r - l_f C_f)/(mV_x) - V_x)\,r + C_f/m\,\delta$$
$$\dot{r} = (l_r C_r - l_f C_f)/(I_z V_x)\,v_y - (l_f^2 C_f + l_r^2 C_r)/(I_z V_x)\,r + l_f C_f/I_z\,\delta$$

### 2.3 가정 + 한계
- 일정 종속도 (제어 설계 시 분리)
- 선형 타이어 (소슬립 영역)
- 그 외 본인이 사용한 가정

---

## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral — AFS + ESC

**설계 목표**:
- yaw rate 추종 (settling < 0.8s, overshoot < 10%)
- |β| > 3° 시 ESC 개입

**선택 기법**: <PID / LQR / SMC / ...>

**Gain 계산 과정**:
<예시>

PID 의 경우 Ziegler-Nichols / IMC tuning 사용:
- 1차 모델 근사: yaw rate transfer function $G(s) = \frac{K}{\tau s + 1}$
- K = ..., τ = ...
- ZN: Kp = 0.6/K·τ, Ki = Kp/(0.5·τ), Kd = Kp·(0.125·τ)

LQR 의 경우:
- $Q$ = diag(1, 100), $R$ = 1 — yaw rate error 100배 비중, slip angle penalty 추가
- `[K, P, e] = lqr(A, B, Q, R)` → K = [...]

**최종 게인 + 정당화**:
```matlab
CTRL.LAT.Kp = ...
CTRL.LAT.Ki = ...
CTRL.LAT.Kd = ...
BETA_THRESHOLD = deg2rad(3)
BETA_GAIN = ...
```

### 3.2 ctrl_longitudinal — 속도 + ABS

(동일 구조로)

### 3.3 ctrl_vertical — CDC (있다면)

(skyhook 등)

### 3.4 ctrl_coordinator — Actuator Allocation

yaw moment → 4-wheel brake 차동 분배:
$$\Delta T_f = M_z / t_f, \quad \Delta T_r = M_z / t_r$$
전후 비율 60:40 적용:
$$T_{FL} = T_{cmd}/4 + 0.6 \cdot \Delta T_f / 2, ...$$

(WLS allocation 사용 시 행렬식 + 가중치)

---

## 4. 시뮬레이션 결과 (2-3 페이지)

### 4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계

| 시나리오 | KPI | OFF | ON (본인) | Δ% |
|---|---|---|---|---|
| A1 DLC | sideSlipMax [°] | 4.51 | x.xx | -xx% |
| A1 | LTR_max | 0.948 | x.xx | -xx% |
| A3 step | yawRateOvershoot [%] | 2.81 | x.xx | -xx% |
| A4 SS | understeerGradient | -- | x.xx | -- |
| A7 BIT | sideSlipMax [°] | 46.3 | x.xx | -xx% |
| A7 | LTR_max | 0.745 | x.xx | -xx% |
| B1 brake | stoppingDistance [m] | 72.4 | x.xx | -xx% |
| D1 통합 | sideSlipMax [°] | 7.65 | x.xx | -xx% |

(`run('scripts/run_icc_benchmark.m')` 출력 + `run('scripts/grade.m')` 점수를 같이 표기)

### 4.2 핵심 plot — A1 DLC

![A1 trajectory comparison](figures/a1_trajectory.png)
*Figure 4.1 — A1 ISO 3888-1 DLC, 차량 trajectory (off vs on) vs reference path.*

![A1 yaw rate](figures/a1_yawrate.png)
*Figure 4.2 — A1 yaw rate 응답: reference (driver bicycle model), off (controller off), on (본인 설계).*

(plot 생성 예시:
```matlab
[r_off, k_off] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  k_on ] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.x_pos, r_off.y_pos, 'r--', r_on.x_pos, r_on.y_pos, 'b-', ...
             r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:');
xlabel('x [m]'); ylabel('y [m]'); legend('off','on','ref'); axis equal;
saveas(gcf, 'docs/figures/a1_trajectory.png');
```)

### 4.3 한 시나리오 deep dive — A7 (또는 본인이 가장 잘 푼 것)

A7 brake-in-turn 의 핵심:
- 베이스라인 sideSlipMax: 46.3° (스핀아웃)
- 본인 설계: x.x°
- 핵심 요인: <ESC 가 작동한 시점 / yaw moment 인가 패턴>

(plot + 분석)

---

## 5. 분석 + 한계 (1-2 페이지)

### 5.1 가장 성공적이었던 시나리오
어느 시나리오에서 가장 큰 KPI 개선이 있었는가? 왜?

### 5.2 가장 부족했던 시나리오
A4 정상선회에서 understeer gradient 가 안 맞았는가? 왜?
- 가설 1: ...
- 가설 2: ...

### 5.3 만약 더 시간이 있었다면
- ...
- ...

---

## 6. 참고문헌

[1] ISO 3888-1:2018 — Passenger cars — Test track for a severe lane-change manoeuvre.
[2] ISO 4138:2021 — Steady-state circular driving behaviour.
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer 2012. §2.5 (yaw rate response), §8 (ESC).
[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley 2008.
[5] (본인이 참고한 논문)

---

## 부록 A — 사용한 AI 도구

(student_info.m 의 ai_usage 항목과 일치하게)

예: ChatGPT 4 를 PID gain tuning 의 첫 추정에 사용 — proposed Kp=2.0, Ki=0.3 → 본인이 simulation 으로 Kp=1.5, Ki=0.15 로 조정.

---

## 부록 B — 본인 sim_params.m 변경사항

```matlab
% 변경 전:
%   CTRL.LAT.Kp = 1.0
%   CTRL.LAT.Ki = 0.1
% 변경 후:
CTRL.LAT.Kp = 2.5
CTRL.LAT.Ki = 0.4
CTRL.LAT.Kd = 0.08
% ...
```
