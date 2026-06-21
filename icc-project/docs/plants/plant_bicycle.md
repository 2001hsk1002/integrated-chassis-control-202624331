# Plant: 2-DOF Linear Bicycle Model

## 1. Overview

`plant_bicycle.m`은 ICC 프로젝트에서 가장 단순한 플랜트 모델이다.
좌우 타이어를 하나로 합산한 **선형 2자유도(2-DOF) Bicycle Model**을 사용하며,
횡방향 동역학(lateral dynamics)만 모델링한다.

- **자유도**: 2 (횡방향 속도 `vy`, 요 레이트 `r`)
- **종방향 속도**: 일정 (상수 `vx`)
- **타이어**: 선형 코너링 강성 모델
- **롤/피치/서스펜션**: 미모델링 (zero-fill)

**적용 단계**: 제어 알고리즘 초기 개발 및 빠른 반복 검증

---

## 2. Physical Model

좌우 타이어를 전축/후축 각각 하나로 합산하여, 차량을 2바퀴 자전거처럼 단순화한다.

```
        δ (조향각)
        │
        ▼
    ┌───●───┐  ← 전축 (Cf, lf)
    │       │
    │  CG ──┤──→ vx (일정)
    │   ↓vy │
    │       │
    └───●───┘  ← 후축 (Cr, lr)
```

### 가정
1. 종방향 속도 `vx`는 일정 (제동/가속에 의한 변화 무시)
2. 타이어 횡력은 슬립 앵글에 선형 비례: `Fy = Cα · α`
3. 롤, 피치, 수직 운동 무시 (평면 운동만 고려)
4. 소각도 근사: `sin(δ) ≈ δ`, `cos(δ) ≈ 1`

---

## 3. Equations of Motion

차체 고정 좌표계에서의 횡방향 및 요 운동 방정식:

### 3.1 횡방향 힘 평형

$$m(\dot{v}_y + v_x \cdot r) = F_{yf} + F_{yr}$$

### 3.2 요 모멘트 평형

$$I_z \dot{r} = l_f F_{yf} - l_r F_{yr} + M_z$$

여기서:
- `m` : 차량 질량 [kg]
- `Iz` : 요 관성 모멘트 [kg·m²]
- `vy` : 횡방향 속도 [m/s]
- `r` : 요 레이트 [rad/s]
- `vx` : 종방향 속도 (상수) [m/s]
- `Fyf` : 전축 횡력 [N]
- `Fyr` : 후축 횡력 [N]
- `lf` : CG-전축 거리 [m]
- `lr` : CG-후축 거리 [m]
- `Mz` : 외부 요 모멘트 (ESC 차동 브레이크) [Nm]

---

## 4. Tire Model (Linear)

선형 코너링 강성 모델:

$$F_{yf} = C_f \cdot \alpha_f, \quad F_{yr} = C_r \cdot \alpha_r$$

타이어 슬립 앵글 (소각도 근사):

$$\alpha_f = \delta - \frac{v_y + l_f r}{v_x}, \quad \alpha_r = -\frac{v_y - l_r r}{v_x}$$

여기서:
- `Cf` : 전륜 코너링 강성 [N/rad] (default: 80,000)
- `Cr` : 후륜 코너링 강성 [N/rad] (default: 85,000)
- `δ` : 로드휠 조향각 [rad]

---

## 5. State-Space Formulation

상태 벡터와 입력:

$$\mathbf{x} = \begin{bmatrix} v_y \\ r \end{bmatrix}, \quad u = \delta$$

타이어 힘을 대입하면 표준 상태공간 형태:

$$\dot{\mathbf{x}} = A\mathbf{x} + Bu + \begin{bmatrix} 0 \\ M_z / I_z \end{bmatrix}$$

### A 행렬 (2×2)

$$A = \begin{bmatrix} -\dfrac{C_f + C_r}{m v_x} & -v_x - \dfrac{C_f l_f - C_r l_r}{m v_x} \\[8pt] -\dfrac{C_f l_f - C_r l_r}{I_z v_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v_x} \end{bmatrix}$$

### B 행렬 (2×1)

$$B = \begin{bmatrix} \dfrac{C_f}{m} \\[8pt] \dfrac{C_f l_f}{I_z} \end{bmatrix}$$

### 물리적 의미

| 원소 | 의미 |
|------|------|
| `A(1,1)` | 횡속도에 대한 감쇠 (타이어 횡력 → 횡속도 저항) |
| `A(1,2)` | 요 레이트 → 횡속도 커플링 (원심력 + 타이어 효과) |
| `A(2,1)` | 횡속도 → 요 모멘트 커플링 |
| `A(2,2)` | 요 레이트에 대한 감쇠 (요 댐핑) |
| `B(1)` | 조향 → 횡력 직접 입력 |
| `B(2)` | 조향 → 요 모멘트 직접 입력 |

---

## 6. Numerical Integration

Forward Euler 적분:

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \dot{\mathbf{x}}_k \cdot \Delta t$$

여기서 `Δt = 0.001 s` (1 kHz).

선형 시스템이므로 Euler 적분에서도 수치적으로 안정하다
(A 행렬 고유값의 실수부가 음수이면 `Δt < 2/|λ_max|` 조건 충족).

---

## 7. ESC Yaw Moment Reconstruction

`ctrl_coordinator`가 `latCmd.yawMoment`를 차동 브레이크 토크로 변환하므로,
Bicycle Model에서는 이를 역으로 복원하여 직접 요 모멘트로 적용한다:

$$M_z = \frac{(T_{FR} - T_{FL})}{r_w} \cdot \frac{w_f}{2} + \frac{(T_{RR} - T_{RL})}{r_w} \cdot \frac{w_r}{2}$$

여기서:
- `T_FL, T_FR, T_RL, T_RR` : 개별 바퀴 브레이크 토크 [Nm]
- `rw` : 타이어 유효 반경 [m]
- `wf, wr` : 전/후 트레드 [m]

---

## 8. Output Mapping

| Output | Equation | 비고 |
|--------|----------|------|
| `vx` | `vx` (상수) | 종방향 속도 변화 없음 |
| `vy` | `x(1)` | 상태에서 직접 |
| `ax` | `0` | 종방향 가속도 없음 |
| `ay` | `vx · r` | 원심 가속도 근사 |
| `yawRate` | `x(2)` | 상태에서 직접 |
| `slipAngle` | `atan2(vy, vx)` | 차체 슬립 앵글 β |
| `roll` | `0` | 미모델링 |
| `pitch` | `0` | 미모델링 |
| `tire.*` | `0` (zero-fill) | 개별 타이어 데이터 없음 |
| `susp.*` | `0` (zero-fill) | 서스펜션 데이터 없음 |

---

## 9. Steady-State Analysis

### 정상상태 요 레이트

`ẋ = 0` 조건에서:

$$r_{ss} = \frac{v_x \cdot \delta}{L + K_{us} \cdot v_x^2}$$

여기서 **언더스티어 그래디언트**:

$$K_{us} = \frac{m l_r}{2 C_f L} - \frac{m l_f}{2 C_r L}$$

- `Kus > 0` : 언더스티어 (속도 증가 → 요 레이트 감소)
- `Kus = 0` : 뉴트럴 스티어
- `Kus < 0` : 오버스티어

### 기본 파라미터에서의 Kus

```
Kus = (1500 × 1.4) / (2 × 80000 × 2.6) - (1500 × 1.2) / (2 × 85000 × 2.6)
    = 0.00505 - 0.00408
    = 0.00097  (언더스티어)
```

---

## 10. Limitations

| 한계 | 영향 | 상위 모델에서 해결 |
|------|------|-------------------|
| 선형 타이어 | 대 슬립 앵글에서 횡력 과대 추정 | 3DOF (Magic Formula) |
| 일정 속도 | 제동/가속 효과 무시 | 3DOF (vx 동역학) |
| 바퀴 회전 없음 | 슬립 비 계산 불가, ABS/TCS 미지원 | 7DOF |
| 롤/피치 없음 | 하중 이동 무시, 승차감 미평가 | 14DOF |
| 서스펜션 없음 | CDC 제어기 검증 불가 | 14DOF |

---

## 11. Parameter Table

| Symbol | Parameter | Default | Unit |
|--------|-----------|---------|------|
| `m` | 차량 질량 | 1500 | kg |
| `Iz` | 요 관성 모멘트 | 2500 | kg·m² |
| `lf` | CG-전축 거리 | 1.2 | m |
| `lr` | CG-후축 거리 | 1.4 | m |
| `L` | 축간 거리 | 2.6 | m |
| `Cf` | 전륜 코너링 강성 | 80,000 | N/rad |
| `Cr` | 후륜 코너링 강성 | 85,000 | N/rad |
| `wf` | 전륜 트레드 | 1.55 | m |
| `wr` | 후륜 트레드 | 1.55 | m |
| `rw` | 타이어 유효 반경 | 0.31 | m |

---

## 12. Source Files

| File | Role |
|------|------|
| `scripts/plant/plant_bicycle.m` | Plant adapter 래퍼 (입출력 변환) |
| `scripts/control/calc_bicycle_model.m` | A, B, C, D 행렬 계산 |
| `scripts/control/calc_ref_yaw_rate.m` | 정상상태 목표 요 레이트 |
| `config/sim_params.m` | 차량 파라미터 정의 |

---

## 13. References

[1] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed. New York, NY: Springer, 2012, ch. 2–3. — Bicycle Model 유도, 상태공간 정식화, 언더스티어 그래디언트의 기본 교과서.

[2] M. Abe, *Vehicle Handling Dynamics: Theory and Application*, 2nd ed. Oxford: Butterworth-Heinemann, 2015, ch. 4–5. — 선형 2-DOF 모델의 정상상태/과도 응답 분석, 안정성 조건.

[3] H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed. Oxford: Butterworth-Heinemann, 2012, ch. 1, 7. — 선형 코너링 강성 정의, Magic Formula로의 확장(3DOF 이상에서 사용).

[4] U. Kiencke and L. Nielsen, *Automotive Control Systems: For Engine, Driveline, and Vehicle*, 2nd ed. Berlin: Springer, 2005, ch. 12. — ESC 요 모멘트 제어, 차동 브레이크를 통한 안정성 보상.

[5] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed. Hoboken, NJ: Wiley, 2008, ch. 6. — 차량 횡방향 동역학 기초, 타이어-차량 상호작용.

[6] ISO 8855:2011, *Road vehicles — Vehicle dynamics and road-holding ability — Vocabulary*. — 차량 동역학 용어 및 좌표계 정의 (SAE/ISO 좌표계).

[7] R. N. Jazar, *Vehicle Dynamics: Theory and Application*, 3rd ed. Cham: Springer, 2017, ch. 8. — Bicycle Model의 고유값 분석, 임계 속도, 안정성 판별.

[8] MathWorks, "Vehicle Dynamics Blockset Documentation: Passenger Vehicle Dynamics Models," 2024. — MATLAB/Simulink Vehicle Dynamics Blockset의 3DOF/7DOF/14DOF 모델 구현 참고.

[9] W. Milliken and D. Milliken, *Race Car Vehicle Dynamics*, Warrendale, PA: SAE International, 1995, ch. 5–6. — 요-사이드슬립 연립 운동 방정식의 원형, 뉴트럴 스티어 포인트 개념.
