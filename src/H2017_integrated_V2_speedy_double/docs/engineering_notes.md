# H2017 기술 상태와 설정 근거

이 문서는 현재 코드에 적용된 중요한 설정, 최근 회귀 수정, 남은 문제만 유지한다.
실행 명령은 [통합 실행 명령 가이드](test5_run_guide.md), 데이터 흐름은
[아키텍처](architecture.md)를 따른다.

## 최신 검증 기준

검증일: 2026-08-11, `ROS_DOMAIN_ID=129`, `rmw_fastrtps_cpp`.

| 조건 | 결과 |
|---|---|
| 순수 로직 테스트 | 시뮬레이터 `128 passed`, 비전 `8 passed` |
| `--box-count 5 --seed 42` | 검출·분류 `5/5`, 적재 안정성 `5/5 PASS`, 물리 경과 46.3초 |
| `--box-count 6 --seed 42` | 검출·분류 `6/6`, 6번째 2단 박스 릴리스까지 성공 |
| 6번째 높이 | 계획 `bottom_z=0.28 m`, 정착 후 지지면 오차 약 `-1.7 mm` |
| 관측 drift | 6번째 정착 후 XY 이동 `7.4~7.6 mm`(WARNING, 실제 pose 반영) |

5개 조건은 GUI·비전·RViz 통합 확인용 기준이다. 6개 조건은 2단 적재 회귀용이다.

## 박스와 표면

`reference-20260809T222855Z-1-001`의 정상 팔레타이징 설정과 박스 사진을 기준으로
임의의 고마찰·고감쇠 값을 사용하지 않는다.

| 항목 | 값 |
|---|---:|
| 밀도 | `150 kg/m³` |
| 정지 / 동 마찰 | `0.45 / 0.35` |
| 선형 / 각 감쇠 | `0.1 / 0.1` |
| 반발계수 | `0.0` |
| sleep / stabilization threshold | `0.01` |
| max depenetration velocity | `5.0 m/s` |
| 골판지 기준 RGB | `[0.62, 0.47, 0.30] ± 0.08` |
| roughness / specular / metallic | `0.85 / 0.05 / 0.0` |

질량은 고정값이 아니라 실측 치수의 부피에 밀도를 곱한다. 예를 들어 1호는
약 `0.564 kg`, 4호는 약 `5.338 kg`이다.

## 카탈로그와 비전 필터

사용 카탈로그는 1~4호다.

```text
1호 0.22 × 0.19 × 0.09 m
2호 0.27 × 0.18 × 0.15 m
3호 0.34 × 0.25 × 0.21 m
4호 0.41 × 0.31 × 0.28 m
```

비전 노드는 컨베이어 기준면 위 점을 DBSCAN으로 군집화한 뒤 다음 범위에 들어오는
후보만 박스로 인정한다.

```text
최소 footprint 변: 0.12 m
최대 footprint 변: 0.50 m
높이: 0.05~0.35 m
```

후보 선택은 World 원점 거리가 아니라 카메라 좌표의 광축 거리로 수행한다.
이 변경으로 Pick Zone 근처 로봇 링크가 실제 박스보다 먼저 선택되던 문제를
제거했다. 트리거 직후 빈 과거 프레임을 받을 때는 비전 노드가 최대 2프레임,
`IntakeStation`이 null 결과를 최대 2회 추가 요청한다.

## DeepPack3D 온라인 계획

현재 설정:

```text
method=baf
resolution=0.01 m
outer edge_margin=0.015 m, center edge_margin=0 m
box_gap=0.015 m
max_stack_height=0.8 m
min_support_ratio > 0.5
lookahead=1
yaw rotation=false
```

각 로봇이 자기 팔레트 반쪽에 독립 `PackingSession`을 갖는다. 이미 확정한 배치는
후속 입고로 이동하지 않는다. 두 세션의 격자 y=0은 모두 중앙선을 가리키며,
남쪽 세션은 월드 Y 변환을 뒤집어 양쪽 모두 중앙→외곽 순서로 채운다. 중앙선
±0.25 m에 발자국이 걸리는 배치는 payload 모서리가 band에서 0.08 m
떨어진 각 로봇의 staging까지 병렬 이동한다. `pallet_center` 인터락을
얻은 팔만 place에 진입하고, 수직 retreat 후 staging으로 탈출하자마자
잠금을 반납한다. 이후 컨베이어 측면 대기점 복귀는 다음 팔의 place와
병렬로 진행한다.

Yaw 배치는 두 로봇 모두 pick lift 뒤 컨베이어 중심에서 0.55 m 벗어난 측면
위치로 이동해 Pick Zone을 반납하고 컨베이어를 재시작한다. 그 위치에서 목표
quaternion 오차 3° 이하를 10 step 연속 확인한 뒤 place release까지 같은 자세를
유지한다. robot_1은 +90°, 대칭인 robot_2는 -90°를 사용하며 기본값은 OFF다.

2026-08-10에는 격자 환산의 부동소수점 경계 문제를 수정했다. 과거에는
`0.28 / 0.01`이 `28.000000000000004`가 되어 높이를 29칸으로 올렸고, 같은 4호
위에 놓는 박스를 실제 지지면보다 10 mm 띄웠다. 정확한 배수에는 수치 오차
tolerance를 적용해 28칸으로 유지한다. 팔레트 가용 폭의 `floor` 계산도 같은
방식으로 마지막 한 칸을 잃지 않게 했다.

## 로봇 제어와 릴리스 기준

릴리스는 계획 지지면 위 6 mm를 목표로 하고 0~12 mm 창에서 자세를 확인한 뒤
고정 30 step 동안 안정화를 관찰하는 단일 경로만 사용한다. 효과가 없었던
저높이 보정, Z feed-forward, adaptive settle, 재흡착 복구 옵션은 제거했다.

Downloads 정상 동작 레퍼런스 기반 안정 설정:

```text
일반 이동 연속 안정: 5 step
place descend 연속 안정: 25 step
grab settle: 10 step
release settle: 고정 30 step
measurement settle: 0 step
RMP target P gain: 45
joint acceleration limits: [8, 8, 10, 12, 12, 12]
max acceleration norm: 20
```

각 로봇은 자기 HOME 관절값으로 생성한 RMPflow description을
`outputs/rmpflow/`에 쓴다. 운반 중 박스 transform은 비균일 scale과 rigid
transform을 분리해 shear가 생기지 않게 한다.

릴리스 직전 기준:

```text
기준 지지면: 계획 Z가 아닌 실제 팔레트/지지 박스 AABB 상면
목표 gap: 6 mm
허용 gap: 0~12 mm
계획 XY 오차: 15 mm 이하
tilt: 3.5° 이하
10 step 연속 만족
```

릴리스 후에는 조건별 보정이나 조기 판정 없이 30 step(0.5초) 뒤 한 번 채점한다.

릴리스 정착 뒤 기준:

```text
릴리스 순간 대비 XY drift: 5 mm 이하 PASS, 5~25 mm WARNING, 25 mm 초과 FAIL
계획 지지면 높이 오차: 5 mm 이하
tilt: 2.0° 이하
```

기본 정책은 `strict`, 실제 pose occupancy 보정은 OFF, 안정성 PASS 후 고정은
ON이다. `continue-drift`는 support/tilt가 정상인 drift-only 실패만 DEGRADED로
기록하고 계속하는 비교 실험용 옵션이다. 릴리스 자세, 높이 또는 tilt 실패는
기본 정책에서 오류로 종료한다.

## 남은 작업 우선순위

1. 2단 박스의 릴리스 직전 XY 오차가 약 8 mm여도 현재 릴리스 기준 15 mm를
   통과한다. 이를 안정성 기준과 어떻게 맞출지 별도 시뮬레이션으로 검증한다.
2. 기준을 단순히 5 mm로 낮추기 전에 RMPflow가 해당 위치까지 수렴 가능한지,
   실제 미끄러짐이 릴리스 오차·지지 박스 접촉·마찰 중 무엇 때문인지 확인한다.
3. J2 peak torque가 `372.07~372.13 Nm / 372 Nm`로 정격 경계에 찍힌다. 수치
   오차로 경고를 숨기지 말고 실제 하드웨어 적용 전 payload와 토크 여유를 검증한다.
4. 여러 seed에서 2단 적재 drift와 관절 peak/RMS를 수집해 단일 seed 결론을 피한다.

## 알려진 비차단 경고

- 시스템 SciPy 1.8은 NumPy `<1.25`를 요구하지만 현재 NumPy는 1.26.4라 시작 시
  경고가 난다. 현재 비전 실행과 테스트는 통과하지만 장기적으로 의존성 정렬이 필요하다.
- Isaac Sim의 RealSense rigid-body pattern, deprecated extension, CameraInfo 왜곡
  모델 경고는 현재 depth·CameraInfo 발행과 5/5 비전 검증을 막지 않는다.
- `XMLPARSER realpath failed` 메시지는 ROS 2 프로세스 시작 때 보이지만 DDS 연결과
  토픽 통신은 정상이다.

## 변경 시 지켜야 할 원칙

- 마찰·감쇠·질량을 올려 적재 실패를 가리지 않는다.
- XY drift는 5 mm를 넘으면 경고하고 25 mm를 넘으면 기본 `strict` 정책으로
  종료한다. 높이·tilt 실패도 즉시 종료하고 관련 로그로 원인을 확인한다.
- 시뮬레이터, 비전, RViz의 `ROS_DOMAIN_ID`와 RMW를 항상 동일하게 둔다.
- 같은 `--seed` 비교에서 입고 순서가 바뀌지 않도록 색상 난수와 입고 난수를
  계속 분리한다.
