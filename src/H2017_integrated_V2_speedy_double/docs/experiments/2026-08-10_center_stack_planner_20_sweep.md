# 지지면 중심 적층 후보 플래너 20박스 스윕

날짜: 2026-08-10

## 구현

- `--stable-tiebreak`: 기존 BAF 점수를 보존하고 동점 후보만 안정성으로 비교.
- `--center-stack-candidates`: free-space 모서리 외에 각 지지 박스 중심 후보 생성.
- `support_edge_margin_ratio`: 상단 박스 가장자리와 실제 지지면 외곽 사이 여유를
  별도 계산해 COM/측면 지지보다 먼저 tie-break.
- 모든 기능은 명시 옵션이며 기존 기본 플래너 동작은 유지한다.

정적 검증: 120 tests passed, compileall 통과.

## 사전 스윕: 중심 후보 없이 COM tie-break/필터

20박스 seed 42에서 relaxed/default/COM 0.10/COM 0.20 네 조건이 모두 기존과 같은
Cube_08 좌표 `(0.882, -1.144)`를 선택했고 drift 13.0 mm로 실패했다. 기존 후보
집합에는 지지면 중심 대안이 없고, 발자국 내부 support metric은 모서리 정렬도
완전 지지로 평가하기 때문이다.

로그: `/tmp/h2017_planner_com_tiebreak_20_sweep/`

## 중심 후보 결과

공통 조건: 20박스, `baf`, legacy release, recovery OFF, stable-tiebreak,
center-stack-candidates, horizontal threshold 0, COM threshold -1.

| seed | 결과 | 종료/완료 지점 | 주요 drift | 적재/제거 |
|---:|---|---|---:|---:|
| 40 | FAIL | Cube_15 | 21.0 mm | 중도 종료 |
| 41 | FAIL | Cube_09 | 15.2 mm | 중도 종료 |
| 42 | COMPLETE | 20개 | Cube_08 9.6 mm, Cube_20 8.4 mm | 15/5 |
| 44 | FAIL | Cube_10 | 13.8 mm | 중도 종료 |
| 47 | COMPLETE | 20개 | hard fail 없음 | 15/5 |

seed 42의 Cube_08은 `(0.882, -1.144)`에서 `(0.912, -1.114)`로 지지면 안쪽
30 mm씩 이동했고 기존 13.0 mm hard fail이 9.6 mm warning으로 낮아졌다.

다중 seed 성공률은 2/5다. 실패한 Cube_15/09/10도 각각 단일 큰 지지 박스의
중심에 놓인 작은 박스였으므로, 모서리 여유만으로 모든 contact-resolution slide를
예측할 수 없다. 중심 후보는 특정 실패를 개선하지만 적재 공간을 분절해 완주한
두 run 모두 5개를 제거했다.

로그:

- seed 42: `/tmp/h2017_center_candidate_seed42_20/run.log`
- seed 40/41/44/47: `/tmp/h2017_center_candidate_multiseed_20_sweep/`

## 결론과 다음 단계

중심 후보 플래너는 기존 Cube_08 실패를 해결했지만 일반 안정성 알고리즘으로는
합격하지 못했다. 기본값으로 켜지 않는다.

다음 실험에서는 geometry score를 더 튜닝하지 않고 release 전후에 지지 박스의
XY 이동도 함께 기록한다. 상단 박스 자체 slide와 하단 지지 박스가 contact impulse로
함께 이동하는 경우를 분리해야 한다. 지지 박스가 움직인다면 상단 배치 위치가 아니라
stack 전체 동역학/접촉 전달을 제어해야 한다.
