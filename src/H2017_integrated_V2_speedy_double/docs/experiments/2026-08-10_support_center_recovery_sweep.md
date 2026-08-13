# Support-center 릴리스 복구 20박스 스윕

> 과거 실험 기록. support-center 복구 구현과 CLI 옵션은 2026-08-11 제거했다.

날짜: 2026-08-10

## 개선 내용

- `support-center` recovery 전략 추가.
- 실제 support AABB와 박스 크기로 안전한 중심 목표를 계산한다.
- 바닥 또는 유효 보정 벡터가 없는 경우 불필요한 재흡착을 생략한다.
- `--release-recovery-max-correction-mm`로 보정 상한을 실험별 지정한다.
- 기존 drift 전략과 마찬가지로 기능 기본값은 OFF다.

## 조건

- 박스 수: 20
- seed: 42
- planner/placer: `baf/default`
- release: `legacy`
- 실제 ROS2 비전 노드를 run별 격리 재시작
- 보정 상한: 15, 20, 30, 40 mm

## 결과

| 최대 보정 | Cube_08 보정 벡터 | 최종 계획 위치 오차 | 결과 |
|---:|---:|---:|---|
| 15 mm | (+11.4, +9.8) mm | 23.2 mm | FAIL |
| 20 mm | (+15.2, +13.0) mm | 29.0 mm | FAIL |
| 30 mm | (+22.7, +19.6) mm | 35.6 mm | FAIL |
| 40 mm | (+30.3, +26.2) mm | 45.6 mm | FAIL |

모든 run은 Cube_08에서 종료됐다. 보정량이 증가할수록 최종 계획 위치 오차가
거의 단조 증가했다. 두 번째 release에서 박스가 원래 계획 위치 쪽으로 되미끄러질
것이라는 가정이 성립하지 않는다.

로그: `/tmp/h2017_support_center_20_sweep/`

## 판단

kinetic 재흡착은 정상이며 recovery 이동과 release gate도 의도대로 작동했다.
실패 원인은 actuator가 아니라 계획 단계에서 큰 지지 박스 모서리에 상단 박스를
배치하는 후보 선택이다. release 이후에 위치를 고치는 방식은 이미 안정된 접촉을
다시 깨고 추가 takt 1.8초 이상을 사용하면서 오차를 키웠다.

다음 개선은 recovery gain 추가 튜닝이 아니라 플래너 단계에서 수행한다.

1. 적층 후보의 support COM margin을 기본 후보 점수에 포함한다.
2. 같은 BAF 점수라면 지지영역 중심 여유가 큰 후보를 먼저 고른다.
3. 20박스 seed 42에서 계획상 Cube_08 위치가 Cube_06 모서리가 아닌 내부로
   이동하는지 플래너 단위 테스트로 먼저 검증한다.
4. 그 뒤 recovery OFF 상태로 20박스 A/B를 수행한다.
