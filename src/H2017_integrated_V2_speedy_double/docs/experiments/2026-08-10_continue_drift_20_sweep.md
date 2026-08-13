# 접근 전환: drift 품질 저하 허용 20박스 스윕

날짜: 2026-08-10

## 문제 재정의

기존 실험은 release 후 XY drift 10 mm를 즉시 안전 실패로 취급했다. 그러나
실패 로그의 박스들은 대부분 tilt 0.1° 안팎, support error 약 -1.6 mm로 실제
지지면에 똑바로 안착해 있었다. 논문 기반 release/재흡착/지지면 중심 보정은
이 숫자를 줄이려다 오히려 접촉을 다시 깨고 적재를 중단시켰다.

새 정책은 drift를 품질 지표와 안전 지표로 분리한다.

- `--stability-policy strict`: 기존 동작. drift >10 mm면 중단.
- `--stability-policy continue-drift`: drift만 초과하고 support/tilt가 정상이면
  `DEGRADED`로 기록하고 계속한다.
- support height 또는 tilt 실패는 정책과 관계없이 즉시 중단한다.
- 기본값은 검증 전이므로 계속 `strict`다.

## 스윕 조건

- 20박스, seed 40/41/42/44/47
- 원본 `baf/default`
- `legacy` release
- recovery, centered candidate, stable placer 모두 OFF
- 실제 ROS2 비전 노드를 run별 격리 재시작

## 결과

| seed | 실행 | 적재/제거 | drift-only degraded | 안전 실패 |
|---:|---|---:|---:|---:|
| 40 | COMPLETE | 17/3 | 2 | 0 |
| 41 | COMPLETE | 15/5 | 1 | 0 |
| 42 | COMPLETE | 15/5 | 1 | 0 |
| 44 | COMPLETE | 17/3 | 1 | 0 |
| 47 | COMPLETE | 15/5 | 1 | 0 |

- 5/5 run, 100/100 투입 완료.
- 총 79개 적재, 21개 제거. 평균 15.8개 적재/20개 투입.
- drift-only degraded 6/79.
- 최대 drift 14.1 mm.
- degraded 최대 tilt 0.18°이며 support/tilt 안전 실패는 0건.
- 기존 strict 정책에서는 seed별 첫 drift hard threshold에서 중단됐다.

로그: `/tmp/h2017_continue_drift_20_sweep/`

## 결론

현재 시스템의 주요 병목은 10 mm drift 자체가 아니라 이를 즉시 전체 실행 실패로
연결한 정책이었다. drift-only 상태를 허용하자 별도 보정 없이 모든 seed가 20개
투입을 완료했고 이후 stack 붕괴도 관측되지 않았다.

이 결과는 10 mm 기준을 삭제한다는 뜻이 아니다. 다음처럼 역할을 바꾼다.

- drift 5 mm: quality warning
- drift 10 mm: degraded placement 및 재계획 트리거
- tilt/support/pallet boundary 위반: safety hard fail

## 다음 구현

현재 물리 장면과 실제 support AABB는 안착 pose를 반영하지만 DeepPack3D 세션의
격자 occupancy는 최초 계획 위치를 유지한다. 다음 단계는 degraded뿐 아니라 모든
안착 박스의 실제 AABB를 격자에 재동기화하고 이후 후보를 다시 계산하는 것이다.
즉 실패 후 박스를 억지로 원위치시키는 대신, 실제 결과를 새로운 세계 상태로
받아들이는 online execution-aware replanning으로 전환한다.
