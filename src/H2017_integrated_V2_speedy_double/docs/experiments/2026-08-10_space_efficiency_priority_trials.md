# 공간 효율 보정 우선순위 실험 (20박스)

> 과거 실험 기록. 실제 운용 지표를 개선하지 못한
> `--expanded-corner-candidates`와 `--catalog-rollout` 구현 및 CLI 옵션은
> 2026-08-11 제거했다. 아래 옵션은 현재 실행할 수 없다.

## 고정된 물리 기준

- `BAF / placer=default / release=legacy / stability=strict`
- dynamic release 뒤 0.5초 안정성 검사
- 검사 PASS 뒤 kinematic 고정 (`freeze-after-stability`)
- 박스 수 20, 기준 시드 `40, 41, 42, 44, 47`
- 기존 기준: `17, 14, 15, 18, 15 = 79/100`, 배치 79개 전부 PASS

## 1. free-space 네 모서리 후보

원본 DeepPack3D의 free-space 좌하단 한 점 외에 나머지 세 모서리 후보를
추가했다.

- 제한 없음: 오프라인 1,000시드에서 16,268 → 17,123 (+5.3%)
- 실제에서는 edge-heavy 적층이 생겼다.
  - seed 40 Cube_19: 아래 4호 중심에서 (-70, -120) mm 치우친 4호 적층
  - drift 13.0 mm hard fail
- 적층 추가 후보를 100% 지지로 제한하면 실제 5시드가 다시 79/100으로 기준과
  같아졌다.

결론: 단독으로 실제 적재량 개선이 없어 이후 코드와 CLI에서 제거했다.

## 2. catalog capacity / exhaustive rollout

현재 후보 뒤에 우체국 1~4호 미래 박스를 가상 배치했다.

- 단순 후보 개수: 총량은 늘지만 대표 시드 회귀가 큼
- depth-3 전수 rollout (`4^3=64`): 오프라인 100시드 +9.6%
- 기대 수용량을 최우선으로 둔 실제 실행은 높은 탑과 로봇에 어려운 자세를
  초반부터 선택해 descend/align 실패가 발생
- `낮은 층 → 지지율 → rollout → BAF`로 안전 우선순위를 바꾸면 seed 44가
  14/20로 기준 18보다 감소

결론: 실제 운용 성능이 회귀해 이후 코드와 CLI에서 제거했다.

## 3. busy 팔레트 대기

기존 scheduler는 idle 로봇 반쪽에 공간이 없으면, busy 로봇 반쪽에 공간이
있어도 앞 박스를 즉시 제거했다.

수정 후에는 busy 세션의 `can_place()`가 참이면 컨베이어를 세우고 cycle 완료를
기다린다. 양쪽 세션 모두 불가능할 때만 제거한다. 반복 확인 중 `place()`의
offered counter가 변하지 않도록 `can_place()` 후 확정한다.

- seed 41 Cube_13에서 실제 대기 후 배정 확인
- 최종 수는 14개로 동일했지만 잘못된 조기 제거를 없애므로 로직은 유지

## 4. 실행 가능한 yaw 회전

플래너에만 있던 90도 회전을 실제 손목 자세로 연결했다.

- pick/grab/lift: 기존 HOME quaternion
- place approach/descend/servo/retreat: world-Z yaw quaternion
- kinetic 흡착 박스가 손목 회전을 따라감
- robot_2는 +90/-90 모두 특정 낮은 위치에서 wrist IK descend가 정지
- 검증된 북쪽 robot_1 세션에만 yaw 후보를 허용

오프라인 robot_1-only yaw 예상:

| seed | yaw OFF | robot_1 yaw |
|---:|---:|---:|
| 40 | 19 | 19 |
| 41 | 14 | 18 |
| 42 | 15 | 18 |
| 44 | 18 | 20 |
| 47 | 17 | 19 |

실제 결과:

| seed | 적재 | 안정성 | 최대 drift | 비고 |
|---:|---:|---:|---:|---|
| 40 | 17/20 | 17/17 PASS | 4.4 mm | Cube_18 3→4호 비전 오분류 |
| 41 | 17/20 | 17/17 PASS | 3.2 mm | 기준 14 대비 +3, 분류 정확 |
| 42 | 무효 | Cube_19 hard fail | 0.2 mm | Cube_11 3→4호 오분류로 지지 높이 +71.2 mm |
| 44 | 18/20 | 18/18 PASS | 5.6 mm | warning 1, 기준과 동일 |
| 47 | 17/20 | 17/17 PASS | 3.0 mm | 기준 15 대비 +2 |

seed 42의 비전 노드는 실제 3호가 Pick Zone에 있을 때 뒤 4호 크기
`[0.3101, 0.3847, 0.2800] m`만 박스 후보로 냈다. 후보 선택 문제가 아니라
앞 박스 클러스터 누락/가림이라 packing 알고리즘 결과에서 제외한다.

## 현재 권장 실행

```bash
scripts/test5_2robot.py \
  --box-count 20 \
  --packing-method baf \
  --placer default \
  --release-mode legacy \
  --stability-policy strict \
  --freeze-after-stability \
  --yaw-rotation
```

실효성이 없었던 두 공간 보정은 현재 코드에서 제거되어 사용할 수 없다.
다음 병목은 packing 점수가 아니라 여러 박스가 카메라 ROI에 있을 때 Pick Zone
앞 박스 클러스터가 누락되는 비전 측정이다.
