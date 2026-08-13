# Kinetic 1회 재흡착 릴리스 복구 검증

> 과거 실험 기록. 효과가 없었던 재흡착 복구 구현과 CLI 옵션은
> 2026-08-11 제거했다.

날짜: 2026-08-10

## 구현

- 기본 OFF인 `--release-recovery` 옵션을 추가했다.
- 최초 release 후 XY drift가 5 mm를 넘고 support/tilt가 정상일 때만 실행한다.
- 현재 박스를 kinetic 방식으로 한 번 재흡착한다.
- 관측 drift 반대 방향으로 gain 1.5를 적용하되 XY 보정은 10 mm로 제한한다.
- 최초 안전 release Z를 유지한 채 재배치하고 두 번째 결과로 종료한다.
- 안정성 관측 중에는 gripper를 박스 위에 유지한다.
- recovery 횟수와 추가 physics step/takt를 별도 출력한다.

## 정적 검증

- `PYTHONPATH=src python3 -m pytest -q -p no:anyio`: 115 passed
- `python3 -m compileall -q src scripts tests`: 통과

## 20박스 seed 42 검증

조건: `legacy`, `baf/default`, 실제 ROS2 비전, headless, 20박스.

- Cube_06: 최초 drift 약 7.4 mm로 recovery 발동.
  - kinetic 재흡착은 정상 동작했다.
  - 최종 계획 위치 오차 7.8 mm, WARNING.
  - recovery 추가 비용 133 step, 2.22 s.
- Cube_08: 최초 drift 약 7.9 mm로 recovery 발동.
  - 반대 방향 10 mm 선보정 후에도 최종 계획 위치 오차 23.9 mm.
  - hard fail로 8개 적재에서 종료.

중간 구현에서는 두 번째 release 순간 대비 drift가 20.8 mm였고, 최초 안전 Z와
보정 목표를 servo 전체에 유지하도록 고친 뒤에도 11.5~12.3 mm였다. 최종 계획
위치 기준으로는 23.9 mm이므로 판정 기준의 문제가 아니라 실제 배치 실패다.

로그: `/tmp/h2017_release_recovery_20/legacy_recovery_seed_42_20.log`

## 결론

kinetic 재흡착 자체는 확실히 동작하지만, 첫 release에서 관측한 drift 방향이
두 번째 release의 slide 방향을 예측하지 못한다. 따라서 단순한 drift 반대 방향
재배치는 20박스 seed 42의 기존 Cube_08 실패를 해결하지 못했고 합격 기준에서
탈락했다. 기능은 검증용 명시 옵션으로만 남기며 기본값은 OFF다.

다음 후보는 과거 drift 벡터가 아니라 현재 지지 박스 AABB의 내부 여유를 직접
계산해 COM을 지지면 안쪽으로 옮기는 recovery target이다. 그 전에는 support box
자체 이동량을 함께 기록해 상자 slide와 지지면 slide를 구분해야 한다.
