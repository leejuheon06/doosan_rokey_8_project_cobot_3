# 안정성 PASS 후 고정 20박스 스윕

날짜: 2026-08-10

## 로직

순서를 다음과 같이 고정했다.

1. 박스를 기존 kinetic attachment에서 release해 dynamic으로 전환한다.
2. 기존과 동일하게 30 physics step(0.5초) 정착시킨다.
3. 기존 strict drift/support/tilt 검사를 그대로 수행한다.
4. PASS한 박스만 속도·각속도를 0으로 만들고 kinematic 지지물로 고정한다.
5. WARNING은 PASS이므로 고정하지만 hard FAIL은 고정하지 않고 즉시 중단한다.

collision은 계속 활성화되어 다음 박스의 물리 지지면으로 동작한다. release 직후
안정성 검사를 우회하지 않으며, 검증 전 박스를 순간이동하거나 고정하지 않는다.

## 조건

- 20박스, seed 40/41/42/44/47
- 원본 `baf/default`, `legacy` release
- strict stability policy
- recovery/centered candidate/pose reconcile OFF
- 실제 ROS2 비전 노드, run별 격리

## 결과

| seed | 실행 | 적재/제거 | stability | warning/hard fail |
|---:|---|---:|---:|---:|
| 40 | COMPLETE | 17/3 | 17/17 PASS | 0/0 |
| 41 | COMPLETE | 14/6 | 14/14 PASS | 0/0 |
| 42 | COMPLETE | 15/5 | 15/15 PASS | 0/0 |
| 44 | COMPLETE | 18/2 | 18/18 PASS | 1/0 |
| 47 | COMPLETE | 15/5 | 15/15 PASS | 0/0 |

- 5/5 run, 100/100 투입 완료.
- 총 79개 적재, 21개 제거.
- 79/79 stability PASS.
- 최대 drift 5.9 mm, hard fail 0.
- seed 42 Cube_08 drift는 기존 13.1 mm에서 1.5 mm로 감소.
- seed 42 Cube_06 drift는 기존 7.4 mm에서 0.0 mm로 감소.

고정 OFF + continue-drift 비교에서는 같은 79개 적재 중 degraded가 6건이고 최대
drift가 14.1 mm였다. 따라서 문제의 주원인은 새 박스 자체의 release보다 이미
안착한 동적 지지 박스가 후속 contact impulse에 다시 움직이는 연쇄 효과였다.

## 기본값

5-seed 20박스 검증 결과에 따라 `FREEZE_AFTER_STABILITY = True`로 승격했다.
비교/순수 동역학 실험은 `--no-freeze-after-stability`로 끌 수 있다.

로그:

- seed 42: `/tmp/h2017_freeze_after_pass_seed42_20/run.log`
- seed 40/41/44/47: `/tmp/h2017_freeze_after_pass_multiseed_20_sweep/`

## Pose reconcile 실험

실제 XY를 1 cm occupancy에 직접 재구성하는 방식은 seed 42에서는 완주했으나,
seed 41에서 Cube_18이 281.6 mm 낙하하고 81.53° 기울어 실제 붕괴했다. 격자
반올림이 물리 free-space를 과대평가할 수 있으므로 기본 OFF를 유지한다.
