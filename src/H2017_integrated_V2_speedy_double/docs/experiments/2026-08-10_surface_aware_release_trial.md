# Surface-aware 저높이 릴리스 1회 검증

실행일: 2026-08-10  
조건: Isaac Sim 5.1, `--box-count 6 --seed 42 --headless`, 실제 비전 노드 사용

## 변경 조건

- 실제 팔레트/계획 지지 박스 AABB 상면을 릴리스 기준으로 사용
- 목표 gap 1.5 mm, 허용 gap 0.5~3.0 mm
- 로봇별 Z 적분 보정을 EMA feed-forward로 학습
- 최소 12 step, 최대 30 step의 pose 속도 기반 adaptive settle
- drift 5 mm 초과 warning, 10 mm 초과 hard fail

## 결과

| box | release gap (mm) | align (step) | settle (step) | drift (mm) | 결과 |
|---|---:|---:|---:|---:|---|
| Cube_01 | 2.0 | 37 | 12 | 0.0 | PASS |
| Cube_02 | 2.0 | 31 | 12 | 0.6 | PASS |
| Cube_03 | 1.9 | 36 | 15 | 1.3 | PASS |
| Cube_04 | 1.7 | 42 | 12 | 1.5 | PASS |
| Cube_05 | 1.5 | 10 | 12 | 1.0 | PASS |
| Cube_06 | 0.8 | 27 | 16 | 7.5 | WARNING |

- 릴리스 gap 평균: 1.65 mm
- align 평균: 30.5 step (0.508 s)
- settle 평균: 13.17 step (0.219 s)
- `align + settle` 평균: 43.67 step (0.728 s)
- 기존 46-pose 평균: align 19 step + settle 30 step = 49 step (0.817 s)
- 순수 릴리스 구간 평균 절감 추정: 5.33 step (0.089 s/box)
- 전체 takt: 평균 7.05 s, 최소 6.52 s, 최대 7.73 s
- 최종: 6/6 hard threshold PASS, Cube_06 quality WARNING

## 판단

저높이 릴리스와 adaptive settle은 동시에 적용해도 릴리스 구간을 느리게 만들지
않았고, 5개 박스는 drift 1.5 mm 이하였다. 그러나 Cube_06은 gap 0.8 mm에서도
7.5 mm 이동했다. 따라서 높은 drop이 유일한 원인이라는 가설은 기각한다.

현재 AABB 방식은 표면까지의 기하 거리만 맞추며, 논문의 force/contact 방식처럼
지지면으로 하중이 전달됐는지 확인하지 않는다. 다음 비교 후보는 다음 둘이다.

1. 동적 rigid body와 gripper 사이의 물리 joint를 유지한 contact handoff 후 joint 해제
2. 5~10 mm warning에만 실행하는 재흡착(regrasp) 기반 XY 보정

첫 번째가 논문의 접촉 후 릴리스와 더 직접적으로 대응한다. 단, 현재 collision-OFF
kinematic attachment를 바꾸는 작업이라 별도 A/B 검증 없이 기본값으로 켜지 않는다.
