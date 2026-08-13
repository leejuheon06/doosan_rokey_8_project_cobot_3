# 릴리스 보정 A/B 스윕 인수인계

작성 시각: 2026-08-10 (KST)

## 현재 상태

- 6박스 A/B 10쌍과 20박스 확장 A/B 2쌍이 모두 종료됐다.
- 최종 보고서: `docs/2026-08-10_release_mode_ab_sweep.md`
- 추가 알고리즘 후보 검증: `docs/2026-08-10_release_algorithm_improvement_screen.md`
  - 6박스 로그: `/tmp/h2017_release_ab_sweep_isolated`
  - 20박스 로그: `/tmp/h2017_release_ab_20_isolated`
- 검증 결론에 따라 기본 release mode를 `legacy`로 되돌렸다. `corrected`는 명시적 실험 옵션이다.
- 검증용 `guarded`와 collision prewarm 코드는 모두 제거했다. 현재 제품 옵션은 다시 `legacy`, `corrected` 두 개뿐이다.
- 마지막 확인 시 A/B/비전/Isaac 잔류 프로세스는 없다.
- 각 seed마다 ROS2 비전 노드를 새로 시작하고, Isaac Sim에 `--exit-on-complete`를 전달한 뒤 비전 노드를 종료한다. 이전 runner의 DDS/프로세스 누적 문제를 피하기 위한 방식이다.
- 현재 프로세스 확인:

```bash
ps -eo pid,ppid,etime,stat,pcpu,pmem,cmd | rg '[h]2017_release_ab_isolated|[t]est5_2robot.py|[c]onveyor_box_measurement/conveyor_box_measurement/node.py'
```

## 코드 변경

- `--release-mode {legacy,corrected}` 토글 추가.
  - `legacy`: 기존 계획 지지면, 6 mm 목표, 0–12 mm window, 고정 30 step 안정화.
  - `corrected`: 실제 지지면 AABB, 지지면 높이 보정/feed-forward, EMA, adaptive settle, 1.5 mm 목표, 0.5–3 mm window.
- 두 모드의 최종 drift warning/hard 기준은 동일하다: warning 5 mm, hard fail 10 mm.
- `--exit-on-complete` 추가: `[적재 안정성 요약]` 출력 직후 종료. 반복 스윕용이며 기본 동작은 유지한다.
- 변경 파일: `src/h2017_palletizing/config.py`, `src/h2017_palletizing/application.py`, `tests/test_stable_planner.py`, `docs/engineering_notes.md`.
- 검증: 전체 111 tests passed; compileall 통과.

## 최종 수치

6박스 59개 관측에서 legacy/corrected drift 평균은 0.931/2.012 mm, p95는 3.32/6.68 mm, 최대는 7.4/11.4 mm였다. 5 mm 초과는 1/59 대 6/59이고 corrected seed 44에서 11.4 mm hard fail이 발생했다.

corrected는 release gap 10.392→1.873 mm, release XY 6.027→1.644 mm, settle 30.0→13.1 step, takt 7.356→7.069 s로 개선했다. 그러나 final drift가 악화되어 안정성 보정으로 채택하지 않는다.

20박스 seed 42는 Cube_08에서 legacy 13.0 mm, corrected 29.3 mm hard fail했다. seed 49는 양쪽 모두 후반 robot_2 place descend 도달 timeout으로 종료했고 corrected에 6.2 mm warning이 추가됐다.

## 후속 개발 후보

### 이미 검증해 탈락한 방법

- 해제 전 속도 gate + 실제 지지면 + 6 mm gap: corrected보다는 나았지만 legacy보다 drift가 크고 takt가 느려 탈락.
- `--placer stable --stable-score-first`: 5-seed 평균/p95/경고 수가 legacy와 같고, 20박스 Cube_08 13.0 mm hard fail도 동일해 탈락.
- kinetic 상태 collision prewarm: 지지면 3.4 mm 관통을 만들어 안전 gate가 차단했으므로 즉시 제거.
- 실제 마찰계수 근거 없이 static/dynamic friction만 올리는 방법은 알고리즘 개선으로 간주하지 않는다.

### 다음 세션 최우선: 1회 폐루프 재배치

목표 동작은 `place → release → 30-step 관측 → drift > 5 mm이면 한 번만 재흡착 → 반대 방향 보정 → 재release → 최종 판정`이다. 기존 kinetic 흡착 방식은 유지한다.

권장 구현 위치:

1. `src/h2017_palletizing/config.py`
   - 기능 토글 `RELEASE_RECOVERY_ENABLED = False` 또는 CLI `--release-recovery` 추가.
   - 최대 재시도 1회, trigger 5 mm, XY 보정 상한 10 mm부터 시작.
2. `src/h2017_palletizing/application.py`
   - `build_cycle_steps()` 안의 release closure와 `pending_stability_checks`에 `attempt`, `owner`, `recovery_target` 기록.
   - 안정성 평가에서 drift만 경고/실패이고 tilt/support는 정상일 때만 recovery 허용.
   - 현재 gripper가 해당 박스 상공에 있고 retreat 전인 `placement_settled` gate에서만 실행.
3. `src/h2017_palletizing/robot.py`
   - 필요하면 현재 gate 자리에 `grab → servo → release → gate` 단계를 한 번 삽입하는 제한된 step 교체 API 추가.
   - broad 상태 초기화나 박스 순간이동은 금지.
4. `src/h2017_palletizing/stability.py`, `tests/`
   - recovery eligibility와 drift 반대 방향 보정 벡터를 순수 함수로 분리해 단위 테스트.

안전 조건:

- 재흡착 가능 거리/자세를 먼저 확인하고, 박스가 기울거나 지지면에서 떨어졌으면 기존 hard fail 유지.
- 보정량은 관측 drift의 반대 방향, 최대 10 mm로 clamp.
- 두 번째 실패는 추가 재시도 없이 종료.
- recovery 횟수와 추가 takt를 별도 로그로 남긴다.

합격 기준:

- 1차: 문제 seed 40, 41, 42, 44, 47에서 hard fail 0, `>5 mm` 1/30 이하, 평균 drift가 legacy 1.050 mm 이하.
- takt: 전체 평균이 legacy 7.284 s 대비 5% 이상 느려지지 않아야 한다. recovery가 발생하지 않은 cycle에는 실질적인 추가 비용이 없어야 한다.
- 2차: seed 40–49 10쌍에서 legacy보다 hard fail/경고 수가 증가하지 않아야 한다.
- 3차: 20박스 seed 42의 기존 Cube_08 13.0 mm hard fail을 recovery 후 10 mm 이하로 낮춰야 한다.

별도 후속으로 seed 49의 `robot_2 place descend` 도달 timeout은 release와 분리해 planner 단계 reachability 검사로 차단한다.

## 주의

- 비전 노드를 외부에서 장시간 재사용하면 Isaac 재기동 뒤 DDS endpoint가 간헐적으로 stale해져 첫 측정 timeout이 발생했다. 반드시 seed별 격리 재시작 방식을 사용한다.
- 이전 `/tmp/h2017_release_ab_sweep` 및 `/tmp/h2017_release_ab_sweep_clean` 로그는 프로세스 누적/ROS 연결 문제로 최종 통계에 사용하지 않는다.
- `--exit-on-complete`는 스윕 전용 보조 옵션이며, 일반 viewer 유지 동작을 바꾸지 않는다.
