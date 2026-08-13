# Release mode A/B sweep 결과

> 과거 실험 기록. `legacy/corrected` 모드 구분과 관련 CLI·보정 코드는
> 2026-08-11 제거했으며, 현재는 검증된 단일 릴리스 경로만 사용한다.

날짜: 2026-08-10

## 결론

현재 `corrected` 릴리스 보정은 기본으로 켜면 안 된다. 실제 지지면 기준 release gap과 정렬 오차를 줄이고 takt를 약 3.9% 단축했지만, 핵심 지표인 릴리스 후 XY drift를 일관되게 줄이지 못했고 오히려 악화했다. 현재 kinetic attachment/release 구조에서는 `legacy`가 더 안정적이다.

`corrected` 모드는 실험 옵션으로 유지한다. 5 mm는 종료 기준이 아니라 warning으로 유지하고, 10 mm hard fail도 유지하는 것이 타당하다.

## 조건

- 플래너: `baf`
- placer: `default`
- yaw rotation: OFF
- 흡착/운반: 기존 kinetic attachment 유지
- 비교 변수: `--release-mode legacy|corrected`만 변경
- 비전: 실제 ROS2 depth 측정 노드를 seed마다 격리 재시작
- 6박스: seed 40–49, 모드별 10회
- 확장: 20박스, seed 42·49, 모드별 2회
- 공통 판정: drift > 5 mm warning, drift > 10 mm hard fail

## 6박스 결과

| 지표 | legacy | corrected | 변화 |
|---|---:|---:|---:|
| 관측 적재 수 | 59 | 59 | 동일 |
| drift 평균 | 0.931 mm | 2.012 mm | +116% |
| drift 중앙값 | 0.1 mm | 1.3 mm | +1.2 mm |
| drift p95 | 3.32 mm | 6.68 mm | +101% |
| drift 최대 | 7.4 mm | 11.4 mm | +4.0 mm |
| drift > 5 mm | 1/59 | 6/59 | 악화 |
| drift > 10 mm | 0/59 | 1/59 | hard fail 신규 발생 |
| release gap 평균 | 10.392 mm | 1.873 mm | -82% |
| release XY 평균 | 6.027 mm | 1.644 mm | -73% |
| settle 평균 | 30.0 step | 13.1 step | -56% |
| 완료 run 평균 takt | 7.356 s | 7.069 s | -0.287 s (-3.9%) |
| 실행 오류 | 0/10 | 1/10 | corrected seed 44 실패 |

seed 48은 양 모드 모두 6개 투입 중 5개 적재/1개 제거였으므로 동일한 5개만 관측됐다. corrected seed 44는 Cube_06 drift 11.4 mm로 hard fail했다.

## 20박스 확장 결과

| seed | legacy | corrected | 판단 |
|---|---|---|---|
| 42 | Cube_08 drift 13.0 mm hard fail | 같은 Cube_08 drift 29.3 mm hard fail | corrected가 크게 악화 |
| 49 | 12개 안정성 관측 후 place descend timeout | 12개 관측 후 같은 place descend timeout | 공통 로봇 도달성 한계; corrected에 6.2 mm warning 추가 |

seed 49의 timeout은 릴리스 알고리즘보다 후반 배치 위치에 대한 robot_2 도달성 문제다. 두 모드에서 거의 같은 target과 약 0.19–0.20 m 오차로 재현됐다.

## 해석

`corrected`는 놓기 직전의 기하학적 정렬에는 효과가 있다. 그러나 kinetic attachment를 해제하는 순간의 접촉 충격·마찰·지지면 경계 조건을 직접 제어하지 않는다. 낮은 gap으로 맞춘 상태에서 접촉력이나 횡방향 상대 운동이 남으면, 정렬 오차가 작더라도 해제 후 더 크게 미끄러질 수 있다. 이는 로그에 나타난 “작은 release XY + 큰 final drift” 조합과 일치한다.

따라서 현재 알고리즘은 “안정적으로 놓는 보정”이 아니라 “낮고 정확한 위치에서 release하는 보정”으로 보는 것이 정확하다.

## 적용 권고

- 기본값: `--release-mode legacy`
- 실험/튜닝 시에만: `--release-mode corrected`
- 5 mm: warning 유지. legacy 정상 분포의 p95 3.32 mm보다 높고, corrected의 악화 사례를 조기에 표시한다.
- 10 mm: hard fail 유지. 11.4, 13.0, 29.3 mm 사례는 명백한 적재 실패다.
- 다음 보정 우선순위: release 위치를 더 낮추는 것이 아니라, 해제 직전 XY 속도/접촉 안정 조건을 gate로 추가하고 지지영역 내부 여유를 확보하는 방식이 적합하다.

## 검증

- 전체 단위 테스트: 111 passed
- `python3 -m compileall -q src scripts tests`: 통과
- 원본 로그:
  - `/tmp/h2017_release_ab_sweep_isolated`
  - `/tmp/h2017_release_ab_20_isolated`
