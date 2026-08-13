# Release 안정성 개선 후보 검증

날짜: 2026-08-10

## 결론

현재 kinetic attachment 구조에서 수동적인 release 튜닝만으로 legacy보다 좋은
drift와 takt를 동시에 얻지는 못했다. 제품 코드는 검증 전 상태로 복원했고 기본
모드는 계속 `legacy`다.

다음으로 의미 있는 알고리즘은 release 파라미터 추가 튜닝이 아니라
`place → 0.5 s 관측 → drift 경고 시 1회 재흡착/재배치` 폐루프 복구다.

## 후보 1: guarded release

구성:

- 실제 지지면 AABB 사용
- gap 6 mm
- Z feed-forward OFF
- 해제 전 box 속도 2 mm/s 이하 및 tilt rate 1°/s 이하 연속 확인
- legacy와 같은 release 후 30 step 평가

문제가 컸던 seed 40, 41, 42, 44, 47의 30개 적재 비교:

| 지표 | legacy | corrected | guarded |
|---|---:|---:|---:|
| drift 평균 | 1.050 mm | 2.733 mm | 2.153 mm |
| drift 중앙값 | 0.3 mm | 2.0 mm | 1.65 mm |
| drift p95 | 3.255 mm | 7.4 mm | 5.51 mm |
| drift 최대 | 7.4 mm | 11.4 mm | 6.5 mm |
| drift > 5 mm | 1 | 6 | 3 |
| 평균 takt | 7.284 s | 7.005 s | 7.954 s |

해제 전 실제 속도는 평균 0.232 mm/s로 이미 충분히 낮았다. 그런데도 drift가
남았으므로 잔류 속도가 주원인은 아니다. guarded는 corrected의 폭주를 줄였지만
legacy보다 drift와 takt가 모두 나빠 탈락했다.

## 후보 2: stable placer + stability-first

`legacy release + --placer stable --stable-score-first`를 같은 5 seed에 적용했다.

| 지표 | legacy/default | legacy/stable-first |
|---|---:|---:|
| drift 평균 | 1.050 mm | 1.057 mm |
| drift p95 | 3.255 mm | 3.255 mm |
| drift 최대 | 7.4 mm | 7.4 mm |
| drift > 5 mm | 1 | 1 |
| 평균 takt | 7.284 s | 7.288 s |
| 적재/제거 | 30/0 | 30/0 |

6박스에서는 사실상 동일했다. 20박스 seed 42에서도 같은 Cube_08이 같은
13.0 mm drift로 hard fail해 후반 실패를 피하지 못했다.

## 후보 3: collision prewarm

dynamic 전환 전에 carried box collision을 3 step 먼저 켜 contact manifold를
예열했다. kinematic box가 지지면과 상호작용하면서 Cube_06이 3.4 mm 관통했고
release gate가 정상적으로 차단했다. 안전하지 않아 즉시 중단하고 코드를 제거했다.

## 해석

- corrected의 낮은 gap이나 해제 속도만이 문제가 아니다.
- stable planner의 현재 지지율/COM 필터도 같은 크기 박스 적층에서 발생하는
  contact-resolution slide를 구분하지 못한다.
- collision OFF→dynamic+collision ON 전환 뒤 생기는 drift는 placement 후보와
  PhysX 접촉 상태의 결합 결과다.
- 마찰계수만 올리면 시뮬레이션 수치는 좋아질 수 있지만 알고리즘 개선 검증이
  아니며 실제 포장재 계수 근거 없이는 적용하면 안 된다.

## 다음 구현 후보

1. release 후 30 step을 관측한다.
2. drift가 5 mm를 넘고 tilt/support가 정상이며 gripper가 아직 위에 있으면 한 번만
   재흡착한다.
3. 관측된 drift 반대 방향으로 목표를 제한된 거리만큼 보정하고 다시 release한다.
4. 두 번째 결과로 최종 판정하고 재시도 횟수·추가 takt를 기록한다.
5. 동일 seed 10쌍과 20박스 seed 42로 recovery 성공률과 takt 비용을 검증한다.

이 방식은 kinetic 흡착을 유지하면서 실제 결과를 feedback으로 쓰는 첫
폐루프 알고리즘이다. 구현 복잡도와 로봇 동작 변경이 커서 별도 기능 토글로
진행해야 한다.

## 상태

- 검증용 guarded/prewarm 제품 코드 제거 완료
- 기본 `RELEASE_MODE = "legacy"`
- 전체 111 tests passed, compileall 통과
- 로그:
  - `/tmp/h2017_release_guarded_screen`
  - `/tmp/h2017_stable_placer_screen`
  - `/tmp/h2017_release_prewarm_screen`
  - `/tmp/h2017_stable_placer_20`
