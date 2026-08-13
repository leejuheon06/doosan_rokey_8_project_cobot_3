# H2017 Integrated V2 Speedy Double

두 대의 H2017 로봇이 각자 전용 컨베이어와 깊이 카메라로 들어오는 택배 박스를
측정하고, DeepPack3D 온라인 계획에 따라 하나의 팔레트 양쪽에 적재하는 Isaac Sim
프로젝트다. `reference3`의 2라인 셀(`h2017_gripper_v8.usd`)을 실행 자산으로 사용한다.
로봇·팔레트·컨베이어·카메라 자산은 프로젝트 내부의 로컬 경로를 사용한다.

## 현재 검증 상태

- 2라인 통합 스모크(`--box-count 2 --seed 42 --headless`): 라인별 1개씩 투입,
  비전 검출·분류 `2/2`, 적재 `2/2`, 안정성 `2/2 PASS`
- 컨베이어 `1.2 m/s` GUI 스모크: 양 라인 Pick Zone `x=0.152~0.153 m` 정지,
  오버슈트 없이 `2/2 PASS`; 전체 물리 경과 `24.4초 → 22.3초`
- 중앙선 우선 배치를 유지하면서 양쪽 팔이 중앙 band 밖 staging까지
  병렬 이동하고, 잠금을 얻은 팔만 중앙에 진입함
- RMP target P gain `35 → 45`, 중앙 탈출 즉시 잠금 반납 후 GUI 스모크:
  로봇 사이클 `9.90/12.10초`, 전체 물리 경과 `22.3초 → 18.3초`,
  적재 안정성 `2/2 PASS`
- 단위 테스트: 시뮬레이터 순수 로직 `136 passed`, 비전 패키지 `11 passed`

## 실행

비전 노드, RViz, Isaac Sim을 함께 실행하는 명령과 공통 ROS 설정은
[통합 실행 명령 가이드](docs/test5_run_guide.md)에 한곳으로 정리되어 있다.

먼저 두 비전 노드를 실행한다.

```bash
export SPEEDY_DOUBLE_ROOT="$PWD"  # 저장소 루트에서 실행
cd "$SPEEDY_DOUBLE_ROOT/conveyor_box_measurement_double"
./run_double_vision.sh
```

그다음 별도 터미널에서 시뮬레이터를 시작한다. 전체 순서는 통합 실행 가이드를
따른다.

```bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SPEEDY_DOUBLE_ROOT="$PWD"  # 저장소 루트에서 실행
export ISAAC_SIM_PYTHON="<Isaac Sim 설치 디렉터리>/python.sh"
cd "$SPEEDY_DOUBLE_ROOT/H2017_integrated_V2_speedy_double"
"$ISAAC_SIM_PYTHON" \
  scripts/test5_2robot.py --box-count 2 --seed 42
```

## 기준 문서

- [통합 실행 명령 가이드](docs/test5_run_guide.md): 실행, RViz, 토픽 점검, 종료, 테스트
- [아키텍처](docs/architecture.md): 프로세스, 데이터 흐름, 모듈과 좌표계
- [기술 상태](docs/engineering_notes.md): 현재 설정 근거, 최근 수정, 알려진 문제
- [수정사항·트러블슈팅 워크북](docs/H2017_수정사항_트러블슈팅_2026-08-10.xlsx): 발표용 요약과 전후 수치

## 코드 구조

| 경로 | 역할 |
|---|---|
| `assets/Collected_h2017_gripper/` | H2017 로봇 셀과 창고 루트 USD |
| `assets/doosan-robot2/` | H2017 URDF와 로봇·VG10 USD |
| `assets/omni_assets/` | 팔레트·컨베이어·카메라 로컬 자산 |
| `config/rmpflow/` | H2017 RMPflow 설정 |
| `scripts/test5_2robot.py` | Isaac Sim 시작과 애플리케이션 진입 |
| `src/h2017_palletizing/application.py` | 온라인 입고와 두 로봇 실행 조정 |
| `src/h2017_palletizing/intake.py` | 비전 측정 상태기계와 재시도 |
| `src/h2017_palletizing/planning.py` | DeepPack3D 온라인 배치 |
| `src/h2017_palletizing/robot.py` | 로봇 상태기계와 박스 부착·릴리스 |
| `src/h2017_palletizing/stability.py` | 릴리스 및 적재 안정성 판정 |
| `src/h2017_palletizing/scene.py` | USD 장면, 박스 물리와 재질 |
| `src/h2017_palletizing/conveyor.py` | 컨베이어, 스폰, Pick Zone |
| `src/h2017_palletizing/ros_bridge.py` | 계획·트리거 발행과 검출 구독 |
| `config/vision_debug.rviz` | 비전 오버레이·점군·마커 RViz 설정 |
| `docs/experiments/` | 날짜별 실험 및 안정성 검증 기록 |

두 비전 노드는 저장소 루트의 인접 패키지
`conveyor_box_measurement_double/`에 있다.
