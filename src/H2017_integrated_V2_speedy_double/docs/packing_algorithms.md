# 팔레타이징 패킹 알고리즘 정리

`src/h2017_palletizing/planning.py`가 쓰는 알고리즘이 무엇이고, 안정성 필터가
그 위에 어떻게 얹혀 있는지 정리한다.

전체 구조는 두 층이다.

| 층 | 역할 | 질문 |
|---|---|---|
| DeepPack3D 4종 휴리스틱 | **최적화** | 어디에 놓으면 잘 채워지나? |
| 안정성 필터 (`--placer stable`) | **제약** | 그 자리가 물리적으로 버티나? |

목적이 직교하기 때문에 서로 간섭하지 않고 조합할 수 있다.

---

## 1. 아래층: DeepPack3D 구성적 휴리스틱

이름은 딥러닝처럼 들리지만 **이 레포에 있는 건 강화학습이 아니다.**
`planning.py` 첫 주석에 "without importing TensorFlow"라고 명시돼 있다.
실제로 도는 것은 고전적인 구성적 휴리스틱(constructive heuristic)이다.

원본: https://github.com/SoftwareImpacts/SIMPAC-2024-311

### 1.1 Maximal free space partitioning

팔레트를 격자로 보고, 빈 공간을 "최대 직육면체(maximal cuboid)" 목록으로
관리한다. 박스를 하나 놓을 때마다:

1. 모든 빈 공간에서 새 박스가 차지한 부피를 뺀다 (`GridCuboid.split`)
2. 쪼개진 조각 중 다른 조각에 완전히 포함되는 것은 버린다 (`_SpacePartitioner.add`)

남은 것이 "더 이상 키울 수 없는" 빈 공간들이다. 중복 후보를 만들지 않으면서
가능한 배치를 빠짐없이 훑기 위한 표현이다.

top-down 흡착 그리퍼라 위가 막힌 공간에는 넣을 수 없으므로, 후보는
**천장까지 열려 있는 공간**만 쓴다 (`_candidates`의 `free_space.top != max_grid_height` 검사).

### 1.2 Height map

각 격자 칸의 현재 높이를 2D 배열(`height_map`)로 들고 있다. 새 박스는 자기
발자국 안의 **최대 높이** 위에 얹힌다. 3D 충돌 검사를 매번 하지 않고 2D 배열
조회로 끝내려는 것이다.

### 1.3 4종 선택 규칙

후보가 여러 개일 때 무엇을 고를지의 차이일 뿐이다. 2D 빈 패킹 문헌의 이름을
그대로 가져왔다 (`_score`).

| 옵션 | 이름 | 기준 |
|---|---|---|
| `bl` | Bottom-Left | 최대한 낮게, 그다음 왼쪽/앞쪽 |
| `baf` | Best Area Fit | 가장 작은 빈 공간부터 채움 |
| `bssf` | Best Short Side Fit | 남는 변 중 짧은 쪽이 최소 |
| `blsf` | Best Long Side Fit | 남는 변 중 긴 쪽이 최소 |

넷 다 **그리디**다. 매 스텝 지금 최선만 고르고 되돌아가지 않는다.
실기에서는 `lookahead=1`이라 한 수 앞도 보지 않는다 — 컨베이어에서 다음에
무엇이 올지 미리 알 수 없기 때문이다.

### 1.4 기존 안정성 개념: `min_support_ratio`

원본에도 안정성 검사가 하나 있다. "발자국의 절반 넘게 받쳐지는가":

```python
support_ratio = np.count_nonzero(footprint == z) / footprint.size
if support_ratio <= self.min_support_ratio:   # 기본 0.5
    continue
```

**면적만 본다.** 이것이 다음 절의 출발점이다.

---

## 2. 위층: 안정성 필터

특별한 이름이 붙은 알고리즘이 아니라, 로보틱스에서 표준인
**지지 다각형(support polygon) 기반 정적 안정성 판정**이다.

물체가 넘어지지 않을 조건은 고등학교 물리 그대로다:

> 무게중심을 바닥에 수직으로 투영한 점이 지지 영역 안에 들어와야 한다.

### 2.1 왜 면적만으로는 부족한가

`min_support_ratio`가 놓치는 경우:

```
        ┌───────────────────┐   ← 위 박스 (폭 1.0)
        │        ●          │   ● = 무게중심
  ┌─────┴─────┐             │
  │  아래 박스 │  ← 60% 받쳐짐, 0.5 문턱 통과
  └───────────┘
                └── 무게중심이 지지 경계 밖 → 실물은 넘어진다
```

60%가 받쳐져도 그 60%가 한쪽에 몰려 있으면 넘어간다. 격자 위에서는 합법인데
실물은 쓰러지는 배치가 이렇게 나온다.

### 2.2 두 개의 직교하는 축

안정성은 서로 다른 두 가지를 본다. **같은 축을 두 번 재면 한쪽이 죽는다** —
실제로 그 버그를 냈다가 §2.4에서 고쳤다.

| 축 | 함수 | 무엇을 재나 |
|---|---|---|
| 바닥 지지 | `support_metrics()` | 아래에서 받쳐주나 |
| 측면 지지 | `lateral_support_ratio()` | 옆에서 받쳐주나 |

#### `support_metrics(height_map, cuboid) -> (지지 비율, 무게중심 여유)`

지지 영역 = 발자국 안에서 윗면 높이가 정확히 `cuboid.z`인 칸들.

- **지지 비율** = 지지 칸 수 / 발자국 칸 수. **얼마나** 받쳐지나 (면적)
- **무게중심 여유** = 박스 중심 → 지지영역 bbox 경계 최소거리 ÷ **지지영역의
  짧은 변**. **어디가** 받쳐지나 (위치)

무게중심 여유 읽는 법 — `0.5` 지지영역 한가운데, `0.0` 경계에 정확히 걸침
(넘어가기 직전), 음수면 지지영역 밖.

정규화 분모가 박스가 아니라 지지영역인 것은 reference2 구현(정완지표를 만든
코드)과 임계값을 호환시키기 위해서다. 그쪽 실측 튜닝 결과를 그대로 쓸 수 있다.

칸 인덱스를 반 칸씩 쓰는 이산 좌표 대신 칸이 덮는 연속 구간 `[i, i+1)`로
계산한다. 그래야 "경계에 정확히 걸림"이 정확히 `0.0`이 된다.

#### `lateral_support_ratio(occupied, cuboid) -> 비율`

박스의 **-x, -y 옆면**이 팔레트 벽이나 이웃 박스와 맞닿는 면적 비율이다.
바닥 지지와 완전히 독립이다 — 옆에서 받쳐주면 같은 바닥 지지율이어도 훨씬
안 넘어진다. 교차적재(interlocking)가 안정성을 높이는 원리가 이것이다.

`+x, +y` 면은 검사하지 않는다. 논문처럼 랩이나 외곽 지지가 있다고 보기도
하고, 뒤에 올 박스가 받쳐줄 수도 있어 지금 시점에 판정할 수 없기 때문이다.

### 2.3 `StableDeepPack3DPlanner`

두 지표로 후보를 거른다. 바닥층(`z == 0`)은 팔레트가 발자국 전체를 받치므로
무게중심 검사에서 제외한다 — height map이 0이라 지지 칸 판정이 무의미하다.

```python
lateral = lateral_support_ratio(partitioner.occupied, candidate.cuboid)
if lateral < self.min_horizontal_support_ratio:
    continue
if candidate.cuboid.z > 0:
    _, com_margin = support_metrics(partitioner.height_map, candidate.cuboid)
    if com_margin < self.min_com_margin_ratio:
        continue
```

후보를 통과한 뒤에는 선택한 bl/baf/bssf/blsf 휴리스틱 점수를 그대로 사용한다.
안정성 점수를 휴리스틱보다 앞세우던 실험 옵션은 장기 물리 시험에서 개선이 없어
운영 코드에서 제거했다.

### 2.4 고쳐 쓴 이력 — 죽은 파라미터였다

처음 구현에서 `min_horizontal_support_ratio`를 **바닥 지지 면적 비율**로
해석했다. 그런데 그건 부모 클래스의 `min_support_ratio`(기본 0.5)와 같은
축이라, 0.3 같은 값은 이미 0.5에 걸러진 뒤라서 **아무 일도 하지 않았다.**

정완지표는 같은 0.3으로 뚜렷한 효과를 냈는데 재현이 안 돼서 원본
(`reference2/deeppack3d_planner.py:748`)을 열어보고 알았다:

```python
def _horizontal_support_ratio(self, partitioner, cuboid) -> float:
    """cuboid의 -x, -y 면이 팔레트 벽/이웃 박스와 접촉하는 면적 비율(0~1)."""
```

**같은 이름에 다른 물리량이 들어 있었다.** 논문(arXiv:2307.11531)이 말하는
"수평 지지"는 수평면 지지가 아니라 **측면** 지지였다. 지금은 그 정의를 따른다.

측정으로도 확인된다 — 고치기 전 0.3/0.05는 배치를 하나도 안 바꿨고
(146→146, 160→161), 고친 뒤에는 확실히 문다 (146→139, 160→144).

## 3. `stability.py`와 헷갈리지 말 것

이름이 비슷하지만 완전히 다른 물건이다.

| | 시점 | 입력 | 하는 일 |
|---|---|---|---|
| `planning.py`의 안정성 필터 | 배치를 **정하기 전** | 격자 height map | 후보를 거른다 |
| `stability.py`의 `assess_box_pose` | 박스를 **놓은 뒤** | USD/PhysX 실측 pose | drift·tilt 측정, 합격 판정 |

전자는 예방, 후자는 검증이다. 둘 다 필요하고 서로를 대체하지 않는다.

---

## 4. 사용법

```bash
--placer default   # 원본 DeepPack3D 그대로 (기본값)
--placer stable    # 측면 지지 + 무게중심 필터 추가

--stable-min-horizontal-support 0.3   # 측면 지지 (기본 0.3)
--stable-min-com-margin 0.05          # 무게중심 여유 (기본 0.05)
```

실행 예시는 `docs/test5_run_guide.md` 참조.

### 4.1 임계값 튜닝 (정완지표 조건)

전체 팔레트 1.003×0.971, **6종 카탈로그 + ±2 cm jitter**, seed 7/13/21/29/37,
회당 60개 투입. 각 칸은 5시드 합계 배치 수 / 평균 체적효율.

`risky` = 측면 지지 < 0.3 이거나 상단층 무게중심 여유 < 0.1 인 배치 수 (총합).

| 설정 | bl | baf | bssf | blsf | risky |
|---|---:|---:|---:|---:|---:|
| default | 146 / 46.4% | 160 / 57.0% | 152 / 58.5% | 135 / 48.5% | **64** |
| stable 0.3/0.05 | 139 / 48.4% | 144 / 53.8% | 146 / 56.8% | 129 / 44.9% | 11 |
| stable 0.5/0.05 | 105 / 41.8% | 128 / 41.8% | 122 / 47.9% | 111 / 42.2% | 6 |
| stable 0.3/0.15 | 124 / 44.8% | 121 / 51.3% | 148 / 54.7% | 131 / 42.5% | **0** |
| stable 0.5/0.15 | 109 / 42.0% | 122 / 40.4% | 123 / 45.8% | 118 / 41.1% | 0 |
| stable 0.3/0.05 +점수 | 118 / 45.6% | 145 / 51.7% | 144 / 50.3% | 127 / 52.1% | **2** |
| stable 0.5/0.15 +점수 | 120 / 40.9% | 106 / 39.3% | 119 / 40.3% | 108 / 40.1% | 0 |

**기본값을 0.3 / 0.05로 정한 근거:**

- 위험 배치를 **64 → 11 (83% 감소)** 시키면서 배치 손실이 5~10%로 가장 싸다
- 정완지표가 Isaac 물리로 실측 검증한 바로 그 설정이다
- 더 조이면(0.5, 0.15) 위험은 0이 되지만 `baf`가 24% 깎인다

표의 `+점수` 행은 제거된 과거 실험 결과다. 오프라인 위험도는 줄었지만 실제
장기 물리 시험에서 drift와 완주율이 개선되지 않아 운영 옵션으로 채택하지 않았다.

### 4.2 물리 검증 (정완지표 Isaac 로그, 단일 매니퓰레이터)

`wrist_fixed`, box60, 4종 × 5시드 = 20런씩. 종료 원인별로 분류한 결과다.

| | default | stable |
|---|---:|---:|
| 붕괴 런 | **6/20** | **1/20** |
| 완주 런 | 9/20 | **18/20** |
| 도달 실패 런 | 5 | 1 |
| 총 적재 | 206 | 240 |

**적재량은 오히려 늘면서 붕괴가 6분의 1로 준다.** `bssf/default`는 5런 중
4런이 붕괴했다 — 이 조합은 쓰면 안 된다.

주의: 같은 자료의 box40 표에서는 모든 조합이 `toppled=0`이다. 팔레트가 덜
차면 차이가 안 드러난다. **박스를 충분히 넣어야 보이는 효과다.**

---

## 4.5 Lookahead (beam) — 측정 결과 도움이 안 된다

`BeamPackingSession`은 다음 박스 k-1개의 치수를 **배치 위치 평가에만** 쓴다.
순서는 절대 바꾸지 않는다 — 벨트 위 뒤쪽 박스를 먼저 집을 수 없기 때문이다.

> **주의:** 플래너의 `lookahead=k`와 혼동하면 안 된다. 그쪽은 k개 중 점수가
> 제일 좋은 것을 **먼저 놓는다.** 원본 벤치마크에서는 정당하지만 우리 설비는
> 실행할 수 없다. reference `test1.py:1046`의 주석은 "평가에만 쓴다"고 적혀
> 있으나 넘겨받는 플래너는 `pending.pop(selected.pending_index)`로 순서를
> 바꾼다 — 주석과 구현이 어긋나 있다.

### 측정: lookahead는 손해다

seed 1~10, 회당 40개 투입, `beam_width=4`. 숫자는 회당 평균 배치 개수.

| method | k=1 | k=2 | k=3 | k=4 | k=5 | k=8 |
|---|---:|---:|---:|---:|---:|---:|
| bl | **19.6** | 16.6 | 13.5 | 13.6 | 12.6 | 12.2 |
| baf | **18.8** | 15.4 | 13.6 | 13.3 | 12.0 | 11.9 |
| bssf | 14.8 | **16.0** | 14.8 | 14.7 | 12.4 | 12.9 |
| blsf | **17.7** | 16.7 | 16.7 | 15.9 | 12.8 | 12.1 |

`beam_width`도 같은 방향이다 (k=3 고정): W=1 → 19.6, W=2 → 17.1,
W=4 → 13.5. **더 넓게 볼수록 나빠진다.**

롤아웃 목적함수를 다섯 가지로 바꿔가며 재봐도 결론이 안 바뀐다:

| 목적함수 | bl | baf | bssf | blsf |
|---|---:|---:|---:|---:|
| `height_map.sum()` | −3.0 | −3.4 | +1.2 | −1.0 |
| 없음 (거부권만) | −3.0 | −2.5 | −0.3 | −3.2 |
| 최대 높이 | −3.1 | −2.6 | +1.0 | −2.9 |
| 최대 높이 + 평탄도 | −3.7 | −2.7 | +0.8 | −2.4 |
| 봉투 대비 낭비 부피 | −3.1 | −2.6 | +1.0 | −2.9 |

"거부권만" 행이 중요하다. 이건 **롤아웃이 더 많이 받아낼 때만** greedy와
다르게 고르는, 이론상 손해볼 수 없어 보이는 설정이다. 그런데도 진다.

즉 **"다음 k-1개가 들어가는 자리"를 고르는 것 자체가 장기적으로 나쁘다.**
당장 두 개를 받으려고 현재 박스를 빡빡하게 놓으면 바닥이 깨진다. 40개
지평선에서는 바닥부터 채우는 bl/baf의 근시안이 더 낫다.

`bssf`만 +1이지만, beam을 켠 `bssf`(16.0)도 그냥 `bl`(19.6)에 크게 진다.

### 분석 중 잡은 자기 버그

첫 목적함수 `height_map.sum()`은 중립적인 tie-break가 아니었다. 이미 높은
지점 위에 쌓으면 합 증가량이 작아서 **탑을 세우도록 유도한다.** bl/seed 4에서
greedy 17개 vs beam 6개(4층 탑, top 77/80)로 갈렸다. 목적함수를 고쳐도 위
표대로 결론은 그대로였다.

### 결론

**k=1을 유지한다.** 이 측정이 서 있는 한, lookahead를 위한 비전/카메라 작업
(다중 검출 발행, 카메라 중앙 이동, 이동 중 측정, 순서 대응)은 근거가 없다.

`BeamPackingSession`은 코드에 남겨 두되 CLI에는 노출하지 않는다. 기본이 꺼져
있고(`open_session()`에 `beam_width`를 주지 않으면 기존 세션), 나중에 아이템
분포나 팔레트 크기가 바뀌었을 때 다시 재보는 계측 도구로 쓴다.

**이 결론이 뒤집힐 수 있는 조건:** 여기서 시험한 것은 "현재 박스를 고정하고
나머지는 그리디로 굴리는" 롤아웃 한 계열뿐이다. bound가 있는 진짜
branch-and-bound, 학습된 가치함수(DeepPack3D의 RL 쪽), 또는 4종보다 다양한
박스 분포에서는 다를 수 있다.

## 4.6 Yaw 회전 — 가장 큰 미사용 레버

`--yaw-rotation`. 박스를 Z축으로 90도 돌려 놓는 선택지다. 플래너는 처음부터
`allow_yaw_rotation`을 지원했지만 계속 `False`로 고정돼 있었다.

반쪽 팔레트는 **1.003 x 0.485 m**로 가늘고 길다. 회전 없이는 큰 박스(0.41 x
0.31)의 긴 변이 깊이 방향에 들어가지 않는다. 90도 돌리면 들어간다.

seed 1~10, 회당 40개, `--placer stable 0.3/0.05`:

| method | yaw | 배치 | util% | 위험률 | 회전 배치 | 최악 시드 |
|---|---|---:|---:|---:|---:|---:|
| bl | OFF | 148 | 46.2 | 0.00% | 0 | 9 |
| bl | ON | 143 | 55.0 | 3.81% | 121 | 11 |
| baf | OFF | 143 | 44.5 | 0.92% | 0 | 9 |
| baf | ON | 148 | 55.0 | 1.72% | 107 | 11 |
| **bssf** | OFF | 109 | 43.9 | 0.00% | 0 | 7 |
| **bssf** | **ON** | **169** | **62.4** | 0.73% | 103 | **14** |
| blsf | OFF | 142 | 43.1 | 0.91% | 0 | 9 |
| blsf | ON | 132 | 47.3 | 1.01% | 62 | 7 |

**`bssf` + yaw가 현재 최선(`bl` OFF)보다 배치 +14%, 체적효율 +16 pp다.**
위험률은 0.00% → 0.73%로 사실상 그대로고, 최악 시드가 7 → 14로 두 배가 되어
분산까지 줄어든다.

### 켜면 안 되는 이유 (아직은)

`robot.py`의 이동은 전부 `self.home_ee_orientation`으로 고정돼 있다
(`_drive_to` 호출 4곳). **회전된 배치를 실행할 수단이 없다.** 켜면 계획과
실제 배치가 조용히 어긋나므로 기본은 꺼 두고, 켤 때 경고를 찍는다.

필요한 작업은 place descend 직전에 손목을 Z축으로 90도 돌린 목표 자세를
넘기는 것이다. 6축 팔이라 마지막 관절 하나면 되고 RMPflow가 목표 orientation을
받는다. 정완지표의 `--wrist-mode free`와는 다르다 — 그건 자세를 **풀어주는**
것이었고(그래서 전도가 늘어 fixed 권장), 이건 정해진 90도로 **명시 제어**다.

### 검토했지만 기각한 대안들

같은 조건(seed 1~30, 회당 40개)에서 적용 가능한 레버를 전부 재봤다. 두 가지
위험을 함께 채점한다 — **전도 위험**(놓은 뒤 넘어지나: 상단층 중 COM 여유<0.1
또는 측면 지지<0.3)과 **충돌 위험**(놓는 도중 옆 박스를 치나: 배치 오차를
주입해 3D 겹침을 센다).

| 후보 | 최선 | 배치 | 최악 | util% | 전도위험 | 충돌 σ5 | 충돌 σ8 |
|---|---|---:|---:|---:|---:|---:|---:|
| 현재 기본값 | bl | 14.7 | 8 | 42.9 | 0.30% | 0.2% | 4.4% |
| **yaw ON** | **bssf** | **17.1** | **12** | **57.9** | 0.73% | **0.2%** | **4.1%** |
| min_support 0.5→0.3 | bl | 15.6 | 9 | 43.7 | 0.27% | 0.2% | 4.5% |
| yaw + min_support 0.3 | bssf | 16.6 | 11 | 57.6 | 1.01% | 0.3% | 4.1% |
| yaw + resolution 0.005 | bssf | 17.0 | 12 | 57.4 | 1.47% | 1.8% | 9.3% |
| box_gap 15→10 mm | bl | 17.2 | 9 | 48.8 | 0.25% | **10.0%** | 22.7% |

**`box_gap` 축소는 기각.** 배치는 17.2로 가장 많지만 충돌 위험이
0.2% → **10.0%로 50배**가 된다. `DEEPPACK3D_BOX_GAP_M = 0.015`는
`PLACE_RELEASE_MAX_HORIZONTAL_ERROR_M = 0.015`와 정확히 같은 값이다 —
넉넉하게 잡아둔 패딩이 아니라 릴리스 허용오차에 맞춰 제대로 사이징된 값이다.

**`resolution` 세분화도 기각.** 0.01 → 0.005로 줄이면 배치는 그대로인데
충돌 위험이 0.2% → 1.8%로 오른다. `_grid_item_size`의 `ceil`이 0.01 격자에서는
box_gap을 한 칸 위로 올림해 여유를 더 줬는데, 격자가 촘촘해지면 그 반올림
여유가 사라진다. **안전 마진이 조용히 깎이는 변경**이라 위험하다.

**`min_support_ratio` 0.5 → 0.3은 남겨둘 만하다.** 배치 +6%, 전도 위험은
오히려 미세하게 낮고(0.30% → 0.27%), 충돌 위험 동일, **로봇 수정이 전혀 필요
없다.** 다만 바닥 지지 30%짜리 배치를 허용하는 것이라 물리 검증이 필요하다 —
기하 지표만으로는 판정할 수 없는 영역이다. §6.1 [1] 논문이 "polygon 기반이
partial-base보다 낫다"고 한 것과 같은 방향이긴 하다.

### 버퍼는 답이 아니었다

순서 제약을 물리적으로 푸는 방법(팔레트 옆 임시 거치대)도 재봤다. 손에 든
것 + 버퍼 b칸 중 아무거나 고를 수 있게 한 설정이다.

| method | 버퍼0 (현재) | 버퍼1 | 버퍼2 | 버퍼3 |
|---|---:|---:|---:|---:|
| bl | 14.8 | 15.3 | 16.1 | 15.3 |
| baf | 14.3 | 12.5 | 10.8 | 10.3 |
| bssf | 10.9 | 11.5 | 10.5 | 9.7 |
| blsf | 14.2 | 12.3 | 11.7 | 9.8 |

`bl`만 조금 오르고 나머지는 나빠진다. §4.5의 "선택권이 생겨도 그리디가 잘
못 쓴다"와 같은 현상이다. **하드웨어를 추가할 근거가 없다.**

### 2로봇 배정 규칙 변경도 효과 없음

현재는 `rank_idle_unit_indices`가 배치 수가 적은 로봇에게 준다(부하 분산).
"그 박스를 더 잘 받는 반쪽에게 준다"(적합도 우선)로 바꿔봤다.

seed 1~30, 회당 80개, 4종 × yaw ON/OFF 전 조합에서 **차이가 정확히 0이었다.**
현재 코드가 이미 첫 로봇이 못 놓으면 다른 로봇으로 넘기기 때문에, 적합도
우선은 그 폴백이 하는 일을 앞당길 뿐이다.

## 5. 지금 못 하는 것

### 하중 기반 규칙

`config.py`의 `BOX_CATALOG`에 치수만 있고 질량이 없다. "무거운 박스를 아래로"
같은 load-bearing 규칙은 카탈로그에 질량 필드를 추가하기 전에는 불가능하다.
현재 얹혀 있는 것은 **순수 기하 안정성**이다.

### Branch-and-bound (`--placer bnb`)

reference의 `test1.py`에는 `--placer bnb` 옵션과
`--bnb-branch-factor` / `--bnb-lookahead-depth` 인자가 정의돼 있으나,
**구현(`StableDeepPack3DPlanner`, `BranchAndBoundStablePlanner`)이 어디에도
없다.** reference는 CLI 규격서지 동작하는 코드가 아니다.

bnb는 다음 박스를 미리 봐야 하는데 현재 파이프라인은 `lookahead=1`이고
컨베이어 입고 순서를 사전에 알 수 없다. 하려면 비전 선행 측정으로 다음 박스
치수를 확보하는 작업이 먼저다.

---

## 6. 참고 문헌

### 6.1 우리 구현의 직접 근거

**[1] Static stability versus packing efficiency in online three-dimensional
packing problems: A new approach and a computational study**
*Computers & Operations Research*, Vol. 178 (2025).
https://doi.org/10.1016/j.cor.2025.106974

가장 가까운 논문이다. 온라인 3D 패킹 휴리스틱에 정적 안정성 제약을 붙였을 때
적재율이 얼마나 손해 보는지를 세 가지 안정성 정의로 나눠 측정했다:

| 논문의 정의 | 우리 코드 |
|---|---|
| full-base support (100% 받쳐짐) | `min_horizontal_support_ratio = 1.0` |
| partial-base support (면적 비율 문턱) | `min_horizontal_support_ratio` |
| polygon-based stability (무게중심 ∈ 지지 다각형) | `min_com_margin_ratio` |

결론도 우리 측정과 일치한다 — **polygon 기반이 full-base/partial-base보다
적재율 손해 대비 안정성 이득이 크다.** 우리가 §4 표에서 `bl`/`bssf`는 손해가
0인데 `baf`/`blsf`의 COM 여유 0.03 배치만 걸러낸 것과 같은 현상이다.

**[2] Ramos, A. G., Oliveira, J. F., Lopes, M. P. (2016).
A container loading algorithm with static mechanical equilibrium stability
constraints.** *Transportation Research Part B*, 91, 565–581.
https://doi.org/10.1016/j.trb.2016.06.003

강체의 정적 역학 평형(뉴턴 법칙)으로 안정성을 판정한다. **면적 기반
support factor의 한계를 지적한 논문**이라 §2.1의 "60% 받쳐져도 넘어진다"
주장의 출처로 쓸 수 있다.

**[3] Comparing a static equilibrium based method with the support factor for
horizontal cargo stability in the container loading problem.**
*Pesquisa Operacional*. https://www.scielo.br/j/pope/a/3Fcz83HRMVSZP7f6J6SNZLp/

support factor(면적 비율) vs 정적 평형을 정면 비교한다. 오픈 액세스.

### 6.2 DeepPack3D와 휴리스틱 4종의 출처

**[4] Tsang, Y. P., Mo, D. Y., Chung, K. T., Lee, C. K. M. (2025).
DeepPack3D: A Python package for online 3D bin packing optimization by deep
reinforcement learning and constructive heuristics.**
*Software Impacts*, 23, 100732. https://doi.org/10.1016/j.simpa.2024.100732

`planning.py`가 이식한 원본. 인용할 때는 이것을 쓴다.

**[5] Jylänki, J. (2010). A Thousand Ways to Pack the Bin — A Practical
Approach to Two-Dimensional Rectangle Bin Packing.**
http://pds25.egloos.com/pds/201504/21/98/RectangleBinPack.pdf

BL / BAF / BSSF / BLSF **이름의 원조**다. 2D MaxRects 계열이고, 우리 코드의
maximal free space partitioning도 여기서 3D로 확장된 것이다. §1.3 표의 각
규칙 정의를 확인하려면 이 문서를 본다.

### 6.3 로보틱스 온라인 패킹 + 안정성

**[6] Zhao, H. et al. (2021). Online 3D Bin Packing with Constrained Deep
Reinforcement Learning.** AAAI 2021.
https://cdn.aaai.org/ojs/16155/16155-13-19649-1-2-20210518.pdf

DeepPack3D의 RL 쪽 계보. 우리는 RL을 안 쓰지만, 안정성을 feasibility mask로
거는 구조가 우리의 `_candidates()` 필터와 같은 아이디어다.

**[7] Gao, Z., Wang, L., Kong, Y., Chong, N. Y. (2025). Online 3D Bin Packing
with Fast Stability Validation and Stable Rearrangement Planning.**
arXiv:2507.09123. https://arxiv.org/abs/2507.09123

Load Bearable Convex Polygon(LBCP)으로 안정성 검사를 빠르게 한다. 우리
`support_metrics()`가 지지 영역 bounding box로 근사한 것을 볼록 껍질 +
하중까지 확장한 형태다. **정밀도를 올리려면 다음 단계로 볼 것.**

**[8] SDF-Pack: Towards Compact Bin Packing with Signed-Distance-Field
Minimization.** arXiv:2307.07356. https://arxiv.org/abs/2307.07356

지지 다각형 구성과 안정성 테스트 구현이 자세하다.

### 6.4 팔레타이징 현실 제약

**[9] Solving Pallet Loading Problem with Real-World Constraints.**
arXiv:2307.11531. https://arxiv.org/abs/2307.11531

`~/Downloads`에 이미 PDF가 있는 그 논문. 안정성·하중·적재 순서 등 현실 제약
전반을 다룬다. §5의 "하중 기반 규칙은 아직 못 한다"가 여기 해당한다.

**[10] Enhancing pallet load stability: A MILP model for the Manufacturer's
Pallet Loading Problem with interlocking constraints.**
*Computers & Industrial Engineering* (2026).
https://www.sciencedirect.com/science/article/pii/S0360835226001841

교차적재(interlocking)를 수식으로 모델링한다. 현재 우리 필터는 지지 면적과
무게중심만 보고 **위아래 층의 이음매가 어긋나는지는 안 본다.** 그 축을
추가하려면 여기부터 본다.

**[11] Cargo Stability in the Container Loading Problem — State-of-the-Art and
Future Research Directions.** Springer.
https://link.springer.com/chapter/10.1007/978-3-319-71583-4_23

이 분야 리뷰. 배경을 한 번에 훑을 때.

### 6.5 코드

- DeepPack3D 원본: https://github.com/SoftwareImpacts/SIMPAC-2024-311
- 테스트: `tests/test_stable_planner.py` (17개), `tests/test_planning.py`
- 실행 방법: `docs/test5_run_guide.md`

```bash
PYTHONPATH=src python3 -m pytest tests/ -p no:anyio -q
```
