# arm-reach

> 🇰🇷 한국어 먼저, [English below](#english).

6자유도 또는 7자유도 로봇 팔의 **도달 범위, 조작성(manipulability), 특이점 근접도**를 브라우저에서 살펴보는
정적 웹 앱입니다. Rust → WebAssembly로 빌드되며 100% 클라이언트에서 동작합니다
(서버, 분석 도구, 외부 CDN·폰트, 런타임 네트워크 요청 없음).

## 무엇을 하나요
- **입력**: DH 표 편집기 (6/7행: a, α, d, θ 오프셋; 표준 / 수정(Craig) DH 전환), 관절별 한계(최소/최대 °),
  관절별 현재 각도 슬라이더, 툴 오프셋. 편집 가능한 프리셋 3종 (모두 "일반형" 예시 값, 특정 제품 아님):
  6자유도 구형 손목 산업용 팔, 7자유도 여유자유도 협동 팔, 6자유도 휴머노이드 팔.
- **기구학**: 순기구학(모든 관절 프레임 + 툴 프레임), 기하 자코비안 6×N, 조작성
  w = √det(J·Jᵀ) (병진 3×N 또는 전체 6×N 선택), J_v의 특이값 σ1 ≥ σ2 ≥ σ3과 조건수
  (3×3 대칭 Jacobi 고유값 풀이, 선형대수 crate 미사용), 가장 가까운 관절 한계 여유.
- **작업공간 샘플링**: 고정 시드 난수로 관절 한계 안에서 균일 샘플(2k / 5k / 10k) → 툴 위치와 조작성.
  최대 도달 거리, 도달 범위(반경·z 최소/최대), "특이점 근접" 기준(기본: 최대 w의 5 %) 미만 샘플 비율.
- **도달 확인**: 목표점 (x, y, z)에 가장 가까운 샘플, 거리, 해당 관절 벡터(버튼으로 슬라이더에 불러오기).
- **3D 뷰 (WebGL2)**: 관절 하우징이 있는 팔, RGB 툴 프레임, 조작성으로 색칠한 포인트 클라우드
  (파랑 낮음 → 노랑 높음, 특이점 근접은 빨강), 툴 위치의 조작성 타원체, 드래그 가능한 목표 구.
- **2D 그래프**: 조작성 히스토그램(현재 자세 표시선), 관절 한계 여유 막대그래프.
- 한국어 / English 전환 (브라우저 언어 기본), 모바일 지원, 시스템 다크 모드 따름.

## 모델과 가정
- 강체 링크, 이상적인 회전 관절(모든 관절은 회전 관절), 백래시·유연성·동역학 미고려.
- 작업공간은 **샘플링** 결과이며 정확한 경계가 아닙니다. 샘플 수가 적으면 가장자리와 특이 영역을 놓칠 수 있습니다.
- 충돌(자기 충돌, 바닥, 주변물) 검사 없음. 역기구학 풀이 없음.
- 6×N 조작성은 병진(m)과 회전(rad) 단위를 섞으므로 같은 팔의 자세끼리 비교하는 용도로만 쓰세요.
- 관절 수가 행 수보다 적으면 √det(Jᵀ·J)를 씁니다 (두 경우 모두 특이값의 곱).

## 조작법
- 왼쪽 드래그: 회전 · 오른쪽 드래그 / 두 손가락: 이동 · 휠 / 핀치: 확대 · 더블클릭: 시점 초기화.
- 뷰 프리셋: 원근 / 위 / 옆 / 앞. 토글: 포인트 클라우드, 타원체, 프레임, 특이점 근접만.
- 분홍 목표 구를 드래그하면 수평면 위에서 움직이고, Shift(또는 두 번째 손가락)로 높이를 바꿉니다.
- 관절 하우징을 클릭하면 그 관절이 선택되고, 좌우로 드래그하면 각도가 바뀝니다(한계 안으로 제한).
- "스윕"은 선택한 관절을 한계 사이에서 왕복시킵니다 (0.25× / 1×).

## 로컬 빌드
```sh
rustup target add wasm32-unknown-unknown
cargo test                 # ar_core 및 렌더러 수학 단위 테스트
trunk build --release      # dist/ 에 정적 사이트 생성 (trunk 0.22.1)
sh tools/privacy_check.sh  # "clean" 이 출력되어야 함
```
`dist/`는 `/arm-reach/` 하위 경로로 배포되도록 빌드됩니다 (GitHub Pages).

## 라이선스
MIT — [LICENSE](LICENSE) 참고.

**교육용 추정치입니다. 실제 로봇에 사용하기 전에 반드시 검증하세요.**

---

<a id="english"></a>
## English

A static web app to explore the **reach, manipulability and singularity proximity** of a 6- or 7-DoF arm.
Written in Rust, compiled to WebAssembly, and runs 100 % client-side (no server, no analytics, no external
CDNs or fonts, no network requests at runtime).

### What it does
- **Inputs**: DH table editor (6/7 rows: a, α, d, θ offset; standard / modified (Craig) DH toggle), per-joint
  limits (min/max °), per-joint angle sliders, tool offset. Three editable presets with illustrative "typical"
  values (not any specific product): 6-DoF spherical-wrist industrial arm, 7-DoF redundant collaborative arm,
  6-DoF humanoid arm.
- **Kinematics**: forward kinematics (all joint frames + tool frame), 6×N geometric Jacobian, manipulability
  w = √det(J·Jᵀ) (translational 3×N or full 6×N), singular values σ1 ≥ σ2 ≥ σ3 of J_v and the condition number
  (small symmetric Jacobi eigen solver, no linear-algebra crates), closest joint-limit margin.
- **Workspace sampling**: uniform random joint vectors inside the limits from a fixed seed (2k / 5k / 10k) →
  tool positions with their manipulability. Max reach, reach envelope (radius and z min/max), and the share of
  samples below a "near-singular" threshold (default 5 % of max w).
- **Reach check**: nearest sampled point to a target (x, y, z), its distance and joint vector (load into sliders).
- **3D view (WebGL2)**: arm with joint housings, RGB tool frame, point cloud coloured by manipulability
  (blue low → yellow high, near-singular red), manipulability ellipsoid at the tool, draggable target sphere.
- **2D plots**: manipulability histogram with the current pose marker; joint-limit margin bars.
- Korean / English toggle (browser default), mobile friendly, follows the system colour scheme.

### Model and assumptions
- Rigid links, ideal revolute joints (all joints are revolute); no backlash, compliance or dynamics.
- The workspace is **sampled**, not exact; low sample counts can miss edges and singular regions.
- No collision checking (self, floor or environment). No inverse kinematics.
- The 6×N measure mixes metres and radians; compare it only between poses of the same arm.
- With fewer joints than rows, √det(Jᵀ·J) is used (in both cases the product of the singular values).

### Controls
- Left drag: orbit · right drag / two fingers: pan · wheel / pinch: zoom · double-click: reset view.
- View presets: Perspective / Top / Side / Front. Toggles: point cloud, ellipsoid, frames, near-singular only.
- Drag the pink target sphere on a horizontal plane; hold Shift (or use a second finger) to change its height.
- Click a joint housing to select that joint, then drag sideways to change its angle (clamped to limits).
- "Sweep" moves the selected joint back and forth across its limits (0.25× / 1×).

### Local build
```sh
rustup target add wasm32-unknown-unknown
cargo test                 # ar_core and renderer math unit tests
trunk build --release      # static site in dist/ (trunk 0.22.1)
sh tools/privacy_check.sh  # must print "clean"
```
`dist/` is built for the `/arm-reach/` sub-path (GitHub Pages).

### License
MIT — see [LICENSE](LICENSE).

**Educational estimates — validate before using on a real robot.**
