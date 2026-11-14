//! Korean / English strings. Static markup carries `data-i18n` keys; Rust looks them up here.

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Lang {
    En,
    Ko,
}

/// (key, English, Korean)
const T: &[(&str, &str, &str)] = &[
    ("tagline", "Reach, dexterity and singularity explorer for 6/7-DoF arms", "6/7자유도 로봇 팔의 도달 범위·조작성·특이점 탐색기"),
    ("model", "Arm model", "팔 모델"),
    ("preset", "Preset", "프리셋"),
    ("industrial6", "6-DoF spherical-wrist industrial arm (typical)", "6자유도 구형 손목 산업용 팔 (일반형)"),
    ("cobot7", "7-DoF redundant collaborative arm (typical)", "7자유도 여유자유도 협동 팔 (일반형)"),
    ("humanoid6", "6-DoF humanoid arm, shoulder-elbow-wrist (typical)", "6자유도 휴머노이드 팔, 어깨-팔꿈치-손목 (일반형)"),
    ("conv", "DH convention", "DH 규약"),
    ("standard", "standard", "표준"),
    ("modified", "modified", "수정(Craig)"),
    ("tool", "Tool offset (last frame) [m]", "툴 오프셋 (마지막 프레임) [m]"),
    ("joints", "Joint angles (click # to select)", "관절 각도 (# 클릭으로 선택)"),
    ("sampling", "Workspace sampling", "작업공간 샘플링"),
    ("samples", "Samples", "샘플 수"),
    ("measure", "Measure", "지표"),
    ("m3", "translational J_v (3×N)", "병진 J_v (3×N)"),
    ("m6", "full J (6×N)", "전체 J (6×N)"),
    ("thresh", "Near-singular below [% of max w]", "특이점 근접 기준 [최대 w의 %]"),
    ("reach", "Reach check", "도달 확인"),
    ("target", "Target [m]", "목표점 [m]"),
    ("load", "Load joints into sliders", "관절값을 슬라이더로 불러오기"),
    ("nogl", "WebGL2 is not available in this browser. Numbers and plots still work.", "이 브라우저에서는 WebGL2를 쓸 수 없습니다. 수치와 그래프는 그대로 동작합니다."),
    ("tlabel", "target", "목표"),
    ("cloud", "point cloud", "포인트 클라우드"),
    ("ellipsoid", "ellipsoid", "타원체"),
    ("frames", "frames", "프레임"),
    ("nearonly", "near-singular only", "특이점 근접만"),
    ("persp", "Perspective", "원근"),
    ("top", "Top", "위"),
    ("side", "Side", "옆"),
    ("front", "Front", "앞"),
    ("sweep", "▶ sweep", "▶ 스윕"),
    ("pause", "❚❚ pause", "❚❚ 정지"),
    ("pose", "Current pose", "현재 자세"),
    ("hist", "Manipulability histogram (marker = current pose)", "조작성 히스토그램 (표시선 = 현재 자세)"),
    ("margins", "Joint-limit margins [°]", "관절 한계 여유 [°]"),
    ("hint", "Left drag: orbit · right drag / two fingers: pan · wheel / pinch: zoom · double-click: reset · drag the pink target (shift / second finger: height) · click a joint housing, then drag sideways to turn it", "왼쪽 드래그: 회전 · 오른쪽 드래그 / 두 손가락: 이동 · 휠 / 핀치: 확대 · 더블클릭: 초기화 · 분홍 목표점 드래그 (Shift / 두 번째 손가락: 높이) · 관절 하우징 클릭 후 좌우 드래그로 회전"),
    ("foot", "Educational estimates: rigid links, ideal revolute joints, sampled (not exact) workspace, no collisions. Validate before using on a real robot. MIT license.", "교육용 추정치입니다: 강체 링크, 이상적인 회전 관절, 샘플링된(정확하지 않은) 작업공간, 충돌 미고려. 실제 로봇에 쓰기 전에 반드시 검증하세요. MIT 라이선스."),
    ("max_reach", "max reach", "최대 도달 거리"),
    ("radius", "radius min / max", "반경 최소 / 최대"),
    ("zrange", "z min / max", "z 최소 / 최대"),
    ("wmax", "max w", "최대 w"),
    ("nearfrac", "near-singular samples", "특이점 근접 샘플"),
    ("nearest", "nearest sample", "가장 가까운 샘플"),
    ("dist", "distance", "거리"),
    ("jvec", "joints [°]", "관절 [°]"),
    ("tool_xyz", "tool x / y / z", "툴 x / y / z"),
    ("sigma", "σ of J_v", "J_v의 σ"),
    ("cond", "condition number", "조건수"),
    ("margin", "nearest limit margin", "최소 한계 여유"),
    ("status", "status", "상태"),
    ("ok", "✅ well-conditioned", "✅ 양호"),
    ("warn", "⚠️ near-singular", "⚠️ 특이점 근접"),
    ("nodata", "no samples", "샘플 없음"),
];

pub fn tr(lang: Lang, key: &str) -> &'static str {
    T.iter().find(|t| t.0 == key).map(|t| if lang == Lang::Ko { t.2 } else { t.1 }).unwrap_or("?")
}
