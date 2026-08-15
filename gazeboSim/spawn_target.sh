#!/usr/bin/env bash
# Spawn gazeboSim/models/target_basket into an already-running Gazebo world,
# so the simulated camera has something for the detectors to find.
#
# Spawning into the live world rather than shipping a modified world file
# keeps this independent of which world is running (the repo's
# simulation/worlds/ensamb_iris_runway.sdf, or ardupilot_gazebo's stock
# iris_runway.sdf) and leaves simulation/ untouched.
#
# Usage: spawn_target.sh [world] [north_m] [east_m]
#
# 타겟 모델에 대하여
# ────────────────────────────────────────────────────────────────────────────
# models/target_basket/model.sdf 는 astrohome(192.168.0.116) 의
# ~/astro-drone/gazeboSim/models/target_basket/model.sdf 를 그대로 가져온 것이다.
# 전체 설명은 astrohome 의 ~/gazebo_yolo_bridge_0815/WHITE_BASKET_SETUP.md 에 있다.
#
#   collision : 실측 0.238 x 0.138 x 0.070 m, pose z=0.035  (실물 그대로)
#   visual    : 수평 평면 + 학습셋에서 잘라낸 실사 텍스처
#               materials/textures/white_basket_reference_crop.png (180x120)
#
# visual 이 실물보다 큰 이유는 "바구니를 크게 만든 것"이 아니라, 저해상도
# 카메라에서도 모델이 학습 때 본 것과 같은 픽셀 크기로 보이게 하려는 것이다.
# collision 은 실측 그대로라 기체를 막거나 걸지 않는다.
#
# 평면 크기와 해상도의 관계 (2026-08-15 실측)
# ────────────────────────────────────────────────────────────────────────────
# WHITE_BASKET_SETUP.md 는 3.10 x 2.07 m (640x480 기준), model.sdf 주석은
# 1.033 x 0.690 m (1920x1080 기준) 이라고 적혀 있다. 정확히 3배이고
# 1920/640 도 3이다 — 해상도가 낮아진 만큼 평면을 키운 것이다.
#
# 이 PC 의 down_camera 는 640x480 이므로 문서대로면 3.10 x 2.07 이 맞는데,
# 실제로 두 크기를 고도별로 재보니 이렇게 나왔다 (0812best.onnx, conf 0.25):
#
#            고도 4.0m   3.0m    2.0m
#   1.033 :   0.789    0.846   0.787     ← 고도에 둔감, 전 구간 안정
#   3.10  :   0.951    0.934   0.413     ← 4m 에서 최고, 내려오면 급락
#
# 3.10 은 고도 4m 진단용으로는 최고(박스 100x64px, 문서가 목표한 111x69px 와
# 일치)지만, 미션은 4m 에서 1m 까지 내려가므로 하강 중에 신뢰도가 무너진다.
# 그래서 기본값은 model.sdf 그대로(1.033) 두고, 고정고도 진단이 필요할 때만
# PLANE_W/PLANE_H 로 바꾼다.
#
# Env:
#   PLANE_W / PLANE_H   평면 크기를 덮어쓴다 (예: PLANE_W=3.10 PLANE_H=2.07).
#                       지정하면 /tmp 에 임시 SDF 를 만들어 그걸 스폰한다.
#                       원본 model.sdf 는 건드리지 않는다.
set -eu

WORLD="${1:-iris_runway}"
NORTH="${2:-8}"
EAST="${3:-0}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# These worlds are ENU, so Gazebo's x is EAST and y is NORTH - the arguments
# do NOT map straight onto pose x/y. Measured against a live world on
# 2026-08-15: with the vehicle at NED north 8 m, `gz model -m ... -p` put it at
# Gazebo (x=0, y=8), confirming NED north == Gazebo +y. Spawning with
# x=$NORTH put the basket 8 m EAST of where the caller asked for it, 11.3 m
# from a vehicle flying due north, which is why the camera never saw it.
SDF_FILE="$SCRIPT_DIR/models/target_basket/model.sdf"
[ -f "$SDF_FILE" ] || { echo "target_basket model missing: $SDF_FILE" >&2; exit 1; }

TEX="$SCRIPT_DIR/models/target_basket/materials/textures/white_basket_reference_crop.png"
[ -f "$TEX" ] || echo "경고: 텍스처가 없습니다 ($TEX) — 평면이 흰색 단색으로 보일 수 있습니다" >&2

if [ -n "${PLANE_W:-}" ] || [ -n "${PLANE_H:-}" ]; then
    PW="${PLANE_W:-1.033}"; PH="${PLANE_H:-0.690}"
    GEN=/tmp/target_basket_plane.sdf
    cat > "$GEN" <<EOF
<?xml version="1.0" ?>
<!-- 생성됨: spawn_target.sh PLANE_W=$PW PLANE_H=$PH
     원본은 models/target_basket/model.sdf (건드리지 않음) -->
<sdf version="1.9">
  <model name="target_basket">
    <static>true</static>
    <link name="basket_link">
      <pose>0 0 0 0 0 0</pose>
      <collision name="basket_collision">
        <pose>0 0 0.035 0 0 0</pose>
        <geometry><box><size>0.238 0.138 0.070</size></box></geometry>
      </collision>
      <visual name="basket_detector_visual">
        <pose>0 0 0.004 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>$PW $PH</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>materials/textures/white_basket_reference_crop.png</albedo_map>
            <roughness>1.0</roughness>
            <metalness>0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
    SDF_FILE="$GEN"
    echo "평면 크기 덮어쓰기: ${PW} x ${PH} m → $GEN"
fi

echo "spawning target_basket into world '$WORLD' at north=${NORTH}m east=${EAST}m (ENU pose x=${EAST} y=${NORTH})"

# Remove a previous instance first so re-running is idempotent; a failure here
# just means it was not there yet. target_drone 도 같이 지운다 — 프레임에
# 물체가 둘이면 다중검출로 간주돼 확정 스트릭이 매 프레임 리셋된다.
for name in target_basket target_drone; do
  gz service -s "/world/${WORLD}/remove" \
    --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: '${name}', type: MODEL" >/dev/null 2>&1 || true
done

REQ="sdf_filename: '${SDF_FILE}', name: 'target_basket', pose: {position: {x: ${EAST}, y: ${NORTH}, z: 0}}"

gz service -s "/world/${WORLD}/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 8000 \
  --req "$REQ"

echo
echo "models now in the world:"
gz model --list 2>/dev/null | head -20 || true
