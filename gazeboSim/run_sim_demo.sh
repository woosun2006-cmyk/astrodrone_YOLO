#!/bin/bash
# ═════════════════════════════════════════════════════════════════════════════
#  run_sim_demo.sh  —  명령 한 줄로 시뮬레이션 전체를 띄운다
#
#      wsl -e bash -lc "~/run_sim_demo.sh"          # YOLO 사용 (기본)
#      wsl -e bash -lc "~/run_sim_demo.sh noyolo"   # HSV 색상검출
#
#  이 스크립트 하나가 다음을 전부 한다. 준비물을 미리 띄워둘 필요가 없다.
#
#      [1] Gazebo GUI + ArduCopter SITL + MAVProxy      (PC, WSLg 로 화면 표시)
#      [2] 타겟 스폰                                     (PC)
#      [3] 카메라 RTP 스트리밍 활성화                     (PC)
#      [4] 검출기 기동 → /target                         (PC)
#      [5] 역터널 + socat 릴레이                          (PC → droneVideo → Jetson)
#      [6] Jetson 스택 기동                              (mavlink_proxy /
#          target_distance / hybrid_guidance, 로그는 [JETSON] 접두어로 스트림)
#      [7] 미션 비행                                      (PC)
#      [8] 종료 시 전부 정리                              (Ctrl+C 포함)
#
#  이전 버전은 준비물이 하나라도 없으면 "준비물이 빠졌습니다" 하고 종료했다.
#  그 준비를 사람이 손으로 하거나 Claude 에게 시켜야 했던 부분을 여기로 옮긴
#  것이 이 스크립트다. 원본은 ~/run_sim_demo.prev.sh 로 보관돼 있다.
#
#  옵션(환경변수)
#      ALT=4.0         비행 고도(m). hard_limit_m(5.0) 아래여야 한다.
#      PLANE_W/PLANE_H 타겟 평면 크기 덮어쓰기. 비우면 model.sdf 그대로
#                      (1.033 x 0.690). 고정고도 4m 진단이면 3.10 / 2.07.
#      JETSON=0        Jetson 단계를 건너뛴다 (PC 만으로 확인할 때)
#      SIM_ONLY=1      기동만 하고 미션은 안 날린다 (수동으로 조종하고 싶을 때)
#      CONF=0.15       YOLO 신뢰도 임계값 (기본 0.25)
#      NORTH=8         타겟 north 위치(m)
#      KEEP=1          종료해도 Gazebo/SITL 을 살려둔다 (연속 실행용)
#      SIM_FX=205.47   시뮬 카메라 실측 초점거리를 Jetson 설정에 임시 적용.
#                      기본은 꺼져 있다(설정 파일을 안 건드림). 켜는 이유는
#                      아래 "초점거리 불일치" 참고. 종료 시 원래 값으로
#                      되돌리고, sim-backup 도 같이 원복한다.
#
#  초점거리 불일치 (2026-08-15 발견)
#  ───────────────────────────────────────────────────────────────────────────
#  setting/MAVLink.yaml 의 pixel_focal_length_px 는 530 인데, 이 시뮬 카메라의
#  실측값은 205.47 이다 (2.58배). 파일 주석도 530 을 "rough starting value,
#  not derived from a measured focal length" 라고 적어두고 있다.
#
#  pos_calculator.cpp 는 ground_offset_m = pixel_offset / fx * altitude 이므로,
#  fx 를 2.58배 크게 잡으면 수평 거리를 그만큼 작게 본다. 그러면 3차원 사선
#  분해에서 전진 성분만 과소평가되어 기체가 거의 수직으로 내려온다.
#
#  2026-08-15 실측 비행에서 그대로 나타났다. 수평 오프셋/고도 비가 접근 시작
#  0.53 에서 종료 시 1.16 으로 커졌다(사선 이동이면 일정해야 한다). 결국
#  고도 1.12m 에서 타겟이 1.31m 옆에 남았고, 그 고도의 지상 반폭이 1.74m라
#  타겟이 화면 가장자리로 밀려 잘리면서 로스트 → 긴급착륙으로 끝났다.
SIM_FX="${SIM_FX:-}"
# ═════════════════════════════════════════════════════════════════════════════
set -u

MODE="${1:-yolo}"
case "$MODE" in
  yolo|noyolo) ;;
  *) echo "사용법: $0 [yolo|noyolo]" >&2; exit 2 ;;
esac

REPO="${REPO:-$HOME/astrodrone_YOLO}"
GZS="$REPO/gazeboSim"
VENV="$HOME/venv-ardupilot/bin/python"
SYSPY=/usr/bin/python3          # apt OpenCV = GStreamer YES (pip 휠은 안 됨)
CORE="$HOME/sim_mission_core.py"
PORT=8002
CONF="${CONF:-0.25}"
NORTH="${NORTH:-8}"
JETSON="${JETSON:-1}"
SIM_ONLY="${SIM_ONLY:-0}"
KEEP="${KEEP:-0}"
# 타겟 평면 크기. 비워두면 models/target_basket/model.sdf 그대로(1.033 x 0.690).
PLANE_W="${PLANE_W:-}"
PLANE_H="${PLANE_H:-}"

# 비행 고도 4.0 m.
#
# setting/safety.yaml 의 altitude_limit 은 2026-08-15 에 soft 4.0->8.0,
# hard 5.0->10.0 으로 올렸다. 상한은 10 m 지만 **실제 탐지 한계는 6~7 m** 다.
#
# 고도별 실측 (0812best.onnx, conf 0.25, 640x480, 평면 1.033 x 0.690):
#
#   고도   4.0    5.0    6.0    7.0    8.0    10.0
#   폭(px)  53     42     35     30     27      21
#   conf  0.785  0.680  0.651  0.549  0.365  0.000(미탐지)
#
# 7 m 를 넘기려면 타겟 텍스처 평면을 키워야 한다(10 m 에서 4 m 와 같은 53 px 를
# 얻으려면 평면이 약 2.6 m). PLANE_W / PLANE_H 로 조정한다.
#
# 하강 구간도 확인돼 있다 — 4.0m 0.777 / 3.0m 0.838 / 2.0m 0.907 / 1.5m 0.879 /
# 1.2m 0.733 로 전 구간 유지된다.
#
# 참고로 WHITE_BASKET_SETUP.md 의 3.10 x 2.07 (640x480 기준) 은 고도 4m 에서
# 0.959 로 더 높지만 내려오면 무너진다 — 2.0m 0.323, 1.5m 0.334, 1.2m 미탐지.
# 미션은 4m 에서 1m 까지 하강하므로 고정고도 진단용이 아니면 쓰지 말 것.
ALT="${ALT:-4.0}"

HOP1="astro@10.0.0.1"           # droneVideo
JET="astro@192.168.0.216"       # astro-desktop
DV_LAN=192.168.0.34
# UserKnownHostsFile=/dev/null 이라 매 접속이 "처음 보는 호스트"가 되고, ssh 가
# 그때마다 "Warning: Permanently added ... to the list of known hosts" 를 찍는다.
# 실제로는 /dev/null 에 쓰므로 아무 데도 안 쌓인다. LogLevel=ERROR 로 그 경고만
# 끄고 동작은 그대로 둔다.
SSHOPT="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10 -o LogLevel=ERROR"

if [ "$MODE" = "yolo" ]; then
  DETECTOR="$GZS/gazebo_yolo_target.py"
  MODEL="$REPO/YOLO_MODEL/0812best.onnx"
  DET_ARGS="--port $PORT --model $MODEL --conf $CONF"
  DET_LOG=/tmp/gazebo_yolo_target.log
  BACKEND=yolo
  DET_DESC="YOLO / 0812best.onnx"
else
  DETECTOR="$GZS/gazebo_camera_target.py"
  MODEL=""
  # 타겟이 실측 흰색(#F4F7F4)이므로 HSV 범위도 흰색으로 맞춘다. 검출기 기본값은
  # 빨강(0,120,70 ~ 10,255,255)인데, 그건 이전 모델을 빨갛게 칠했을 때 얘기다.
  # 흰색 = 채도 낮고 명도 높음. 잔디는 채도가 높아 그대로 걸러진다.
  DET_ARGS="--port $PORT --hsv-lo 0,0,200 --hsv-hi 180,40,255"
  DET_LOG=/tmp/gazebo_camera_target.log
  BACKEND=hsv
  DET_DESC="HSV 흰색검출 (YOLO 없음)"
fi

# WSLg 로 Gazebo GUI 를 띄우기 위한 환경. wsl -e bash -lc 로 들어오면 보통
# 이미 세팅돼 있지만, 비어 있는 경우를 대비해 기본값을 박아둔다.
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"

TUNNEL_PIDS=()
TAIL_PID=""

step() { echo; echo "═══ $* ═══"; }
info() { echo "    $*"; }

# ── 원격 실행 헬퍼 ───────────────────────────────────────────────────────────
# 2단 SSH 에 명령을 통과시킬 때 따옴표가 겹쳐 깨지는 일이 잦아서, 명령을
# base64 로 감싸 보낸다. 이러면 내부에 어떤 따옴표가 들어가도 안전하다.
dv() {   # droneVideo 에서 실행
  local b64; b64=$(printf '%s' "$1" | base64 -w0)
  sshpass -p astro ssh $SSHOPT "$HOP1" "echo $b64 | base64 -d | bash -s"
}
jetson() {  # Jetson 에서 실행 (droneVideo 경유)
  local b64; b64=$(printf '%s' "$1" | base64 -w0)
  sshpass -p astro ssh $SSHOPT "$HOP1" \
    "sshpass -p astro ssh $SSHOPT $JET 'echo $b64 | base64 -d | bash -s'"
}

# ── 정리 ─────────────────────────────────────────────────────────────────────
cleanup() {
  echo
  step "정리"
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null
  for p in "${TUNNEL_PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  info "역터널 종료"

  if [ "$JETSON" = "1" ]; then
    # run_new_algorithm.sh 는 자체 trap 으로 setting/MAVLink.yaml 을 되돌린다.
    # 그래도 강제 종료로 trap 을 놓칠 수 있으니 복원을 한 번 더 확인한다.
    # tail -f 도 같이 죽여야 한다. 로컬 ssh 를 kill 해도 원격에서 돌던
    # tail 은 살아남아 Jetson 에 프로세스가 하나씩 쌓인다.
    jetson 'pkill -f run_new_algorithm.sh; pkill -f hybrid_guidance; pkill -f target_distance; pkill -f mavlink_proxy; pkill -f "socat .*sitl_serial"; pkill -f "socat TCP-LISTEN:8002"; pkill -f "tail -n \+1 -f /tmp/run_new_algorithm.log"; sleep 1; cd ~/astro-drone && [ -f gazeboSim/MAVLink.yaml.sim-backup ] && cp gazeboSim/MAVLink.yaml.sim-backup setting/MAVLink.yaml && echo "  MAVLink.yaml 복원됨"' 2>/dev/null
    info "Jetson 스택 종료"

    if [ -n "$SIM_FX" ]; then
      # 실비행 설정으로 확실히 되돌린다. setting/MAVLink.yaml 과
      # gazeboSim/MAVLink.yaml.sim-backup 둘 다 손본다 — 후자를 놔두면
      # 다음 실행이 그걸로 "복원"해서 시뮬값이 영구히 남는다.
      jetson 'cd ~/astro-drone && for f in setting/MAVLink.yaml gazeboSim/MAVLink.yaml.sim-backup; do [ -f "$f" ] && sed -i "s/^\( *pixel_focal_length_px: \).*/\1530/" "$f"; done; grep -n "pixel_focal_length_px" setting/MAVLink.yaml gazeboSim/MAVLink.yaml.sim-backup' 2>/dev/null | sed 's/^/    /'
      info "pixel_focal_length_px 530 으로 원복 완료"
    fi
  fi

  if [ "$KEEP" != "1" ]; then
    pkill -f 'gazebo_yolo_target.py'   2>/dev/null
    pkill -f 'gazebo_camera_target.py' 2>/dev/null
    info "검출기 종료  (Gazebo/SITL 은 KEEP=0 이어도 남겨둡니다 — 다시 돌릴 때 start_pc_sim.sh 가 알아서 정리합니다)"
  else
    info "KEEP=1 — 검출기까지 그대로 둡니다"
  fi
  echo
  echo "로그: PC $DET_LOG / ~/gazebo_run.log / ~/sitl_run.log,  Jetson /tmp/run_new_algorithm.log"
}
trap cleanup EXIT INT TERM

echo "═════════════════════════════════════════════════════════════"
echo "  시뮬레이션 전체 기동   검출기: $DET_DESC"
echo "  Jetson 단계: $([ "$JETSON" = 1 ] && echo 포함 || echo 생략)   미션: $([ "$SIM_ONLY" = 1 ] && echo 생략 || echo 실행)"
echo "═════════════════════════════════════════════════════════════"

# ── 사전 파일 확인 (여기서만 중단한다) ───────────────────────────────────────
fail=0
for f in "$GZS/start_pc_sim.sh" "$DETECTOR" "$CORE"; do
  [ -f "$f" ] || { echo "  [X] 없음: $f"; fail=1; }
done
[ -z "$MODEL" ] || [ -f "$MODEL" ] || { echo "  [X] 가중치 없음: $MODEL"; fail=1; }
[ "$fail" = 0 ] || exit 1

# ═══ [1] Gazebo + SITL + MAVProxy ════════════════════════════════════════════
step "[1/7] Gazebo GUI + ArduCopter SITL + MAVProxy"
info "start_pc_sim.sh 가 순서대로 띄웁니다 (Gazebo→SITL→MAVProxy, 90초 정도)"
info "타겟은 이 스크립트가 따로 스폰하므로 여기선 생략합니다"
SPAWN_TARGET=0 bash "$GZS/start_pc_sim.sh" 2>&1 | sed 's/^/    /'

if ! pgrep -f 'gz-sim' >/dev/null; then
  echo "  [X] Gazebo 가 뜨지 않았습니다 — ~/gazebo_run.log 확인"; exit 1
fi
ss -lun 2>/dev/null | grep -q ':9002' || { echo "  [X] FDM 9002 미바인딩"; exit 1; }
info "[OK] Gazebo 화면이 떠 있어야 합니다 (WSLg)"

WORLD=$(gz topic -l 2>/dev/null | sed -n 's#^/world/\([^/]*\)/.*#\1#p' | head -1)
WORLD="${WORLD:-iris_runway}"
info "월드: $WORLD"

# ═══ [2] 타겟 스폰 ═══════════════════════════════════════════════════════════
step "[2/7] 타겟 스폰"
# 두 모드 모두 실제 탐지 대상인 흰색 바구니를 쓴다.
#   collision : 실측 0.238 x 0.138 x 0.070 m  (pose z=0.035)
#   visual    : 학습셋에서 잘라낸 실사 텍스처를 입힌 수평 평면
# 모델은 astrohome(192.168.0.116) 의 astro-drone/gazeboSim/models/target_basket
# 를 그대로 가져온 것. 전체 설명은 그 머신의
# ~/gazebo_yolo_bridge_0815/WHITE_BASKET_SETUP.md 에 있다.
info "target_basket (흰색 바구니, 실사 텍스처 평면) → 북 ${NORTH}m"
[ -n "$PLANE_W$PLANE_H" ] && info "평면 덮어쓰기: ${PLANE_W:-1.033} x ${PLANE_H:-0.690} m"
PLANE_W="$PLANE_W" PLANE_H="$PLANE_H" \
  bash "$GZS/spawn_target.sh" "$WORLD" "$NORTH" 0 2>&1 | sed 's/^/    /'

# ═══ [3] 카메라 스트리밍 ═════════════════════════════════════════════════════
step "[3/7] 카메라 RTP 스트리밍 활성화"
# GstCameraPlugin 은 enable_streaming 을 받기 전까지 아무것도 안 내보낸다.
# 이게 빠지면 검출기는 조용히 프레임 0장으로 영원히 대기한다.
IMG_TOPIC=""
for i in $(seq 1 15); do
  IMG_TOPIC=$(gz topic -l 2>/dev/null | grep -E '/image$' | head -1)
  [ -n "$IMG_TOPIC" ] && break
  sleep 1
done
if [ -n "$IMG_TOPIC" ]; then
  # gz topic -p 는 한 번 쏘고 즉시 종료한다. gz-transport 는 발행자가 구독자를
  # 발견하는 데 시간이 걸리므로, 발견 전에 프로세스가 죽으면 메시지가 그냥
  # 사라진다. 2026-08-15 12:22 실행에서 실제로 이렇게 유실되어 프레임이 한 장도
  # 안 왔고, 검출기는 176샘플 내내 found=false 였다. 그런데도 이 단계는 발행만
  # 하고 [OK] 를 찍었다 — 검증이 없었던 것이 진짜 결함이다.
  #
  # 그래서 이제는 udp 5600 에 RTP 가 실제로 흐르는지 확인하고, 흐를 때까지
  # 다시 발행한다. 검출기는 다음 단계에서 뜨므로 여기서 5600 을 잠깐 잡아도
  # 충돌하지 않는다.
  stream_alive() {
    /usr/bin/python3 - <<'PY'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    s.bind(("127.0.0.1", 5600))
except OSError:
    sys.exit(1)
s.settimeout(2.0)
try:
    s.recvfrom(4096)
except socket.timeout:
    sys.exit(1)
sys.exit(0)
PY
  }
  ok_stream=0
  for i in $(seq 1 6); do
    gz topic -t "${IMG_TOPIC}/enable_streaming" -m gz.msgs.Boolean -p "data: true" >/dev/null 2>&1
    sleep 1
    if stream_alive; then ok_stream=1; break; fi
  done
  if [ "$ok_stream" = 1 ]; then
    info "[OK] RTP 수신 확인 (${i}회 시도): $IMG_TOPIC"
  else
    echo "  [X] enable_streaming 을 6회 보냈지만 udp 5600 에 프레임이 없습니다."
    echo "      이대로 진행하면 검출기가 아무것도 못 봅니다. ~/gazebo_run.log 의"
    echo "      GstCameraPlugin 줄을 확인하세요."
    exit 1
  fi
else
  echo "  [!] image 토픽 없음 — 카메라 없는 월드일 수 있습니다"
fi

# ═══ [4] 검출기 ══════════════════════════════════════════════════════════════
step "[4/7] 검출기 기동 — $DET_DESC"
pkill -f 'gazebo_yolo_target.py'   2>/dev/null
pkill -f 'gazebo_camera_target.py' 2>/dev/null
sleep 1
( cd "$GZS" && setsid nohup "$SYSPY" -u "$DETECTOR" $DET_ARGS \
    >"$DET_LOG" 2>&1 </dev/null & disown )
for i in $(seq 1 30); do
  curl -s --max-time 1 "http://127.0.0.1:$PORT/target" >/dev/null 2>&1 && break
  sleep 1
done
if ! curl -s --max-time 2 "http://127.0.0.1:$PORT/target" >/dev/null 2>&1; then
  echo "  [X] /target 이 열리지 않았습니다"; tail -20 "$DET_LOG" | sed 's/^/    /'; exit 1
fi
grep -E 'self-check|classes|model:' "$DET_LOG" 2>/dev/null | head -3 | sed 's/^/    /'
info "[OK] /target: $(curl -s --max-time 2 http://127.0.0.1:$PORT/target)"

# ═══ [5] 터널 ════════════════════════════════════════════════════════════════
step "[5/7] 역터널 + socat 릴레이 (PC → droneVideo → Jetson)"
if [ "$JETSON" = "1" ]; then
  # 포트 선택 근거는 gazeboSim/README.md 참고:
  #   5762  SITL SERIAL1 (5760 은 MAVProxy 가 단독 점유)
  #   25760 droneVideo 중계 (15760 은 Jetson sshd 가 사용중)
  #   28002 /target 중계
  # 마지막 구간이 socat 인 이유: Jetson sshd 가 remote port forward 를 거부한다.
  pkill -f 'ssh .*-R 25760' 2>/dev/null
  pkill -f 'ssh .*-R 28002' 2>/dev/null
  # 원격 sshd 가 포트를 놓을 때까지 기다린다. 바로 다시 붙으면
  # ExitOnForwardFailure 때문에 새 터널이 즉사한다.
  for i in $(seq 1 15); do
    dv 'ss -lnt | grep -qE "127.0.0.1:(25760|28002)" && echo BUSY || echo FREE' \
      2>/dev/null | grep -q FREE && break
    sleep 1
  done
  info "이전 터널 정리 완료"

  # ExitOnForwardFailure=yes 가 중요하다. 이게 없으면 원격 포트가 이미
  # 잡혀 있을 때 ssh 는 "remote port forwarding failed" 를 한 줄 찍고도
  # 그냥 살아있어서, 터널이 없는데 프로세스는 보이는 상태가 된다.
  # 그러면 Jetson 이 타겟을 못 보는 이유를 한참 찾게 된다.
  sshpass -p astro ssh $SSHOPT -o ExitOnForwardFailure=yes \
    -N -R 25760:127.0.0.1:5762 "$HOP1" &
  TUNNEL_PIDS+=($!)
  sshpass -p astro ssh $SSHOPT -o ExitOnForwardFailure=yes \
    -N -R 28002:127.0.0.1:$PORT "$HOP1" &
  TUNNEL_PIDS+=($!)
  sleep 3
  alive=0
  for p in "${TUNNEL_PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=$((alive+1)); done
  if [ "$alive" = 2 ]; then
    info "[OK] 역터널 2개 (25760←SITL, 28002←/target)"
  else
    echo "  [X] 역터널이 $alive/2 만 살아있습니다 — droneVideo 에서 포트가 이미 잡혀있을 수 있습니다"
    dv 'ss -lnt | grep -E ":(25760|28002)"' | sed 's/^/      /'
    exit 1
  fi

  dv "pkill -f 'socat TCP-LISTEN:25760'; pkill -f 'socat TCP-LISTEN:28002'; sleep 1
      setsid nohup socat TCP-LISTEN:25760,bind=$DV_LAN,fork,reuseaddr TCP:127.0.0.1:25760 >/tmp/socat_mav.log 2>&1 </dev/null & disown
      setsid nohup socat TCP-LISTEN:28002,bind=$DV_LAN,fork,reuseaddr TCP:127.0.0.1:28002 >/tmp/socat_yolo.log 2>&1 </dev/null & disown
      sleep 1; ss -lnt | grep -E ':(25760|28002)' | sed 's/^/      /'"
  info "[OK] droneVideo socat 릴레이"

  jetson "pkill -f 'socat TCP-LISTEN:8002'; sleep 1
          setsid nohup socat TCP-LISTEN:8002,bind=127.0.0.1,fork,reuseaddr TCP:$DV_LAN:28002 >/tmp/socat_target.log 2>&1 </dev/null & disown
          sleep 2
          echo -n '      Jetson 127.0.0.1:8002 → '; curl -s --max-time 3 http://127.0.0.1:8002/target || echo '(응답 없음)'"
  info "[OK] Jetson 127.0.0.1:8002 연결"
else
  info "JETSON=0 — 터널 생략"
fi

# ═══ [6] Jetson 스택 ═════════════════════════════════════════════════════════
step "[6/7] Jetson 스택 (mavlink_proxy / target_distance / hybrid_guidance)"
if [ "$JETSON" = "1" ]; then
  if [ -n "$SIM_FX" ]; then
    # target_distance 는 기동할 때 YAML 을 읽으므로 그 전에 바꿔야 한다.
    # 정리 단계에서 setting/MAVLink.yaml 과 sim-backup 양쪽을 원복한다 —
    # run_new_algorithm.sh 가 뜬 뒤에 백업을 뜨기 때문에, 여기서만 바꾸면
    # 그 백업에 시뮬값이 굳어져 실비행 설정으로 되돌아간다.
    info "pixel_focal_length_px → $SIM_FX (시뮬 실측값, 종료 시 530 으로 원복)"
    jetson "cd ~/astro-drone && sed -i 's/^\\( *pixel_focal_length_px: \\).*/\\1$SIM_FX/' setting/MAVLink.yaml && grep -n 'pixel_focal_length_px' setting/MAVLink.yaml | sed 's/^/      /'"
  fi
  info "run_new_algorithm.sh --sim --camera 를 백그라운드로 띄우고 로그를 여기로 흘립니다"
  info "(첫 실행이면 cmake/make 때문에 수 분 걸릴 수 있습니다)"
  jetson "cd ~/astro-drone && rm -f /tmp/run_new_algorithm.log
          setsid nohup ./scripts/run_new_algorithm.sh --sim --camera >/tmp/run_new_algorithm.log 2>&1 </dev/null & disown
          sleep 2; echo '      기동됨'"

  # Jetson 로그를 [JETSON] 접두어로 계속 흘려보낸다. 미션 출력과 섞여서
  # 어느 쪽에서 무슨 일이 일어나는지 한 화면에서 보인다.
  ( jetson "tail -n +1 -f /tmp/run_new_algorithm.log" 2>/dev/null \
      | sed 's/^/[JETSON] /' ) &
  TAIL_PID=$!

  # hybrid_guidance 가 MAVLink 를 잡을 때까지 기다린다. 이게 붙기 전에
  # 미션을 시작하면 가로챌 주체가 없다.
  info "hybrid_guidance 준비 대기 (최대 300초)"
  for i in $(seq 1 150); do
    if jetson "pgrep -f hybrid_guidance >/dev/null && echo UP" 2>/dev/null | grep -q UP; then
      info "[OK] hybrid_guidance 실행 중 (${i}0초 이내)"; break
    fi
    sleep 2
  done
else
  info "JETSON=0 — 건너뜀. 가로채기는 일어나지 않고 AUTO 미션만 납니다"
fi

# ═══ [7] 미션 ════════════════════════════════════════════════════════════════
step "[7/7] 미션 비행"
if [ "$SIM_ONLY" = "1" ]; then
  info "SIM_ONLY=1 — 기동만 하고 멈춥니다. Ctrl+C 로 정리하세요."
  while true; do sleep 5; done
fi

timeout 560 "$VENV" -u "$CORE" --backend "$BACKEND" --alt "$ALT" \
  --target-url "http://127.0.0.1:$PORT/target"

echo
echo "── 검출기 로그 마지막 8줄 ──"
tail -8 "$DET_LOG" 2>/dev/null | sed 's/^/    /'
