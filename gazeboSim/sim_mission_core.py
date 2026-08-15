#!/usr/bin/env python3
"""AUTO 비행 -> 타겟 탐지 -> GUIDED 가로채기 -> 3D 사선 접근 -> 1m 정지 -> 착륙.

run_sim_demo_yolo.sh / run_sim_demo_noyolo.sh 가 공통으로 호출하는 미션 본체.
원래 run_sim_demo.sh 안에 heredoc 으로 박혀 있던 코드를 그대로 꺼내온 것이라
비행 시퀀스 자체는 동일하고, 달라진 점은 두 가지뿐이다.

  1. /target 엔드포인트 주소를 인자로 받는다 (--target-url).
     검출기가 HSV 든 YOLO 든 계약이 같으므로 미션은 구분할 필요가 없다.
  2. 어느 검출기를 쓰는지 라벨로 찍어준다 (--backend), 로그를 나중에 볼 때
     둘을 헷갈리지 않도록.

이 스크립트는 arm/이륙을 직접 한다. Jetson 의 hybrid_guidance 는
Document/algorithm.md 3-1 에 적힌 대로 arm/이륙을 하지 않고 이미 날고 있는
AUTO 미션을 지켜보다 가로채기만 하므로, 그 "이미 날고 있는 AUTO 미션"을
여기서 만들어 주는 것이다.

venv-ardupilot 의 python 으로 실행해야 한다 (pymavlink 가 거기 있다).
"""
import argparse
import json
import subprocess
import sys
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target-url", default="http://127.0.0.1:8002/target",
                    help="검출기의 /target 엔드포인트")
    ap.add_argument("--backend", default="unknown",
                    help="로그에 찍을 검출기 이름 (hsv | yolo)")
    ap.add_argument("--mavlink", default="udpin:127.0.0.1:14551",
                    help="MAVProxy 가 내보내는 UDP")
    ap.add_argument("--alt", type=float, default=4.0,
                    help="비행 고도(m). setting/safety.yaml 의 hard_limit_m(5.0) 아래여야 한다")
    ap.add_argument("--start", type=float, default=-6.0,
                    help="출발점 north(m). 타겟 남쪽에서 시작해 북쪽으로 날아간다")
    ap.add_argument("--wp-north", type=float, default=10.0,
                    help="AUTO 웨이포인트 north(m). 타겟(북 8m) 2m 너머")
    ap.add_argument("--watch-sec", type=float, default=220.0,
                    help="가로채기를 기다리는 최대 시간(초)")
    ap.add_argument("--stale-ms", type=float, default=2000.0,
                    help="이보다 오래된 탐지는 미검출로 표시한다")
    args = ap.parse_args()

    ALT, START, WP_NORTH = args.alt, args.start, args.wp_north

    m = mavutil.mavlink_connection(args.mavlink)
    if m.wait_heartbeat(timeout=30) is None:
        sys.exit("MAVLink 연결 실패")

    def mode(name):
        mid = m.mode_mapping()[name]
        m.mav.set_mode_send(m.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid)
        for _ in range(60):
            hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb and hb.custom_mode == mid:
                print("  모드 → %s" % name, flush=True)
                return True
        return False

    def pos():
        return m.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)

    def target_state():
        """(found, confirmed) — 단, 오래된 탐지는 미검출로 친다.

        원본 run_sim_demo.sh 는 응답 문자열에 '"found": true' 가 들어있는지만
        봤는데, 검출기들은 스트림이 끊겨도 마지막 탐지 상태를 그대로 들고
        있는다. 실제로 Gazebo 를 내린 뒤 8002 를 찔러보면 age_ms 가 1755만
        (=4.9시간)인데도 found=true 를 반환한다. 그대로 찍으면 화면에는
        탐지=O 가 계속 뜨지만 실제로는 프레임이 한 장도 안 오는 상태다.

        hybrid_guidance 가 tracking 을 판정할 때 링크 신선도를 함께 보는 것과
        같은 취지로, 여기서도 age 를 게이트로 건다. 이 값은 표시 전용이고
        비행 판단에는 쓰이지 않는다 — 판단은 Jetson 쪽이 한다.
        """
        out = subprocess.run(["curl", "-s", "--max-time", "1", args.target_url],
                             capture_output=True, text=True).stdout
        try:
            j = json.loads(out)
        except (ValueError, TypeError):
            return False, False
        fresh = float(j.get("age_ms", 1e9)) <= args.stale_ms
        return bool(j.get("found")) and fresh, bool(j.get("confirmed")) and fresh

    bar = "════════════════════════════════════════════"

    print(bar, flush=True)
    print("  1. 이륙   (검출기: %s)" % args.backend, flush=True)
    print(bar, flush=True)
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
    mode('GUIDED')
    if not armed:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        for _ in range(60):
            hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("  시동", flush=True)
                break
        else:
            sys.exit("arm 실패")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 0, 0, 0, 0, 0, 0, ALT)
        t0 = time.time()
        while time.time() - t0 < 70:
            p = pos()
            if p and -p.z >= ALT - 0.6:
                print("  이륙완료 %.2fm" % (-p.z), flush=True)
                break

    print(flush=True)
    print(bar, flush=True)
    print("  2. AUTO 미션 준비 (출발점: 타겟 남쪽)", flush=True)
    print(bar, flush=True)
    t0 = time.time()
    while time.time() - t0 < 120:
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
            START, 0.0, -ALT, 0, 0, 0, 0, 0, 0, 0, 0)
        p = pos()
        if p and abs(p.x - START) < 0.6 and abs(p.y) < 0.8:
            print("  출발점 north=%.2f alt=%.2f" % (p.x, -p.z), flush=True)
            break
        time.sleep(0.2)

    g = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    items = [(0, g.lat, g.lon, ALT),
             (1, g.lat + int((WP_NORTH - START) * 90), g.lon, ALT)]
    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    for _ in range(len(items)):
        req = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                           blocking=True, timeout=10)
        if req is None:
            sys.exit("미션 업로드 실패")
        s, la, lo, al = items[req.seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, s,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, la, lo, al)
    m.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    print("  미션 웨이포인트: 북 %.0fm" % WP_NORTH, flush=True)

    print(flush=True)
    print(bar, flush=True)
    print("  3. AUTO 비행 시작 — hybrid_guidance 가 가로챌 때까지", flush=True)
    print(bar, flush=True)
    mode('AUTO')
    prev, t0, took, minalt = 'AUTO', time.time(), False, 99.0
    while time.time() - t0 < args.watch_sec:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if not hb:
            continue
        md = mavutil.mode_string_v10(hb)
        ar = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        p = m.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        found, confirmed = target_state()
        if md != prev:
            if md == 'GUIDED':
                print(flush=True)
                print("  ★★★ 타겟 확정 — AUTO → GUIDED 가로채기 ★★★", flush=True)
                print(flush=True)
                took = True
            elif md == 'LAND':
                print(flush=True)
                print("  ★★★ 착륙 시작 ★★★", flush=True)
                print(flush=True)
            else:
                print("  모드 전환: %s → %s" % (prev, md), flush=True)
            prev = md
        if p:
            a = -p.z
            if 0.1 < a < minalt:
                minalt = a
            # 탐지: -  없음 / O  found / ★  confirmed (hybrid_guidance 가
            # 가로채기 판정에 실제로 쓰는 것은 confirmed 쪽이다)
            mark = "★" if confirmed else ("O" if found else "-")
            print("  t=%5.1fs %-7s 북=%6.2f 동=%5.2f 고도=%5.2f  탐지=%s"
                  % (time.time() - t0, md, p.x, p.y, a, mark), flush=True)
        if not ar:
            print(flush=True)
            print("  착륙 완료 — 시동 꺼짐", flush=True)
            break
        time.sleep(0.9)

    print(flush=True)
    print(bar, flush=True)
    print("  결과: 검출기=%s  가로채기=%s  최저고도=%.2fm"
          % (args.backend, "성공" if took else "실패", minalt), flush=True)
    print("  상세 로그: Jetson /tmp/run_new_algorithm.log", flush=True)
    print(bar, flush=True)


if __name__ == "__main__":
    main()
