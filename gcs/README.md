
## 현재 전송 경계

`telem_sender`는 Pixhawk serial을 열지 않습니다. 외부 MAVLink router가
serial을 단독 소유하고, publisher는 `udp:127.0.0.1:14553` 같은 별도 loopback
telemetry fan-out만 읽습니다. `14551`은 target-distance/control subscriber가
사용하므로 같은 UDP socket을 여러 프로세스가 공유하지 않습니다. `SET_MESSAGE_INTERVAL`과 vehicle-affecting
MAVLink 송신도 수행하지 않습니다.

텔레메트리는 `setting/gcs.yaml`의 FEC 설정으로 노트북에 UDP 전송됩니다.
영상은 YOLO가 받은 frame을 별도 bounded worker에서 축소 JPEG로 인코딩해
`GCS_VIDEO_HOST:GCS_VIDEO_PORT`로 전송합니다. 영상 publisher가 멈춰도
`/target`, target-distance, control에는 영향을 주지 않습니다.

기본 영상에는 bbox나 라벨을 그리지 않습니다. YOLO는 raw frame과 탐지 metadata를
각각 보내고, 노트북 dashboard가 metadata의 원본 `frame_width`/`frame_height`와
`bbox_*_px`를 사용해 canvas에 overlay합니다. `GCS_VIDEO_ANNOTATED=1`은 Gazebo
로컬 디버그 topic에서만 annotated frame을 켜는 선택 옵션이며, 기본 GCS 영상은
계속 raw입니다. YOLO의 `GCS_LOCAL_PREVIEW=1`을 지정하면 같은 worker가 만든
JPEG를 로컬 `/stream`에도 제공합니다.

노트북에서는 다음처럼 bridge를 실행합니다.

```bash
python3 gcs/tools/gcs_bridge.py --listen-port 15550 --video-port 15560
```

브라우저는 `http://127.0.0.1:8080/`에서 telemetry와 `/video.mjpg`를
확인합니다. bbox는 onboard 영상에 그리지 않고 telemetry 좌표로 viewer가
overlay합니다.

## 데이터 경로

SITL에서는 `simulation/scripts/start_router.sh`가 선택적으로
`udp:127.0.0.1:14553`을 GCS 전용 telemetry fan-out으로 추가합니다.
`telem_sender`는 이 endpoint를 읽기만 하고, 결과 JSON을 `setting/gcs.yaml`의
노트북 주소로 FEC 전송합니다. `control`의 14550, `target-distance`의 14551과
GCS publisher의 입력을 같은 UDP 소켓에 겹쳐 바인드하지 않습니다.

publisher만 실행할 때:

```bash
MAVPROXY_GCS_TELEMETRY_ENDPOINT=udp:127.0.0.1:14553 \
  ./gcs/run_publisher.sh
```

실제 장비에서는 외부 MAVLink router가 serial을 소유하고 같은 loopback
fan-out을 제공해야 합니다. `telem_sender`와 GCS bridge는 어떤 MAVLink 명령,
`SET_MESSAGE_INTERVAL`, serial open도 수행하지 않습니다. 영상이 끊겨도
telemetry publisher와 onboard control은 계속 실행되며, bridge는 영상 stale
상태만 표시합니다.
