# kordon_video_stream

Pipeline:

`ROS2 camera topic -> kordon_video_stream node -> H.264 (x264enc) -> RTSP publish -> MediaMTX -> WebRTC/WHEP -> frontend`

## 1. Dependencies

Ubuntu / ROS2 host packages:

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly
```

## 2. Start MediaMTX

```bash
cd $(ros2 pkg prefix kordon_video_stream)/share/kordon_video_stream/config
docker compose -f docker-compose.mediamtx.yml up -d
```

## 3. Build workspace

```bash
cd /home/antondeulia/kordon_ws
colcon build --packages-select kordon_video_stream
source install/setup.bash
```

## 4. Start ROS2 camera -> H.264 publisher node

```bash
ros2 launch kordon_video_stream camera_stream.launch.py \
  input_topic:=/kordon001/camera/image_raw \
  rtsp_url:=rtsp://127.0.0.1:8554/kordon001 \
  bitrate_kbps:=2500 \
  keyint:=30 \
  fps:=30.0
```

For second robot:

```bash
ros2 launch kordon_video_stream camera_stream.launch.py \
  input_topic:=/kordon002/camera/image_raw \
  rtsp_url:=rtsp://127.0.0.1:8554/kordon002
```

## 5. Frontend connection

### Very simple (iframe)

```html
<iframe src="http://<server-ip>:8889/kordon001?controls=true&muted=true&autoplay=true"></iframe>
```

### App-level WebRTC (WHEP)

Use MediaMTX `reader.js` and endpoint:

`http://<server-ip>:8889/kordon001/whep`

Minimal JS init:

```js
reader = new MediaMTXWebRTCReader({
  url: "http://<server-ip>:8889/kordon001/whep",
  onTrack: (evt) => {
    document.getElementById("video").srcObject = evt.streams[0];
  },
  onError: (err) => console.error(err),
});
```

## Notes

- Browser compatibility for WebRTC + H.264 is generally good on Chrome/Edge/Safari.
- If frontend is outside local machine, set public IP/domain in `mediamtx.yml` `webrtcAdditionalHosts`.
- This setup uses MediaMTX as media server (one publisher, many readers). If strict SFU room semantics are needed, migrate server layer to LiveKit/Janus, while keeping this ROS2 node as H.264 publisher.
