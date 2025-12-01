from pymavlink import mavutil
import time

# Kết nối UDP với SITL
master = mavutil.mavlink_connection("udpout:127.0.0.1:14550") # gửi về QGC
sitl = mavutil.mavlink_connection("udp:127.0.0.1:14551")      # nhận từ SITL

print("Camera simulator running...\n")

# Camera properties
CAMERA_ID = 1
vendor = "ChatGPTCam"
model = "SimV2"

def send_camera_information():
    master.mav.camera_information_send(
        time_boot_ms=0,
        camera_id=CAMERA_ID,
        vendor_name=vendor.encode(),
        model_name=model.encode(),
        firmware_version=1,
        focal_length=35.0,
        sensor_size_h=13.0,
        sensor_size_v=8.0,
        resolution_h=1920,
        resolution_v=1080,
        lens_id=1,
        flags=0,
        cam_definition_version=1,
        cam_definition_uri=b""
    )
    print(">>> Sent CAMERA_INFORMATION")

def send_camera_settings():
    master.mav.camera_settings_send(
        time_boot_ms=0,
        camera_id=CAMERA_ID,
        mode_id=1,
        zoomLevel=1.0,
        focusLevel=0.0
    )
    print(">>> Sent CAMERA_SETTINGS")

def send_capture_status():
    master.mav.camera_capture_status_send(
        time_boot_ms=0,
        camera_id=CAMERA_ID,
        image_status=1,
        video_status=0,
        image_interval=0,
        recording_time_ms=0,
        available_capacity=10000
    )
    print(">>> Sent CAMERA_CAPTURE_STATUS")

def send_image_captured():
    master.mav.camera_image_captured_send(
        time_utc=0,
        time_boot_ms=0,
        camera_id=CAMERA_ID,
        lat=0,
        lon=0,
        alt=0,
        relative_alt=0,
        q=[1,0,0,0],
        image_index=1,
        capture_result=1,
        file_url=b"/tmp/img_001.jpg"
    )
    print(">>> Sent CAMERA_IMAGE_CAPTURED")

# -------- MAIN LOOP --------
while True:
    msg = sitl.recv_match(blocking=True)
    if not msg:
        continue

    # ========== IN TẤT CẢ GÓI TIN TỪ QGROUND CONTROL ==========
    print(f"\n<<< Received from QGC: {msg.get_type()}")
    print(msg)  # in toàn bộ nội dung message
    
    # ===========================================================

    # ------- PROCESS COMMANDS -------
    if msg.get_type() == "COMMAND_LONG":
        # REQUEST_MESSAGE
        if msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:

            if msg.param1 == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION:
                print("QGC requested CAMERA_INFORMATION")
                send_camera_information()

            elif msg.param1 == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_SETTINGS:
                print("QGC requested CAMERA_SETTINGS")
                send_camera_settings()

            elif msg.param1 == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
                print("QGC requested CAMERA_CAPTURE_STATUS")
                send_capture_status()

        # START_CAPTURE
        if msg.command == mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE:
            print("QGC sent IMAGE_START_CAPTURE")
            time.sleep(1)
            send_image_captured()

        # TRIGGER CONTROL
        if msg.command == mavutil.mavlink.MAV_CMD_DO_TRIGGER_CONTROL:
            print("QGC sent DO_TRIGGER_CONTROL")
            time.sleep(1)
            send_image_captured()
