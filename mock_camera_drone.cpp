#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <iomanip> 
#include <thread>
#include <atomic>
#include <mutex>
#include <sstream>
#include <chrono>
#include <cmath>    // ← THÊM CHO M_PI, sin, cos, sqrt, atan2
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

// ← THAY THẾ: Include ArduPilot dialect
// Nếu không có, dùng common và define thủ công
#include "c_library_v2/ardupilotmega/mavlink.h"
//#include "c_library_v2/common/mavlink.h"

// Define CAMERA_FEEDBACK message ID (nếu thiếu)
// #ifndef MAVLINK_MSG_ID_CAMERA_FEEDBACK
// #define MAVLINK_MSG_ID_CAMERA_FEEDBACK 180

// // Struct cho CAMERA_FEEDBACK (parse thủ công)
// typedef struct __mavlink_camera_feedback_t {
//     uint64_t time_usec;
//     int32_t lat;
//     int32_t lng;
//     float alt_msl;
//     float alt_rel;
//     float roll;
//     float pitch;
//     float yaw;
//     float foc_len;
//     uint16_t img_idx;
//     uint8_t target_system;
//     uint8_t cam_idx;
//     uint8_t flags;
//     uint16_t completed_captures;
// } mavlink_camera_feedback_t;
// #endif

// Định nghĩa M_PI nếu chưa có
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// === BẮT BUỘC: Buộc std::cout in ngay lập tức ===
static const auto _force_cout_flush = []() {
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;
    return 0;
}();

// Cấu hình mạng
#define QGC_IP "127.0.0.1"
#define QGC_PORT 14550 
#define MY_PORT 14540
#define SITL_PORT 14560      // ← THÊM: Port cho SITL forward
#define RTSP_PORT 8554

// IDs
#define SYS_ID 1         
#define COMP_ID_CAMERA 100 

// Commands
#define MAV_CMD_LEGACY_PHOTO 203 
#define MAV_CMD_REQUEST_CAMERA_INFORMATION 521
#define MAV_CMD_DO_SET_CAM_TRIGG_DIST 206
#define MAV_CMD_DO_DIGICAM_CONTROL 203

// =====================================================================
// FORWARD DECLARATIONS (Khai báo trước các hàm)
// =====================================================================
void send_image_captured_with_pose();
void send_camera_capture_status();
void send_mavlink(mavlink_message_t* msg);
void check_auto_capture();              // ← THÊM DÒNG NÀY
void execute_capture(std::string reason);  // ← THÊM DÒNG NÀY
void handle_socket_data(int fd, const char* tag);

// =====================================================================
// ← PHẦN MỚI: GLOBAL VARIABLES CHO CAPTURE PIPELINE
// =====================================================================
GstElement *capture_pipeline = nullptr;
GstElement *appsink = nullptr;
bool capture_ready = false;

// ← THÊM: Biến cho auto capture
std::atomic<bool> auto_capture_enabled(false);
std::atomic<float> trigger_distance(0.0f);  // Khoảng cách giữa các lần chụp (mét)
std::atomic<double> last_capture_lat(0.0);
std::atomic<double> last_capture_lon(0.0);

// Global state (giữ nguyên)
int sock;
int sitl_sock = -1;  // ← THÊM: Socket riêng cho SITL
struct sockaddr_in qgcAddr;
struct sockaddr_in sitlAddr;  // ← THÊM
struct sockaddr_in myAddr;
std::atomic<int> image_count(0);
std::atomic<bool> video_recording(false);
std::atomic<bool> running(true);
std::mutex log_mutex;
std::atomic<double> current_lat{21.0077678};
std::atomic<double> current_lon{105.8433921};
std::atomic<double> current_alt{50.0};
std::atomic<float> current_yaw{0.0f};
std::atomic<bool> heartbeat_logged_qgc(false);
std::atomic<bool> heartbeat_logged_sitl(false);

uint32_t get_time_boot_ms() { 
    return (uint32_t)time(NULL) * 1000; 
}

void send_mavlink(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    sendto(sock, buf, len, 0, (struct sockaddr*)&qgcAddr, sizeof(qgcAddr));
}

void send_image_captured_with_pose() {
    mavlink_message_t msg;

    double lat = current_lat.load();
    double lon = current_lon.load();
    double alt = current_alt.load();
    float yaw = current_yaw.load();

    int32_t lat_int = static_cast<int32_t>(lat * 1e7);
    int32_t lon_int = static_cast<int32_t>(lon * 1e7);
    int32_t alt_int = static_cast<int32_t>(alt * 1000);
    int32_t rel_alt_int = alt_int + 5000;

    float q[4];
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    q[0] = cy;  q[1] = 0.0f;  q[2] = 0.0f;  q[3] = sy;

    uint64_t time_usec = static_cast<uint64_t>(get_time_boot_ms()) * 1000ULL;
    
    int current_count = image_count.load();

    // ← SỬA: Sử dụng đúng tham số cho CAMERA_IMAGE_CAPTURED
    mavlink_msg_camera_image_captured_pack(
        SYS_ID, 
        COMP_ID_CAMERA, 
        &msg,
        get_time_boot_ms(),          // time_boot_ms
        time_usec,                    // time_utc (microseconds since UNIX epoch)
        1,                            // camera_id
        lat_int,                      // lat (degE7)
        lon_int,                      // lon (degE7)
        alt_int,                      // alt (mm)
        rel_alt_int,                  // relative_alt (mm)
        q,                            // q[4] quaternion
        current_count,                // image_index ← QUAN TRỌNG
        1,                            // capture_result: 1 = SUCCESS (thay vì -1)
        "http://192.168.15.60:8000/latest.jpg"  // file_url
    );

    send_mavlink(&msg);
    
    // ← LOG ĐỂ DEBUG
    {
        std::lock_guard<std::mutex> lock(log_mutex);
        std::cout << "[SEND] 📤 CAMERA_IMAGE_CAPTURED (index=" << current_count 
                  << ", result=SUCCESS) | GPS: " << lat << "," << lon 
                  << " | Alt: " << alt << "m\n";
    }
}

// =====================================================================
// ← PHẦN MỚI: KHỞI TẠO CAPTURE PIPELINE (1 LẦN DUY NHẤT)
// =====================================================================
void init_capture_pipeline() {
    std::cout << "[CAPTURE] Đang khởi tạo GStreamer capture pipeline...\n";
    
    std::string pipeline_str = 
        "rtspsrc location=rtsp://127.0.0.1:" + std::to_string(RTSP_PORT) + "/webcam "
        "latency=0 protocols=tcp ! "
        "rtph264depay ! h264parse ! avdec_h264 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";
    
    GError *error = nullptr;
    capture_pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    
    if (error) {
        std::cerr << "[ERROR] Không tạo được pipeline: " << error->message << "\n";
        g_error_free(error);
        return;
    }
    
    appsink = gst_bin_get_by_name(GST_BIN(capture_pipeline), "sink");
    if (!appsink) {
        std::cerr << "[ERROR] Không tìm thấy appsink!\n";
        return;
    }
    
    // Start pipeline
    GstStateChangeReturn ret = gst_element_set_state(capture_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[ERROR] Không start được pipeline!\n";
        return;
    }
    
    // Đợi pipeline sẵn sàng
    sleep(2);
    
    capture_ready = true;
    std::cout << "[CAPTURE] ✅ GStreamer capture pipeline READY!\n";
}

// =====================================================================
// ← PHẦN MỚI: HÀM CHỤP ẢNH TỐI ƯU (THAY THẾ execute_capture CŨ)
// =====================================================================
void execute_capture(std::string reason) {
    if (!capture_ready) {
        std::cerr << "[ERROR] Capture pipeline chưa sẵn sàng!\n";
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(log_mutex);
        std::cout << "\n[CAMERA] >>> CHỤP ẢNH OPTIMIZED! (Nguồn: " << reason << ") <<<\n";
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Lấy sample từ appsink
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
    if (!sample) {
        std::cerr << "[ERROR] Không lấy được frame từ stream!\n";
        return;
    }
    
    // Chuyển GstSample -> OpenCV Mat
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    
    int width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);
    
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);
    
    // Tạo Mat từ buffer (zero-copy)
    cv::Mat frame(height, width, CV_8UC3, (void*)map.data);
    
    // Tạo filename
    time_t now = time(0);
    std::stringstream ss;
    ss << "snapshot_" << now << ".jpg";
    std::string filename = ss.str();
    
    // Lưu ảnh với OpenCV (NHANH)
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95);
    cv::imwrite(filename, frame, compression_params);
    
    // Cleanup GStreamer
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    {
        std::lock_guard<std::mutex> lock(log_mutex);
        std::cout << "[PERF] ⚡ Chụp xong trong " << duration.count() << "ms\n";
    }
    
    // Lấy GPS hiện tại
    double lat = current_lat.load();
    double lon = current_lon.load();
    double alt = current_alt.load();
    float yaw_deg = current_yaw.load() * 57.2958f;
    
    // ← QUAN TRỌNG: Tăng count TRƯỚC KHI gửi message
    image_count++;
    
    int new_count = image_count.load();
    
    {
        std::lock_guard<std::mutex> lock(log_mutex);
        std::cout << "[CAMERA] 📸 Image count: " << new_count << "\n";
    }
    
    // ĐỢI 1 chút để đảm bảo ảnh đã lưu xong
    usleep(100000);  // 100ms
    
    // Gửi CAMERA_IMAGE_CAPTURED (QGC dùng message này để cập nhật UI)
    send_image_captured_with_pose();
    
    // Gửi thêm CAMERA_CAPTURE_STATUS để đồng bộ
    send_camera_capture_status();
    
    // Gắn EXIF ASYNC (không block) - sau khi đã gửi message
    std::thread([filename, lat, lon, alt, yaw_deg]() {
        // Xác định hướng GPS
        std::string lat_ref = (lat >= 0) ? "N" : "S";
        std::string lon_ref = (lon >= 0) ? "E" : "W";
        
        std::string exif_cmd = "exiftool -q -overwrite_original "
            "-GPSLatitudeRef=\"" + lat_ref + "\" "
            "-GPSLatitude=\"" + std::to_string(std::abs(lat)) + "\" "
            "-GPSLongitudeRef=\"" + lon_ref + "\" "
            "-GPSLongitude=\"" + std::to_string(std::abs(lon)) + "\" "
            "-GPSAltitude=\"" + std::to_string(alt) + "\" "
            "-GPSImgDirection=\"" + std::to_string(yaw_deg) + "\" "
            "-XMP:FlightYawDegree=\"" + std::to_string(yaw_deg) + "\" "
            "\"" + filename + "\"";
        
        int result = system(exif_cmd.c_str());
        if (result != 0) {
            std::cerr << "[EXIF] Warning: exiftool returned " << result << "\n";
        } else {
            std::cout << "[EXIF] ✅ Đã geotag: " << filename 
                      << " | GPS: " << lat << ", " << lon 
                      << " (" << lat_ref << "/" << lon_ref << ")"
                      << " | Yaw: " << yaw_deg << "°\n";
        }
    }).detach();
}

// =====================================================================
// ← PHẦN MỚI: DỌN DẸP CAPTURE PIPELINE KHI THOÁT
// =====================================================================
void cleanup_capture_pipeline() {
    if (capture_pipeline) {
        std::cout << "[CAPTURE] Đang dừng capture pipeline...\n";
        gst_element_set_state(capture_pipeline, GST_STATE_NULL);
        gst_object_unref(capture_pipeline);
    }
    capture_ready = false;
}

// =============================================================================
// RTSP SERVER FUNCTIONS (GIỮ NGUYÊN)
// =============================================================================
void rtsp_server_thread() {
    gst_init(nullptr, nullptr);
    
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    GstRTSPServer *server = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server, std::to_string(RTSP_PORT).c_str());

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    gst_rtsp_media_factory_set_launch(factory, 
        "( v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=I420 ! "
        "x264enc tune=zerolatency bitrate=3000 speed-preset=ultrafast key-int-max=10 ! "
        "rtph264pay name=pay0 pt=96 )");
    
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_mount_points_add_factory(mounts, "/webcam", factory);
    
    g_object_unref(mounts);
    gst_rtsp_server_attach(server, NULL);

    std::cout << "[RTSP] Server running at: rtsp://127.0.0.1:" << RTSP_PORT << "/webcam\n";
    
    g_main_loop_run(loop);
}

// =============================================================================
// MAVLINK FUNCTIONS (GIỮ NGUYÊN - CHỈ THAY ĐỔI NHỎ Ở handle_command_long)
// =============================================================================

void setup_udp() {
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    memset(&myAddr, 0, sizeof(myAddr));
    myAddr.sin_family = AF_INET;
    myAddr.sin_addr.s_addr = INADDR_ANY;
    myAddr.sin_port = htons(MY_PORT);

    if (bind(sock, (struct sockaddr *)&myAddr, sizeof(myAddr)) == -1) {
        perror("Bind error"); 
        exit(1);
    }

    memset(&qgcAddr, 0, sizeof(qgcAddr));
    qgcAddr.sin_family = AF_INET;
    qgcAddr.sin_addr.s_addr = inet_addr(QGC_IP);
    qgcAddr.sin_port = htons(QGC_PORT);
    
    // ← THÊM: Setup SITL socket
    memset(&sitlAddr, 0, sizeof(sitlAddr));
    sitlAddr.sin_family = AF_INET;
    sitlAddr.sin_addr.s_addr = INADDR_ANY;
    sitlAddr.sin_port = htons(SITL_PORT);

    sitl_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sitl_sock == -1) {
        perror("Socket error (SITL)");
    } else {
        int sitl_flags = fcntl(sitl_sock, F_GETFL, 0);
        fcntl(sitl_sock, F_SETFL, sitl_flags | O_NONBLOCK);

        if (bind(sitl_sock, (struct sockaddr *)&sitlAddr, sizeof(sitlAddr)) == -1) {
            perror("Bind error (SITL)");
            close(sitl_sock);
            sitl_sock = -1;
        } else {
            std::cout << "[SITL] Listening on UDP port " << SITL_PORT 
                      << " for ArduPilot feedback\n";
        }
    }
   
    std::cout << "============================================================" << std::endl;
    std::cout << "   CAMERA PROTOCOL V2 + RTSP SIMULATOR (OPTIMIZED)" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "   MAVLink Port: " << MY_PORT << " -> " << QGC_PORT << std::endl;
    std::cout << "   SITL Port:    " << SITL_PORT << std::endl;
    std::cout << "   RTSP Port:    " << RTSP_PORT << std::endl;
    std::cout << "============================================================" << std::endl;
}

void send_heartbeat() {
    mavlink_message_t msg;
    
    // ← QUAN TRỌNG: Set đúng type và autopilot cho ArduPilot
    mavlink_msg_heartbeat_pack(
        SYS_ID,                      // system_id = 1 (cùng với SITL)
        COMP_ID_CAMERA,              // component_id = 100
        &msg,
        MAV_TYPE_CAMERA,             // type: CAMERA
        MAV_AUTOPILOT_INVALID,       // autopilot: INVALID (vì là peripheral)
        0,                           // base_mode
        0,                           // custom_mode
        MAV_STATE_ACTIVE             // system_status
    );
    
    send_mavlink(&msg);
}

void send_camera_information() {
    mavlink_message_t msg;
    
    const char* uri = "http://192.168.1.100:8000/camera_definition.xml";  // ← Đổi IP thật
    
    uint8_t v[32] = "SimCam v1.0";
    uint8_t m[32] = "ArduPilot Camera";
    
    uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE | 
                     CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                     CAMERA_CAP_FLAGS_HAS_MODES |
                     CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM;

    mavlink_msg_camera_information_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(), 
        v,                    // vendor_name
        m,                    // model_name
        1,                    // firmware_version
        50.0f,                // focal_length
        10.0f,                // sensor_size_h
        10.0f,                // sensor_size_v
        1920,                 // resolution_h
        1080,                 // resolution_v
        0,                    // lens_id
        flags,                // flags
        1,                    // cam_definition_version
        uri,                  // cam_definition_uri
        0,                    // gimbal_device_id
        0                     // camera_device_id
    );
    send_mavlink(&msg);
    
    std::cout << " [SEND] CAMERA_INFORMATION (ArduPilot compatible)\n";
}

void send_camera_settings() {
    mavlink_message_t msg;
    mavlink_msg_camera_settings_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(),
        CAMERA_MODE_IMAGE,
        1.0f,
        1.0f,
        0
    );
    send_mavlink(&msg);
    
    std::cout << " [SEND] CAMERA_SETTINGS\n";
}

void send_camera_capture_status() {
    mavlink_message_t msg;
    
    uint8_t image_status = 0;
    uint8_t video_status = video_recording ? 1 : 0;
    int current_count = image_count.load();
    
    mavlink_msg_camera_capture_status_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(),
        image_status,
        video_status,
        0.0f,
        0,
        27000.0f,
        current_count,
        0
    );
    send_mavlink(&msg);
}

void send_storage_information() {
    mavlink_message_t msg;
    mavlink_msg_storage_information_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(),
        1, 1, STORAGE_STATUS_READY,
        32000.0f, 5000.0f, 27000.0f,
        90.0f, 45.0f, STORAGE_TYPE_SD,
        "", 0
    );
    send_mavlink(&msg);
    
    std::cout << " [SEND] STORAGE_INFORMATION\n";
}

void send_video_stream_information() {
    mavlink_message_t msg;
    
    char name[32] = "Webcam Stream";
    char uri[160];
    snprintf(uri, sizeof(uri), "rtsp://127.0.0.1:%d/webcam", RTSP_PORT);
    
    mavlink_msg_video_stream_information_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        1, 1, VIDEO_STREAM_TYPE_RTSP,
        VIDEO_STREAM_STATUS_FLAGS_RUNNING,
        30.0f, 1920, 1080, 3000,
        0, 60,
        name, uri,
        0, 0
    );
    send_mavlink(&msg);
    
    std::cout << " [SEND] VIDEO_STREAM_INFORMATION: " << uri << "\n";
}

void send_ack(uint16_t command, uint8_t result = MAV_RESULT_ACCEPTED) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(SYS_ID, COMP_ID_CAMERA, &msg, 
                                 command, result, 0, 0, 0, 0);
    send_mavlink(&msg);
}

void handle_command_long(mavlink_command_long_t& cmd) {
    // ← THÊM LOG ĐỂ XEM NGUỒN GỬI
    std::cout << "\n[RECV] Command: " << cmd.command 
              << " | From: sysid=" << (int)cmd.target_system 
              << ", compid=" << (int)cmd.target_component << std::endl;
    
    if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
        uint32_t msg_id = (uint32_t)cmd.param1;
        
        if (msg_id == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
            send_camera_information();
            send_ack(cmd.command);
        }
        else if (msg_id == MAVLINK_MSG_ID_CAMERA_SETTINGS) {
            send_camera_settings();
            send_ack(cmd.command);
        }
        else if (msg_id == MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) {
            send_camera_capture_status();
            send_ack(cmd.command);
        }
        else if (msg_id == MAVLINK_MSG_ID_STORAGE_INFORMATION) {
            send_storage_information();
            send_ack(cmd.command);
        }
        else if (msg_id == MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION) {
            send_video_stream_information();
            send_ack(cmd.command);
        }
        else {
            send_ack(cmd.command, MAV_RESULT_UNSUPPORTED);
        }
    }
    
    // ← THAY ĐỔI: Bỏ thread riêng, gọi trực tiếp (vì đã tối ưu)
    else if (cmd.command == MAV_CMD_IMAGE_START_CAPTURE) {
        std::cout << " -> IMAGE_START_CAPTURE\n";
        send_ack(cmd.command);
        execute_capture("QGC Button");  // ← Gọi trực tiếp, không cần thread
    }
    
    else if (cmd.command == MAV_CMD_IMAGE_STOP_CAPTURE) {
        send_ack(cmd.command);
    }
    
    else if (cmd.command == MAV_CMD_VIDEO_START_CAPTURE) {
        std::cout << " -> VIDEO_START_CAPTURE\n";
        video_recording = true;
        send_ack(cmd.command);
    }
    
    else if (cmd.command == MAV_CMD_VIDEO_STOP_CAPTURE) {
        std::cout << " -> VIDEO_STOP_CAPTURE\n";
        video_recording = false;
        send_ack(cmd.command);
    }
    
    else if (cmd.command == MAV_CMD_SET_CAMERA_MODE) {
        int mode = (int)cmd.param2;
        std::cout << " -> SET_CAMERA_MODE: " << mode << "\n";
        send_ack(cmd.command);
        usleep(100000);
        send_camera_settings();
    }
    
    else if (cmd.command == MAV_CMD_REQUEST_CAMERA_INFORMATION) {
        send_camera_information();
        send_ack(cmd.command);
    }
    else if (cmd.command == MAV_CMD_REQUEST_CAMERA_SETTINGS) {
        send_camera_settings();
        send_ack(cmd.command);
    }
    else if (cmd.command == MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS) {
        send_camera_capture_status();
        send_ack(cmd.command);
    }
    else if (cmd.command == MAV_CMD_LEGACY_PHOTO) {
        if ((int)cmd.param5 == 1) {
            std::cout << " -> LEGACY PHOTO\n";
            send_ack(cmd.command);
            execute_capture("Legacy Command");
        }
    }
    
    else if (cmd.command == 112) {
        std::cout << " -> CAMERA_TRIGGER from Mission\n";
        send_ack(cmd.command);
        execute_capture("Mission Trigger");
    }
    
    else {
        send_ack(cmd.command, MAV_RESULT_UNSUPPORTED);
    }
}

void handle_socket_data(int fd, const char* tag) {
    if (fd < 0) {
        return;
    }

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    struct sockaddr_in src;
    socklen_t len = sizeof(src);

    while (true) {
        len = sizeof(src);
        ssize_t recsize = recvfrom(fd, buf, MAVLINK_MAX_PACKET_LEN, 0,
                                   (struct sockaddr*)&src, &len);
        if (recsize <= 0) {
            break;
        }

        mavlink_message_t msg;
        mavlink_status_t status;

        for (int i = 0; i < recsize; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                bool is_heartbeat = msg.msgid == MAVLINK_MSG_ID_HEARTBEAT;
                bool is_gpos = msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT;

                if (is_heartbeat) {
                    std::atomic<bool>* hb_flag = nullptr;
                    if (strcmp(tag, "QGC") == 0) {
                        hb_flag = &heartbeat_logged_qgc;
                    } else if (strcmp(tag, "SITL") == 0) {
                        hb_flag = &heartbeat_logged_sitl;
                    }

                    bool expected = false;
                    if (!hb_flag || hb_flag->compare_exchange_strong(expected, true)) {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        std::cout << "[HB][" << tag << "] sysid=" << (int)msg.sysid
                                  << ", compid=" << (int)msg.compid
                                  << " | type=" << (int)hb.type
                                  << ", autopilot=" << (int)hb.autopilot
                                  << ", status=" << (int)hb.system_status << "\n";
                    }
                    continue;
                }

                if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                    mavlink_command_long_t cmd;
                    mavlink_msg_command_long_decode(&msg, &cmd);

                    std::cout << "\n[DEBUG][" << tag << "] Received from: "
                              << "sysid=" << (int)msg.sysid 
                              << ", compid=" << (int)msg.compid
                              << " | Target: sysid=" << (int)cmd.target_system
                              << ", compid=" << (int)cmd.target_component
                              << " | Command: " << cmd.command << "\n";

                    if ((cmd.target_system == SYS_ID || cmd.target_system == 0) &&
                        (cmd.target_component == COMP_ID_CAMERA || cmd.target_component == 0)) {
                        handle_command_long(cmd);
                    }
                }
                else if (msg.msgid == MAVLINK_MSG_ID_CAMERA_TRIGGER) {
                    std::cout << "\n[RECV][" << tag << "] CAMERA_TRIGGER from Mission Planner!\n";
                    execute_capture("Mission Trigger");
                }
                else if (msg.msgid == 180 || msg.msgid == MAVLINK_MSG_ID_CAMERA_FEEDBACK) {
                    std::cout << "\n[DEBUG][" << tag << "] ✅ DETECTED CAMERA_FEEDBACK (msgid=180)!\n";

                    uint8_t* payload = (uint8_t*)msg.payload64;

                    uint64_t time_usec;
                    int32_t lat_int, lng_int;
                    float alt_msl, alt_rel, roll, pitch, yaw, foc_len;
                    uint16_t img_idx, completed_captures;
                    uint8_t target_system, cam_idx, flags;

                    memcpy(&time_usec,          payload + 0,  8);
                    memcpy(&lat_int,            payload + 8,  4);
                    memcpy(&lng_int,            payload + 12, 4);
                    memcpy(&alt_msl,            payload + 16, 4);
                    memcpy(&alt_rel,            payload + 20, 4);
                    memcpy(&roll,               payload + 24, 4);
                    memcpy(&pitch,              payload + 28, 4);
                    memcpy(&yaw,                payload + 32, 4);
                    memcpy(&foc_len,            payload + 36, 4);
                    memcpy(&img_idx,            payload + 40, 2);
                    memcpy(&target_system,      payload + 42, 1);
                    memcpy(&cam_idx,            payload + 43, 1);
                    memcpy(&flags,              payload + 44, 1);
                    memcpy(&completed_captures, payload + 45, 2);

                    double lat = lat_int / 1e7;
                    double lon = lng_int / 1e7;
                    float yaw_deg = yaw;

                    std::cout << "\n╔══════════════════════════════════════════╗\n";
                    std::cout << "║   📸 CAMERA_FEEDBACK TỪ ARDUPILOT!     ║\n";
                    std::cout << "╚══════════════════════════════════════════╝\n";
                    std::cout << "  Source: " << tag << "\n";
                    std::cout << "  Image #" << img_idx << "\n";
                    std::cout << "  GPS: " << std::fixed << std::setprecision(7)
                              << lat << ", " << lon << "\n";
                    std::cout << "  Alt rel: " << alt_rel << "m\n";
                    std::cout << "  Yaw: " << yaw_deg << "°\n";
                    std::cout << "  Time: " << time_usec << " μs\n\n";

                    current_lat.store(lat);
                    current_lon.store(lon);
                    current_alt.store(alt_rel);
                    current_yaw.store(yaw_deg * (M_PI / 180.0f));

                    execute_capture("ArduPilot CAMERA_FEEDBACK");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);
                    
                    current_lat.store(pos.lat / 1e7);
                    current_lon.store(pos.lon / 1e7);
                    current_alt.store(pos.relative_alt / 1000.0);
                    current_yaw.store(pos.hdg / 100.0f * 3.14159f / 180.0f);
                }
            }
        }
    }
}

void status_loop() {
    while (running) {
        send_camera_capture_status();
        sleep(1);
    }
}

void heartbeat_loop() {
    int counter = 0;
    while (running) {
        send_heartbeat();
        if (counter % 5 == 0) {
            send_camera_information();
        }
        counter++;
        sleep(1);
    }
}

// =============================================================================
// MAIN - ← THAY ĐỔI QUAN TRỌNG
// =============================================================================
int main() {
    setup_udp();

    // 1. Khởi động RTSP server
    std::cout << "\n[INIT] Starting RTSP server on port " << RTSP_PORT << "...\n";
    std::thread rtsp_thread(rtsp_server_thread);
    rtsp_thread.detach();
    sleep(3);

    // ← THÊM MỚI: 2. Khởi tạo capture pipeline
    std::cout << "\n[INIT] Initializing capture pipeline...\n";
    init_capture_pipeline();
    
    if (!capture_ready) {
        std::cerr << "[ERROR] Capture pipeline FAILED! Exiting...\n";
        return 1;
    }

    // 3. Gửi thông tin camera
    std::cout << "[INIT] Sending initial CAMERA_INFORMATION + SETTINGS...\n";
    send_camera_information();
    send_camera_settings();
    send_storage_information();
    send_video_stream_information();

    std::cout << "[INIT] Sending initial heartbeats...\n";
    for (int i = 0; i < 8; i++) {
        send_heartbeat();
        usleep(150000);
    }

    // 4. Khởi động threads
    std::cout << "[MAIN] Khởi động thread HEARTBEAT...\n";
    std::thread hb_thread(heartbeat_loop);
    std::cout << "[MAIN] Khởi động thread STATUS...\n";
    std::thread status_thread(status_loop);

    // 5. Thông báo thành công
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "   MOCK CAMERA + RTSP (OPTIMIZED) ĐÃ HOẠT ĐỘNG 100%!" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "   Performance  : GStreamer + OpenCV (50-100ms/capture)" << std::endl;
    std::cout << "   Component ID : " << COMP_ID_CAMERA << " (Camera)" << std::endl;
    std::cout << "   MAVLink Port : " << MY_PORT << " → " << QGC_PORT << std::endl;
    std::cout << "   RTSP Stream  : rtsp://127.0.0.1:" << RTSP_PORT << "/webcam" << std::endl;
    std::cout << std::string(60, '=') << "\n" << std::endl;

    // 6. Main loop
// === THÊM DEBUG VÀO MAIN LOOP ===
// Thay thế phần main loop (dòng 720-780) bằng code này:

    // 6. Main loop
    while (running) {
        handle_socket_data(sock, "QGC");
        handle_socket_data(sitl_sock, "SITL");
        usleep(5000);
    }

    // 7. Dọn dẹp
    std::cout << "\nShutting down...\n";
    running = false;
    
    cleanup_capture_pipeline();  // ← THÊM MỚI
    
    hb_thread.join();
    status_thread.join();
    close(sock);
    if (sitl_sock >= 0) {
        close(sitl_sock);
    }

    return 0;
}