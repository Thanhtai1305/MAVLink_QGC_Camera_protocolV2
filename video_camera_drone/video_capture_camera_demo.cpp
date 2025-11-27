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
#include "mavlink/common/mavlink.h"

// Cấu hình mạng
#define QGC_IP "127.0.0.1"
#define QGC_PORT 14550 
#define MY_PORT 14540 

// IDs
#define SYS_ID 1         
#define COMP_ID_CAMERA 100 

// Commands
#define MAV_CMD_LEGACY_PHOTO 203 
#define MAV_CMD_REQUEST_CAMERA_INFORMATION 521

// Global state
int sock;
struct sockaddr_in qgcAddr;
struct sockaddr_in myAddr;
std::atomic<int> image_count(0);
std::atomic<bool> video_recording(false);
std::atomic<bool> running(true);

uint32_t get_time_boot_ms() { return (uint32_t)time(NULL) * 1000; }

void setup_udp() {
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    memset(&myAddr, 0, sizeof(myAddr));
    myAddr.sin_family = AF_INET;
    myAddr.sin_addr.s_addr = INADDR_ANY;
    myAddr.sin_port = htons(MY_PORT);

    if (bind(sock, (struct sockaddr *)&myAddr, sizeof(myAddr)) == -1) {
        perror("Bind error"); exit(1);
    }

    memset(&qgcAddr, 0, sizeof(qgcAddr));
    qgcAddr.sin_family = AF_INET;
    qgcAddr.sin_addr.s_addr = inet_addr(QGC_IP);
    qgcAddr.sin_port = htons(QGC_PORT);
   
    std::cout << "============================================================" << std::endl;
    std::cout << "   CAMERA PROTOCOL V2 SIMULATOR (C++ - FIXED)" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "   Port: " << MY_PORT << " | Target: " << QGC_PORT << std::endl;
    std::cout << "============================================================" << std::endl;
}

void send_mavlink(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    sendto(sock, buf, len, 0, (struct sockaddr*)&qgcAddr, sizeof(qgcAddr));
}

void send_heartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID_CAMERA, &msg,
                               MAV_TYPE_CAMERA, MAV_AUTOPILOT_INVALID,
                               0, 0, MAV_STATE_ACTIVE);
    send_mavlink(&msg);
}

void send_camera_information() {
    mavlink_message_t msg;
    
    // URI tới file XML
    const char* uri = "http://127.0.0.1:8000/camera_definition.xml"; 
    
    uint8_t v[32] = "SimCam";
    uint8_t m[32] = "Virtual Camera V2";
    
    // *** QUAN TRỌNG: Thêm HAS_MODES flag ***
    uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE | 
                     CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                     CAMERA_CAP_FLAGS_HAS_MODES;

    mavlink_msg_camera_information_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(), v, m, 1, 50.0f, 10.0f, 10.0f, 1920, 1080, 0,
        flags, 
        1, // Version
        uri, 0, 0 
    );
    send_mavlink(&msg);
    std::cout << " [SEND] GUI CAMERA_INFORMATION: " << uri << std::endl;
}

// *** MỚI: Gửi CAMERA_SETTINGS - QGC CẦN NÓ! ***
void send_camera_settings() {
    mavlink_message_t msg;
    mavlink_msg_camera_settings_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(),
        CAMERA_MODE_IMAGE,  // mode = 0 (Photo mode)
        1.0f,               // zoom level
        1.0f,               // focus level
        0                   // target system (broadcast)
    );
    send_mavlink(&msg);
    std::cout << " [SEND] GUI CAMERA_SETTINGS (mode=PHOTO)" << std::endl;
}

// *** MỚI: Gửi CAMERA_CAPTURE_STATUS - QUAN TRỌNG NHẤT! ***
void send_camera_capture_status() {
    mavlink_message_t msg;
    
    uint8_t image_status = 0;  // 0 = IDLE (sẵn sàng chụp!)
    uint8_t video_status = video_recording ? 1 : 0;
    
    mavlink_msg_camera_capture_status_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(),
        image_status,           // *** PHẢI LÀ 0 (IDLE) ĐỂ NÚT ENABLED ***
        video_status,
        0.0f,                   // image_interval
        0,                      // recording_time_ms
        27000.0f,               // available_capacity (MB)
        image_count.load(),     // image_count
        0                       // target system
    );
    send_mavlink(&msg);
    // Không log mỗi lần vì sẽ spam
}

// *** MỚI: Gửi STORAGE_INFORMATION ***
void send_storage_information() {
    mavlink_message_t msg;
    mavlink_msg_storage_information_pack(
        SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(),
        1,                      // storage_id
        1,                      // storage_count
        STORAGE_STATUS_READY,   // status
        32000.0f,               // total_capacity (MB)
        5000.0f,                // used_capacity (MB)
        27000.0f,               // available_capacity (MB)
        90.0f,                  // read_speed (MB/s)
        45.0f,                  // write_speed (MB/s)
        STORAGE_TYPE_SD,        // type
        "",                     // name (empty string)
        0                       // storage_usage
    );
    send_mavlink(&msg);
    std::cout << " [SEND] GUI STORAGE_INFORMATION" << std::endl;
}

void send_ack(uint16_t command, uint8_t result = MAV_RESULT_ACCEPTED) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(SYS_ID, COMP_ID_CAMERA, &msg, command, result, 0, 0, 0, 0);
    send_mavlink(&msg);
    std::cout << " [SEND] GUI ACK cho lenh: " << command << std::endl;
}

void send_image_captured() {
    mavlink_message_t msg;
    float q[4] = {1,0,0,0};
    image_count++;
    
    mavlink_msg_camera_image_captured_pack(SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(), 0, 0, 0, 0, 0, 0, q, image_count.load(), 1, "http://img.jpg");
    send_mavlink(&msg);
    std::cout << " [EVENT] -> CHUP ANH THANH CONG! Image #" << image_count.load() << std::endl;
}

void print_incoming_command(mavlink_command_long_t& cmd, uint8_t sysid, uint8_t compid) {
    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << " [RECV] Nhan COMMAND_LONG tu " << (int)sysid << ":" << (int)compid << std::endl;
    std::cout << "   Command ID: " << cmd.command;
    
    if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
        std::cout << " (REQUEST_MESSAGE)" << std::endl;
        std::cout << "   Requested msg_id: " << (int)cmd.param1 << std::endl;
    }
    else if (cmd.command == MAV_CMD_REQUEST_CAMERA_INFORMATION) 
        std::cout << " (REQUEST_CAMERA_INFORMATION)" << std::endl;
    else if (cmd.command == MAV_CMD_REQUEST_CAMERA_SETTINGS)
        std::cout << " (REQUEST_CAMERA_SETTINGS)" << std::endl;
    else if (cmd.command == MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS)
        std::cout << " (REQUEST_CAMERA_CAPTURE_STATUS)" << std::endl;
    else if (cmd.command == MAV_CMD_SET_CAMERA_MODE)
        std::cout << " (SET_CAMERA_MODE)" << std::endl;
    else if (cmd.command == MAV_CMD_IMAGE_START_CAPTURE) 
        std::cout << " (IMAGE_START_CAPTURE)" << std::endl;
    else if (cmd.command == MAV_CMD_IMAGE_STOP_CAPTURE)
        std::cout << " (IMAGE_STOP_CAPTURE)" << std::endl;
    else if (cmd.command == MAV_CMD_VIDEO_START_CAPTURE)
        std::cout << " (VIDEO_START_CAPTURE)" << std::endl;
    else if (cmd.command == MAV_CMD_VIDEO_STOP_CAPTURE)
        std::cout << " (VIDEO_STOP_CAPTURE)" << std::endl;
    else if (cmd.command == MAV_CMD_LEGACY_PHOTO) 
        std::cout << " (DO_DIGICAM_CONTROL - V1)" << std::endl;
    else 
        std::cout << " (Unknown)" << std::endl;
}

// *** MỚI: Background thread gửi CAPTURE_STATUS ***
void status_loop() {
    std::cout << " [THREAD] Status loop started" << std::endl;
    while (running) {
        send_camera_capture_status();
        sleep(1);  // Gửi mỗi giây
    }
}

int main() {
    setup_udp();
    
    // Gửi heartbeat ban đầu
    std::cout << "\n[INIT] Gui heartbeat ban dau..." << std::endl;
    for(int i=0; i<3; i++) { 
        send_heartbeat(); 
        usleep(100000); 
    }
    
    // Gửi CAMERA_INFORMATION ban đầu
    std::cout << "[INIT] Gui CAMERA_INFORMATION ban dau..." << std::endl;
    usleep(500000);
    send_camera_information();
    
    // *** MỚI: Gửi CAMERA_SETTINGS ngay ***
    std::cout << "[INIT] Gui CAMERA_SETTINGS..." << std::endl;
    send_camera_settings();

    std::cout << "\n============================================================" << std::endl;
    std::cout << "   CAMERA DANG CHAY" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "Dang lang nghe commands tu QGroundControl..." << std::endl;
    std::cout << "\nMO QGC VA THU CHUP ANH!" << std::endl;
    std::cout << "============================================================\n" << std::endl;

    // *** MỚI: Start background thread gửi status ***
    std::thread status_thread(status_loop);
    
    time_t last_hb = 0;
    int hb_counter = 0;

    while (running) {
        time_t now = time(NULL);
        if (now - last_hb >= 1) { 
            send_heartbeat(); 
            last_hb = now; 
            hb_counter++;
            
            // Tự động broadcast CAMERA_INFORMATION mỗi 5 giây
            if (hb_counter % 5 == 0) {
                std::cout << " [AUTO] Broadcasting CAMERA_INFORMATION..." << std::endl;
                send_camera_information();
            }
        }

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        struct sockaddr_in src; 
        socklen_t len = sizeof(src);
        ssize_t recsize = recvfrom(sock, buf, MAVLINK_MAX_PACKET_LEN, 0, (struct sockaddr *)&src, &len);

        if (recsize > 0) {
            mavlink_message_t msg; 
            mavlink_status_t status;
            
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    
                    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);

                        if (cmd.target_component == COMP_ID_CAMERA || cmd.target_component == 0) {

                            // *** MỚI: Xử lý MAV_CMD_REQUEST_MESSAGE ***
                            if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
                                uint32_t msg_id = (uint32_t)cmd.param1;
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                
                                if (msg_id == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
                                    send_camera_information();
                                    send_ack(MAV_CMD_REQUEST_MESSAGE);
                                }
                                else if (msg_id == MAVLINK_MSG_ID_CAMERA_SETTINGS) {
                                    send_camera_settings();
                                    send_ack(MAV_CMD_REQUEST_MESSAGE);
                                }
                                else if (msg_id == MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) {
                                    send_camera_capture_status();
                                    send_ack(MAV_CMD_REQUEST_MESSAGE);
                                }
                                else if (msg_id == MAVLINK_MSG_ID_STORAGE_INFORMATION) {
                                    send_storage_information();
                                    send_ack(MAV_CMD_REQUEST_MESSAGE);
                                }
                                else {
                                    send_ack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_UNSUPPORTED);
                                }
                            }
                            
                            // Xử lý REQUEST_CAMERA_INFORMATION (deprecated)
                            else if (cmd.command == MAV_CMD_REQUEST_CAMERA_INFORMATION) {
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                send_camera_information();
                                send_ack(MAV_CMD_REQUEST_CAMERA_INFORMATION);
                            }
                            
                            // Xử lý CHỤP ẢNH V2 (ID 2000)
                            else if (cmd.command == MAV_CMD_IMAGE_START_CAPTURE) {
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                send_ack(MAV_CMD_IMAGE_START_CAPTURE);
                                std::cout << " [PROCESS] Dang xu ly chup anh..." << std::endl;
                                usleep(200000); 
                                send_image_captured();
                            }
                            
                            // Xử lý CHỤP ẢNH V1 (ID 203) - Legacy
                            else if (cmd.command == MAV_CMD_LEGACY_PHOTO) { 
                                if ((int)cmd.param5 == 1) {
                                    print_incoming_command(cmd, msg.sysid, msg.compid);
                                    send_ack(MAV_CMD_LEGACY_PHOTO);
                                    std::cout << " [PROCESS] Chup anh (Legacy)..." << std::endl;
                                    usleep(200000);
                                    send_image_captured();
                                } else {
                                    send_ack(MAV_CMD_LEGACY_PHOTO);
                                }
                            }
                            
                            // *** MỚI: Xử lý VIDEO commands ***
                            else if (cmd.command == MAV_CMD_VIDEO_START_CAPTURE) {
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                video_recording = true;
                                send_ack(cmd.command);
                                std::cout << " [EVENT] -> BAT DAU QUAY VIDEO!" << std::endl;
                            }
                            else if (cmd.command == MAV_CMD_VIDEO_STOP_CAPTURE) {
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                video_recording = false;
                                send_ack(cmd.command);
                                std::cout << " [EVENT] -> DUNG QUAY VIDEO!" << std::endl;
                            }
                            
                            // *** MỚI: Xử lý SET_CAMERA_MODE (530) ***
                            else if (cmd.command == MAV_CMD_SET_CAMERA_MODE) {
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                int mode = (int)cmd.param2;
                                std::cout << " [SETTING] -> Chuyen che do camera: " << mode << std::endl;
                                std::cout << "   (0=Photo, 1=Video)" << std::endl;
                                send_ack(MAV_CMD_SET_CAMERA_MODE);
                                // Gửi lại settings để confirm
                                usleep(100000);
                                send_camera_settings();
                            }
                            
                            // *** MỚI: Xử lý IMAGE_STOP_CAPTURE ***
                            else if (cmd.command == MAV_CMD_IMAGE_STOP_CAPTURE) {
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                send_ack(MAV_CMD_IMAGE_STOP_CAPTURE);
                                std::cout << " [INFO] -> Stop image capture (OK)" << std::endl;
                            }
                            
                            // *** MỚI: Xử lý REQUEST_CAMERA_SETTINGS (deprecated) ***
                            else if (cmd.command == MAV_CMD_REQUEST_CAMERA_SETTINGS) {
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                send_camera_settings();
                                send_ack(MAV_CMD_REQUEST_CAMERA_SETTINGS);
                            }
                            
                            // *** MỚI: Xử lý REQUEST_CAMERA_CAPTURE_STATUS (deprecated) ***
                            else if (cmd.command == MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS) {
                                print_incoming_command(cmd, msg.sysid, msg.compid);
                                send_camera_capture_status();
                                send_ack(MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
                            }
                            
                            // Lệnh không hỗ trợ
                            else {
                                std::cout << " [WARN] Lenh chua xu ly: " << cmd.command << std::endl;
                                send_ack(cmd.command, MAV_RESULT_UNSUPPORTED);
                            }
                        }
                    }
                }
            }
        }
        usleep(5000);
    }
    
    running = false;
    status_thread.join();
    close(sock);
    
    return 0;
}