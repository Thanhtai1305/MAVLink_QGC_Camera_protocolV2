#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <iomanip> 
#include "mavlink/common/mavlink.h"

#define QGC_IP "127.0.0.1"
#define QGC_PORT 14550 
#define MY_PORT 14540 

// --- Dung ID 1 ---
#define SYS_ID 1            
#define COMP_ID_CAMERA 100 

// Cac lenh MAVLink
#define MAV_CMD_LEGACY_PHOTO 203 
#define MAV_CMD_REQUEST_CAMERA_INFORMATION 521 
#define MAV_CMD_REQUEST_STORAGE_INFORMATION 261
#define MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS 262

int sock;
struct sockaddr_in qgcAddr;
struct sockaddr_in myAddr;

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
   
    std::cout << "   DRONE CAMERA SIMULATOR   " << std::endl;
    std::cout << "   Port: " << MY_PORT << " | Target: " << QGC_PORT << std::endl;
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

    std::cout << " [INFO] Dang gui Heartbeat ... " << std::endl;
}

void send_ack(uint16_t command, uint8_t result = MAV_RESULT_ACCEPTED) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(SYS_ID, COMP_ID_CAMERA, &msg, command, result, 0, 0, 0, 0);
    send_mavlink(&msg);
}

// 1. Gui XML URI
void send_camera_information() {
    mavlink_message_t msg;
    const char* uri = "http://127.0.0.1:8000/camera_test.xml"; 
    uint8_t v[32] = "FullOptionCam"; uint8_t m[32] = "Ultra_V2";
    uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE | CAMERA_CAP_FLAGS_CAPTURE_VIDEO | CAMERA_CAP_FLAGS_HAS_MODES;

    // Su dung cach nay an toan hon va tranh truong hop BI THIEU THAM SO
    mavlink_camera_information_t cam_info;
    memset(&cam_info, 0, sizeof(cam_info));
    
    cam_info.time_boot_ms = get_time_boot_ms();
    memcpy(cam_info.vendor_name, v, 32);
    memcpy(cam_info.model_name, m, 32);
    cam_info.firmware_version = 1;
    cam_info.focal_length = 50.0f;
    cam_info.sensor_size_h = 10.0f;
    cam_info.sensor_size_v = 10.0f;
    cam_info.resolution_h = 1920;
    cam_info.resolution_v = 1080;
    cam_info.flags = flags;
    cam_info.cam_definition_version = 1;
    strncpy(cam_info.cam_definition_uri, uri, sizeof(cam_info.cam_definition_uri) - 1);

    mavlink_msg_camera_information_encode(SYS_ID, COMP_ID_CAMERA, &msg, &cam_info);
    send_mavlink(&msg);
    std::cout << " [INFO] Gui Camera Info (XML Link)" << std::endl;
}

// 2. Gui thong tin the nho
void send_storage_information() {
    mavlink_message_t msg;
    
    // --- SUA LOI: Dung struct + encode ---
    mavlink_storage_information_t storage;
    memset(&storage, 0, sizeof(storage)); // Tu dong dien 0 vao cac tham so la

    storage.time_boot_ms = get_time_boot_ms();
    storage.storage_id = 1;
    storage.storage_count = 1;
    storage.status = 2; // Formatted/Ready
    storage.total_capacity = 1000.0f; // MB
    storage.used_capacity = 10.0f;
    storage.available_capacity = 990.0f;
    storage.read_speed = 10.0f;
    storage.write_speed = 10.0f;
    
    mavlink_msg_storage_information_encode(SYS_ID, COMP_ID_CAMERA, &msg, &storage);
    send_mavlink(&msg);
}

// 3. Gui trang thai camera
void send_capture_status() {
    mavlink_message_t msg;
    
    // --- SUA LOI: Dung struct + encode ---
    mavlink_camera_capture_status_t cap_status;
    memset(&cap_status, 0, sizeof(cap_status)); // Tu dong dien 0 vao cac tham so la

    cap_status.time_boot_ms = get_time_boot_ms();
    cap_status.image_status = 0; // Idle
    cap_status.video_status = 0; // Idle
    cap_status.image_interval = 0;
    cap_status.recording_time_ms = 0;
    cap_status.available_capacity = 990.0f;

    mavlink_msg_camera_capture_status_encode(SYS_ID, COMP_ID_CAMERA, &msg, &cap_status);
    send_mavlink(&msg);
}

void send_image_captured() {
    mavlink_message_t msg;
    float q[4] = {1,0,0,0};
    mavlink_msg_camera_image_captured_pack(SYS_ID, COMP_ID_CAMERA, &msg,
        get_time_boot_ms(), 0, 0, 0, 0, 0, 0, q, 1, 1, "http://img.jpg");
    send_mavlink(&msg);
    std::cout << " [EVENT] -> CHUP ANH THANH CONG (IMAGE CAPTURED) !!!" << std::endl;
}

void print_incoming_command(mavlink_command_long_t& cmd) {
    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << " [RECV] COMMAND ID: " << cmd.command;
    if (cmd.command == MAV_CMD_IMAGE_START_CAPTURE) std::cout << " (IMAGE_START_CAPTURE - V2)";
    else if (cmd.command == MAV_CMD_LEGACY_PHOTO) std::cout << " (LEGACY PHOTO)";
    std::cout << std::endl;
}

int main() {
    setup_udp();
    
    // Gui lien tuc 5 heartbeat cho QGC
    for(int i=0; i<5; i++) { send_heartbeat(); usleep(50000); }

    int counter = 0;

    while (true) {
        // Chu ky gui cac goi tin
        if (counter % 200 == 0) send_heartbeat(); // 1Hz
        
        // Gui lien tuc cac goi nay de QGC biet camera SAN SANG
        if (counter % 600 == 0) { // 3s một lần
            send_camera_information();
            send_storage_information(); 
            send_capture_status();      
        }
        
        counter++;

        // Nhan du lieu
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        struct sockaddr_in src; socklen_t len = sizeof(src);
        ssize_t recsize = recvfrom(sock, buf, MAVLINK_MAX_PACKET_LEN, 0, (struct sockaddr *)&src, &len);

        if (recsize > 0) {
            mavlink_message_t msg; mavlink_status_t status;
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);

                        if (cmd.target_component == COMP_ID_CAMERA || cmd.target_component == 0) {
                            
                            // 1. Handshake Info
                            if (cmd.command == MAV_CMD_REQUEST_CAMERA_INFORMATION || 
                                (cmd.command == MAV_CMD_REQUEST_MESSAGE && (int)cmd.param1 == MAVLINK_MSG_ID_CAMERA_INFORMATION)) {
                                send_camera_information();
                                send_ack(cmd.command);
                            }
                            // 2. Handshake Storage
                            else if (cmd.command == MAV_CMD_REQUEST_STORAGE_INFORMATION) {
                                send_storage_information();
                                send_ack(cmd.command);
                            }
                            // 3. Handshake Status
                            else if (cmd.command == MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS) {
                                send_capture_status();
                                send_ack(cmd.command);
                            }
                            // 4. Chup anh V2
                            else if (cmd.command == MAV_CMD_IMAGE_START_CAPTURE) {
                                print_incoming_command(cmd);
                                send_ack(MAV_CMD_IMAGE_START_CAPTURE);
                                std::cout << " [PROCESS] Dang chup anh V2..." << std::endl;
                                usleep(200000); 
                                send_image_captured();
                            }
                            // 5. Chup anh V1 - legacy
                            else if (cmd.command == MAV_CMD_LEGACY_PHOTO) { 
                                if ((int)cmd.param5 == 1) {
                                    print_incoming_command(cmd);
                                    send_ack(MAV_CMD_LEGACY_PHOTO);
                                    std::cout << " [PROCESS] Dang chup anh Legacy..." << std::endl;
                                    usleep(200000);
                                    send_image_captured();
                                }
                            }
                        }
                    }
                }
            }
        }
        usleep(5000); // Ngu sau 5ms
    }
    return 0;
}