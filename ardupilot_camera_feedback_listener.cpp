#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <vector>              // ← THÊM
#include <unordered_set>
#include <unordered_map>

// ← QUAN TRỌNG: Include ardupilotmega trước common
#include "c_library_v2/ardupilotmega/mavlink.h"

// ← THÊM: Lắng nghe NHIỀU ports
#define PORT_1 14550  // QGC port (primary)
#define PORT_2 14556  // Companion computer
#define PORT_3 14540  // Camera component

struct SocketInfo {
    int sock;
    int port;
    std::string name;
};

std::vector<SocketInfo> sockets;
std::unordered_set<int> printed_ids;
std::unordered_map<int, int> msgid_count;

void setup_socket(int port, const std::string& name) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    
    // Set non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "⚠️  Port " << port << " (" << name << ") bind failed\n";
        close(sock);
        return;
    }
    
    sockets.push_back({sock, port, name});
    std::cout << "✅ Listening on port " << port << " (" << name << ")\n";
}

void setup() {
    std::cout << "============================================\n";
    std::cout << "  MAVLink Sniffer - Multi-port Listener\n";
    std::cout << "============================================\n\n";
    
    setup_socket(PORT_1, "QGC/SITL");
    setup_socket(PORT_2, "Companion");
    setup_socket(PORT_3, "Camera");
    
    std::cout << "\nChờ messages...\n\n";
}

const char* get_message_name(uint32_t msgid) {
    switch(msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: return "HEARTBEAT";
        case MAVLINK_MSG_ID_CAMERA_FEEDBACK: return "CAMERA_FEEDBACK";
        case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED: return "CAMERA_IMAGE_CAPTURED";
        case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: return "CAMERA_CAPTURE_STATUS";
        case MAVLINK_MSG_ID_CAMERA_INFORMATION: return "CAMERA_INFORMATION";
        case MAVLINK_MSG_ID_CAMERA_SETTINGS: return "CAMERA_SETTINGS";
        case MAVLINK_MSG_ID_CAMERA_TRIGGER: return "CAMERA_TRIGGER";
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return "GLOBAL_POSITION_INT";
        case MAVLINK_MSG_ID_COMMAND_LONG: return "COMMAND_LONG";
        case MAVLINK_MSG_ID_COMMAND_ACK: return "COMMAND_ACK";
        case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: return "VIDEO_STREAM_INFORMATION";
        default: return "UNKNOWN";
    }
}

const char* get_command_name(uint16_t cmd) {
    switch(cmd) {
        case 203: return "MAV_CMD_DO_DIGICAM_CONTROL";
        case 206: return "MAV_CMD_DO_SET_CAM_TRIGG_DIST";
        case 530: return "MAV_CMD_SET_CAMERA_MODE";
        case 2000: return "MAV_CMD_IMAGE_START_CAPTURE";
        case 2001: return "MAV_CMD_IMAGE_STOP_CAPTURE";
        case 2500: return "MAV_CMD_VIDEO_START_CAPTURE";
        case 2501: return "MAV_CMD_VIDEO_STOP_CAPTURE";
        case 112: return "MAV_CMD_DO_TRIGGER_CONTROL";
        case 521: return "MAV_CMD_REQUEST_CAMERA_INFORMATION";
        case 522: return "MAV_CMD_REQUEST_CAMERA_SETTINGS";
        case 527: return "MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS";
        default: return "UNKNOWN";
    }
}

void print_camera_feedback(mavlink_message_t& msg) {
    mavlink_camera_feedback_t feedback;
    mavlink_msg_camera_feedback_decode(&msg, &feedback);
    
    std::cout << "\n╔══════════════════════════════════════════╗\n";
    std::cout << "║   📸 CAMERA_FEEDBACK (180) DETECTED!    ║\n";
    std::cout << "╚══════════════════════════════════════════╝\n";
    std::cout << "  From: sysid=" << (int)msg.sysid 
              << ", compid=" << (int)msg.compid << "\n";
    std::cout << "  Image #" << feedback.img_idx << "\n";
    std::cout << "  GPS: " << std::fixed << std::setprecision(7)
              << (feedback.lat/1e7) << ", " << (feedback.lng/1e7) << "\n";
    std::cout << "  Altitude: " << feedback.alt_rel << "m (relative)\n";
    std::cout << "  Attitude: Roll=" << feedback.roll << "°, Pitch=" 
              << feedback.pitch << "°, Yaw=" << feedback.yaw << "°\n";
    std::cout << "  Flags: 0x" << std::hex << (int)feedback.flags << std::dec << "\n";
    std::cout << "  Completed: " << feedback.completed_captures << "\n\n";
}

void print_command_long(mavlink_message_t& msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    
    std::cout << "\n╔══════════════════════════════════════════╗\n";
    std::cout << "║   🎮 COMMAND_LONG (76) DETECTED!        ║\n";
    std::cout << "╚══════════════════════════════════════════╝\n";
    std::cout << "  From: sysid=" << (int)msg.sysid 
              << ", compid=" << (int)msg.compid << "\n";
    std::cout << "  Command: " << cmd.command 
              << " (" << get_command_name(cmd.command) << ")\n";
    std::cout << "  Target: sysid=" << (int)cmd.target_system 
              << ", compid=" << (int)cmd.target_component << "\n";
    std::cout << "  Params:\n";
    std::cout << "    param1: " << cmd.param1 << "\n";
    std::cout << "    param2: " << cmd.param2 << "\n";
    std::cout << "    param3: " << cmd.param3 << "\n";
    std::cout << "    param4: " << cmd.param4 << "\n";
    std::cout << "    param5: " << cmd.param5 << "\n";
    std::cout << "    param6: " << cmd.param6 << "\n";
    std::cout << "    param7: " << cmd.param7 << "\n";
    
    // Giải thích camera commands
    if (cmd.command == 206) {  // DO_SET_CAM_TRIGG_DIST
        std::cout << "  → Trigger distance: " << cmd.param1 << " meters\n";
    } else if (cmd.command == 203) {  // DO_DIGICAM_CONTROL
        std::cout << "  → Take photo now!\n";
    } else if (cmd.command == 2000) {  // IMAGE_START_CAPTURE
        std::cout << "  → Interval: " << cmd.param2 << "s, Total: " << cmd.param3 << "\n";
    }
    std::cout << "\n";
}

void print_camera_trigger(mavlink_message_t& msg) {
    mavlink_camera_trigger_t trigger;
    mavlink_msg_camera_trigger_decode(&msg, &trigger);
    
    std::cout << "\n╔══════════════════════════════════════════╗\n";
    std::cout << "║   📷 CAMERA_TRIGGER (112) DETECTED!     ║\n";
    std::cout << "╚══════════════════════════════════════════╝\n";
    std::cout << "  From: sysid=" << (int)msg.sysid 
              << ", compid=" << (int)msg.compid << "\n";
    std::cout << "  Sequence: " << trigger.seq << "\n";
    std::cout << "  Time: " << trigger.time_usec << " μs\n\n";
}

int main() {
    setup();
    
    if (sockets.empty()) {
        std::cerr << "❌ Không bind được port nào!\n";
        return 1;
    }
    
    uint8_t buf[2048];
    mavlink_message_t msg;
    mavlink_status_t status;
    bool got_heartbeat = false;
    int camera_feedback_count = 0;
    int camera_command_count = 0;
    int camera_trigger_count = 0;
    
    while (true) {
        for (auto& s : sockets) {
            struct sockaddr_in src_addr;
            socklen_t src_len = sizeof(src_addr);
            
            int n = recvfrom(s.sock, buf, sizeof(buf), 0, 
                           (struct sockaddr*)&src_addr, &src_len);
            
            if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                        // Count messages
                        msgid_count[msg.msgid]++;
                        
                        // HEARTBEAT
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && !got_heartbeat) {
                            std::cout << "✅ HEARTBEAT from sysid=" << (int)msg.sysid 
                                     << ", compid=" << (int)msg.compid 
                                     << " (port " << s.port << ")\n\n";
                            got_heartbeat = true;
                        }
                        
                        // ← CAMERA_FEEDBACK
                        else if (msg.msgid == MAVLINK_MSG_ID_CAMERA_FEEDBACK) {
                            camera_feedback_count++;
                            print_camera_feedback(msg);
                        }
                        
                        // ← COMMAND_LONG (Lệnh camera)
                        else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                            mavlink_command_long_t cmd;
                            mavlink_msg_command_long_decode(&msg, &cmd);
                            
                            // Chỉ in camera-related commands
                            if (cmd.command == 203 || cmd.command == 206 || 
                                cmd.command == 530 || cmd.command == 2000 || 
                                cmd.command == 2001 || cmd.command == 2500 || 
                                cmd.command == 2501 || cmd.command == 112 ||
                                cmd.command == 521 || cmd.command == 522 || 
                                cmd.command == 527) {
                                camera_command_count++;
                                print_command_long(msg);
                            }
                        }
                        
                        // ← CAMERA_TRIGGER
                        else if (msg.msgid == MAVLINK_MSG_ID_CAMERA_TRIGGER) {
                            camera_trigger_count++;
                            print_camera_trigger(msg);
                        }
                        
                        // CAMERA_IMAGE_CAPTURED
                        else if (msg.msgid == MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED) {
                            mavlink_camera_image_captured_t cap;
                            mavlink_msg_camera_image_captured_decode(&msg, &cap);
                            std::cout << "[INFO] 📸 CAMERA_IMAGE_CAPTURED: index=" 
                                     << cap.image_index << " from compid=" 
                                     << (int)msg.compid << "\n";
                        }
                        
                        // CAMERA_CAPTURE_STATUS
                        else if (msg.msgid == MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) {
                            mavlink_camera_capture_status_t status;
                            mavlink_msg_camera_capture_status_decode(&msg, &status);
                            
                            if (msgid_count[msg.msgid] % 10 == 1) {  // Print mỗi 10 lần
                                std::cout << "[INFO] 📊 CAPTURE_STATUS: images=" 
                                         << status.image_count 
                                         << ", video=" << (int)status.video_status << "\n";
                            }
                        }
                        
                        // DEBUG: Print first occurrence
                        else if (printed_ids.count(msg.msgid) == 0) {
                            std::cout << "[DEBUG] Message ID " << msg.msgid 
                                     << " (" << get_message_name(msg.msgid) 
                                     << ") from port " << s.port << "\n";
                            printed_ids.insert(msg.msgid);
                        }
                        // Thêm vào phần xử lý message
                        else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                            mavlink_command_ack_t ack;
                            mavlink_msg_command_ack_decode(&msg, &ack);
                            
                            if (ack.command == 203 || ack.command == 2000 || ack.command == 206) {
                                std::cout << "\nCOMANDO_ACEPTADO → ";
                                if (ack.command == 203) std::cout << "DO_DIGICAM_CONTROL (Chụp ngay)\n";
                                if (ack.command == 2000) std::cout << "IMAGE_START_CAPTURE (Bắt đầu chụp định kỳ)\n";
                                if (ack.command == 206)  std::cout << "DO_SET_CAM_TRIGG_DIST (Trigger theo khoảng cách)\n";
                                
                                std::cout << "   Result: " << (int)ack.result 
                                        << " (" << (ack.result == 0 ? "ACCEPTED" : "REJECTED") << ")\n\n";
                            }
                        }
                    }
                }
            }
        }
        
        usleep(1000);  // 1ms
    }
    
    return 0;
}