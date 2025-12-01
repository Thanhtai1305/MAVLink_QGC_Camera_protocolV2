// simple_camera_listener.cpp
// Chỉ nhận lệnh chụp ảnh từ ArduPilot (port 14540) → in log rõ ràng

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "c_library_v2/common/mavlink.h"

#define LISTEN_PORT 14540   // ← ArduPilot gửi về đây khi dùng --serial2=udpclient:127.0.0.1:14540

int sock;
struct sockaddr_in my_addr, src_addr;

void setup_udp() {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    my_addr.sin_family = AF_INET;
    my_addr.sin_addr.s_addr = INADDR_ANY;
    my_addr.sin_port = htons(LISTEN_PORT);
    bind(sock, (struct sockaddr*)&my_addr, sizeof(my_addr));
    std::cout << "Camera Listener đang nghe ở port " << LISTEN_PORT << " (nhận lệnh từ ArduPilot)\n";
}

int main() {
    setup_udp();
    uint8_t buf[1024];
    mavlink_message_t msg;
    mavlink_status_t status;

    std::cout << "ĐANG CHỜ LỆNH CHỤP ẢNH TỪ ARDUPILOT...\n\n";

    while (true) {
        int recsize = recvfrom(sock, buf, sizeof(buf), 0, nullptr, nullptr);
        if (recsize > 0) {
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);

                        std::cout << "NHẬN LỆNH TỪ FC (sys:" << (int)msg.sysid 
                                  << " comp:" << (int)msg.compid << ")\n";

                        if (cmd.command == MAV_CMD_DO_SET_CAM_TRIGG_DIST && cmd.param1 > 0) {
                            std::cout << "AUTO CAPTURE BẬT – Khoảng cách: " 
                                      << cmd.param1 << " mét\n";
                        }
                        else if (cmd.command == MAV_CMD_IMAGE_START_CAPTURE) {
                            std::cout << "CHỤP ẢNH LIỀN TỤC – Số ảnh: " 
                                      << (int)cmd.param2 << " | Khoảng cách: " 
                                      << cmd.param3 << " giây\n";
                        }
                        else if (cmd.command == MAV_CMD_DO_DIGICAM_CONTROL || 
                                 cmd.command == MAV_CMD_IMAGE_START_CAPTURE) {
                            std::cout << "CHỤP ẢNH ĐƠN! (Legacy hoặc Start Capture)\n";
                        }
                    }
                    else if (msg.msgid == MAVLINK_MSG_ID_CAMERA_TRIGGER) {
                        mavlink_camera_trigger_t trig;
                        mavlink_msg_camera_trigger_decode(&msg, &trig);
                        std::cout << "CAMERA_TRIGGER NHẬN ĐƯỢC! (seq=" 
                                  << trig.seq << ") → CHỤP ẢNH NGAY!\n";
                    }
                }
            }
        }
        usleep(1000);
    }
    return 0;
}