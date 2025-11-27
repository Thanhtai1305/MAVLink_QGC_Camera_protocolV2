#!/usr/bin/env python3
"""
Simple HTTP server để phục vụ camera_definition.xml
Chạy trên port 8000
"""

from http.server import HTTPServer, SimpleHTTPRequestHandler
import os

class CameraXMLHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/camera_definition.xml':
            if os.path.exists('camera_definition.xml'):
                self.send_response(200)
                self.send_header('Content-type', 'application/xml')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                
                with open('camera_definition.xml', 'rb') as f:
                    self.wfile.write(f.read())
                
                print(f"✓ Phục vụ camera_definition.xml cho {self.client_address[0]}")
            else:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b'File not found')
                print(f"✗ Không tìm thấy camera_definition.xml")
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        # Tắt log mặc định
        pass

def main():
    port = 8000
    print("="*60)
    print("   HTTP Server cho Camera Definition XML")
    print("="*60)
    print(f"Port: {port}")
    print(f"URL:  http://127.0.0.1:{port}/camera_definition.xml")
    print("="*60)
    
    if not os.path.exists('camera_definition.xml'):
        print("\n⚠️  CẢNH BÁO: Không tìm thấy camera_definition.xml")
        print("   Tạo file camera_definition.xml trong thư mục này\n")
    else:
        print(f"\n✓ Tìm thấy camera_definition.xml")
        print(f"  Kích thước: {os.path.getsize('camera_definition.xml')} bytes\n")
    
    print("Đang chạy... Nhấn Ctrl+C để dừng\n")
    
    server = HTTPServer(('0.0.0.0', port), CameraXMLHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nĐã dừng HTTP server")
        server.shutdown()

if __name__ == '__main__':
    main()