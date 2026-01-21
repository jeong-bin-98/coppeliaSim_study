"""
소켓 서버 모듈
클라이언트로부터 좌표를 수신하여 목표 타겟 위치를 업데이트
"""
import socket
import threading
import os
from dotenv import load_dotenv

load_dotenv()

class SocketServer:
    """소켓 서버: 클라이언트로부터 좌표를 수신"""
    
    def __init__(self, host=os.getenv('SOCKET_HOST'), port=os.getenv('SOCKET_PORT')):
        self.host = host
        self.port = port
        self.running = False
        self.server = None
        self.thread = None
        self._goal_target = None  # numpy array reference
    
    def start(self, goal_target):
        """
        소켓 서버 쓰레드 시작
        
        Args:
            goal_target: 목표 좌표를 저장할 numpy array (참조로 업데이트됨)
        """
        self._goal_target = goal_target
        self.running = True
        self.thread = threading.Thread(target=self._server_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """소켓 서버 종료"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        print("[서버] 소켓 서버 종료")
    
    def _server_loop(self):
        """소켓 서버 메인 루프"""
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.server.settimeout(1)
        
        print(f"[서버] 소켓 서버 시작 (포트: {self.port})")
        
        while self.running:
            try:
                conn, addr = self.server.accept()
                with conn:
                    data = conn.recv(1024).decode().strip()
                    if data:
                        coords = data.split()
                        if len(coords) == 3:
                            self._goal_target[0] = float(coords[0])
                            self._goal_target[1] = float(coords[1])
                            self._goal_target[2] = float(coords[2])
                            print(f"\n[수신] 새 목표 좌표: ({self._goal_target[0]:.4f}, {self._goal_target[1]:.4f}, {self._goal_target[2]:.4f})")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[서버 오류] {e}")
        
        self.server.close()
