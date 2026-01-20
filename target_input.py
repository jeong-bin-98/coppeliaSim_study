"""
타겟 좌표 입력 클라이언트
터미널에서 실행하여 좌표를 입력하면 inverseKinematics_sim.py로 전송
"""
import socket

HOST = '127.0.0.1'
PORT = 5555

def main():
    print("=" * 50)
    print("  타겟 좌표 입력 클라이언트")
    print("=" * 50)
    print("[사용법] x y z 형식으로 입력 (예: 0.3 0.2 0.4)")
    print("[종료] q 입력")
    print()

    while True:
        try:
            user_input = input(">> 좌표 입력: ").strip()
            
            if user_input.lower() == 'q':
                print("종료합니다.")
                break
            
            # 입력값 검증
            coords = user_input.split()
            if len(coords) != 3:
                print("[오류] 형식: x y z (예: 0.3 0.2 0.4)")
                continue
            
            try:
                x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
            except ValueError:
                print("[오류] 숫자를 입력해주세요.")
                continue
            
            # 서버로 전송
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(2)
                    s.connect((HOST, PORT))
                    message = f"{x} {y} {z}"
                    s.sendall(message.encode())
                    print(f"[전송 완료] ({x}, {y}, {z})")
            except ConnectionRefusedError:
                print("[오류] 시뮬레이션 서버가 실행 중이 아닙니다.")
            except socket.timeout:
                print("[오류] 연결 시간 초과")
                
        except KeyboardInterrupt:
            print("\n종료합니다.")
            break

if __name__ == "__main__":
    main()
