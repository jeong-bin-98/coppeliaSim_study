import os
import math
import time
import numpy as np
import sim
import simConst
from dotenv import load_dotenv

from coppeliasim_client import Coppeliasim_client

load_dotenv()

def main():
    client = Coppeliasim_client()
    client.check_connection()
    client.initialize_handles()
    
    # --- 2. 시뮬레이션 시작 ---
    sim.simxStartSimulation(client.client_id, simConst.simx_opmode_blocking)
    client.start_streaming() # 데이터 전송 요청

    print(">> Simulation Started")
    time.sleep(1) # 데이터가 안정적으로 들어올 때까지 잠시 대기

    try:
        while True:
            # --- 3. 데이터 획득 (Synchronized) ---
	        # 관절 각도와 실제 TCP 위치를 동시에 가져옴
            angles, sim_tcp_world = client.get_data_synchronized()

            # 데이터가 아직 안 들어왔으면 건너뜀 (Safety)
            if len(angles) != 6: 
                continue 

            # 결과 출력
            print("-" * 50)
            print(f"Sim TCP(World): X={sim_tcp_world[0]:.4f}, Y={sim_tcp_world[1]:.4f}, Z={sim_tcp_world[2]:.4f}")

            # 루프 속도 조절
            time.sleep(0.5) 

    except KeyboardInterrupt:
        print("\n>> 종료 요청 받음.")
    
    finally:
        # --- 4. 안전 종료 ---
        sim.simxStopSimulation(client.client_id, simConst.simx_opmode_blocking)
        sim.simxFinish(client.client_id)
        print(">> Remote API 연결 종료")

if __name__ == "__main__":
    main()