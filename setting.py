import numpy as np
ACCEPTED_DELAY = 1 #可接受的通訊延遲 sec
BASE_SPEED=7  #base 在guided mode 下的移動速度m/sec
ROVER_SPEED_LIMIT=6
SEND_INTERVAL = 1# base 發送座標間隔 sec
SLEEP_LENGTH = 0.3 # rover接收座標並進行飛行控制間隔
FORMATION=0 #設定隊形
SPEED_THRESHOLD=2 # 判定base 是否靜止
KP=1
KD=-0.1
K1 = np.array([[0.6, 0], [0, 0.6]])  # Acts on the position error (Proportional Gain)
damping = 0.2  # Damping factor to control high frequency velocity changes (overshoot and oscillations)
max_velocity=10
"""
隊形(formation)包括:
    0. Line: 1台rover 跟在 base 飛行方向的後方
    1. Wedge: 倒 V 型，base 帶領2台rover, 
    2. Square: base在中間，4台rover在方形的四個角上
"""