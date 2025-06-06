ACCEPTED_DELAY = 1 #可接受的通訊延遲 sec
BASE_SPEED=6  #base 在guided mode 下的移動速度m/sec
ROVER_SPEED_LIMIT=8
SEND_INTERVAL = 0.5# base 發送座標間隔 sec
SLEEP_LENGTH = 0.5 # rover接收座標並進行飛行控制間隔
FORMATION=0 #設定隊形
SPEED_THRESHOLD=3 # 判定base 是否靜止
"""
隊形(formation)包括:
    0. Line: 1台rover 跟在 base 飛行方向的後方
    1. Wedge: 倒 V 型，base 帶領2台rover, 
    2. Square: base在中間，4台rover在方形的四個角上
"""