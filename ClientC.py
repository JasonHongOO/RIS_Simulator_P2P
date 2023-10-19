import sys, struct, time, json, signal,random 
import socket
import threading

import datetime
import Const

# ===========================================
# Socker 參數
NAME = "Move_Simulator" 

client_sockets = {}
Thread_Record = []
send_xApp_thread = 0
send_GUI_thread = 0

# 全域變數
running = True
Cur_Case_Value = 0
Cur_Angle_Value = 0
Target_Angle_Value = 0
Cur_RSRP_Value = 0

server_socket = 0
sender_client_socket = {}
# ===========================================

def signal_handler(sig, frame):
    print("接收到Ctrl+C信號，正在停止服務器...")
    global running, send_xApp_thread, send_GUI_thread, server_socket, client_socket
    running = False

    # 執行清理操作，例如關閉套接字和停止線程
    for Thread in Thread_Record:
        Thread.join()

    send_xApp_thread.join()
    send_GUI_thread.join()

    server_socket.close()
    for client_socket in sender_client_socket:
        sender_client_socket[client_socket].close()
    sys.exit(0)

class MoveSimulator:
    def __init__(self):
        self.increment = Const.STEP_OFFEST_BASE
        self.timer = 0
        self.timer_Max = 1
        self.State = 0

    def Start(self):
        self.Updata()

    def Updata(self):
        global running, Cur_Case_Value, Cur_Angle_Value, Target_Angle_Value, Cur_RSRP_Value

        while running:
            if Cur_Angle_Value != Target_Angle_Value:
                if Cur_Angle_Value < Target_Angle_Value:
                    Cur_Angle_Value += self.increment + (random.randint(Const.STEP_OFFEST_MIN, Const.STEP_OFFEST_MAX) if Const.STEP_RANDOM_ACTIVATE == True else 0)
                else:
                    Cur_Angle_Value -= self.increment + (random.randint(Const.STEP_OFFEST_MIN, Const.STEP_OFFEST_MAX) if Const.STEP_RANDOM_ACTIVATE == True else 0)

                if Cur_Angle_Value > Const.ANGLE_MAX: Cur_Angle_Value = Const.ANGLE_MAX
                elif Cur_Angle_Value < Const.ANGLE_MIN: Cur_Angle_Value = Const.ANGLE_MIN
                    

            # 隨機移動
            # if Cur_Angle_Value == 0 and self.State == 0: 
            #     if self.timer < 8 : self.timer += 1
            #     else:
            #         self.State = 1
            #         self.timer = 0
            #         positive_random_int = random.randint(0, 60)
            #         if positive_random_int != 0: random_integer = positive_random_int * random.choice([1, -1])
            #         else : random_integer = 0
            #         Target_Angle_Value = random_integer

            # elif self.State == 1: 
            #     if self.timer < self.timer_Max : self.timer += 1
            #     else:
            #         self.State = 2
            #         self.timer = 0
                
            # elif self.State == 2: 
            #     self.State = 1
            #     positive_random_int = random.randint(0, 60)
            #     if positive_random_int != 0: random_integer = positive_random_int * random.choice([1, -1])
            #     else : random_integer = 0
            #     Target_Angle_Value = random_integer

                    
            # 0 -> 60 -> 0
            if Cur_Angle_Value == 0 and self.State == 0: 
                if self.timer < self.timer_Max : 
                    self.timer += 1
                else:
                    self.State = 1
                    Target_Angle_Value = 20
                    self.timer = 0
            elif Cur_Angle_Value == 20 and self.State == 1: 
                if self.timer < self.timer_Max : self.timer += 1
                else:
                    self.State = 2
                    Target_Angle_Value = 0
                    self.timer = 0
            elif Cur_Angle_Value == 0 and self.State == 2: 
                if self.timer < self.timer_Max : self.timer += 1
                else:
                    self.State = 3
                    self.timer = 0

            # print(f"self.timer : {self.timer}")

            time.sleep(random.uniform(2, 3)) 

def ReadJsonData():
    Data_Fixed_Json_Path = Const.DATA_FILE
    with open(Data_Fixed_Json_Path, 'r') as f:
        Data_Fixed_Json = json.load(f)

    Data_Fixed_Behind_Json_Path = Const.DATA_BEHIND_FILE
    with open(Data_Fixed_Behind_Json_Path, 'r') as f:
        Data_Fixed_Behind_Json = json.load(f)

    return Data_Fixed_Json, Data_Fixed_Behind_Json

def send_message(client_socket, message):
    message_json = json.dumps(message)
    message_length = len(message_json)
    client_socket.send(struct.pack('!I', message_length))  # 發送消息長度
    client_socket.send(message_json.encode('utf-8'))  # 發送消息內容

def Receiver(client_socket, client_name):
    global running, Cur_Case_Value, Cur_Angle_Value, Target_Angle_Value, Cur_RSRP_Value

    client_address = client_socket.getpeername()
    print(f"\n與客戶端 {client_name} ({client_address}) 連接成功")
    client_sockets[client_name] = client_socket

    try:
        while running == True:
            try:
                message_length_data = client_socket.recv(4)
                if not message_length_data: break

                message_length = struct.unpack('!I', message_length_data)[0]
                data = client_socket.recv(message_length)
                if not data: break

                # 處理來自客戶端的消息
                received_data = json.loads(data.decode('utf-8'))
                print(f"來自 {client_name} 給 {received_data['Receiver_Name']} 的消息: {received_data['Msg']}")
                print(received_data)

                # 儲存訊息
                if received_data['Sender_Name'] == "xApp":
                    print(f"Cur_Case_Value : {received_data['Cur_Case_Value']}")
                    Cur_Case_Value = received_data['Cur_Case_Value']
                    print(f"Cur_Case_Value : {Cur_Case_Value}")

                elif received_data['Sender_Name'] == "GUI":
                    print(f"Target_Angle_Value : {received_data['Target_Angle_Value']}")
                    Target_Angle_Value = received_data['Target_Angle_Value']

                # time.sleep(Const.SERVER_INTERVAL)
            except socket.timeout:
                print("等待訊息超時 => 休息一下")
                time.sleep(Const.C_RECEIVER_INTERVAL)

    except Exception as e:
        print(f"與客戶端 {client_name} 發生錯誤: {str(e)}")
 
    client_socket.close()
    print(f"與客戶端 {client_name} 斷開連接")

def MOVER_Server():
    global server_socket

    host = '0.0.0.0'
    port = Const.MOVER_PORT

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.settimeout(5)
    server_socket.listen(10)        # queue

    # 要連線的清單 [xApp, GUI]
    xApp_Checker = False
    GUI_Checker = False
    while True:
        try:
            print("等待客戶端連接...")
            client_socket, client_address = server_socket.accept()

            message_length_data = client_socket.recv(4)
            message_length = struct.unpack('!I', message_length_data)[0]
            client_name = client_socket.recv(message_length).decode('utf-8')  # 接收客戶端發送的用戶名
            client_handler = threading.Thread(target=Receiver, args=(client_socket, client_name))
            Thread_Record.append(client_handler)
            client_handler.start()

            if client_name == "xApp":
                xApp_Checker = True
            elif client_name == "GUI":
                GUI_Checker = True

            if xApp_Checker == True and GUI_Checker == True:
                break
        except socket.timeout:
            print("等待 Client 連線時間超過 => 休息一下")
            time.sleep(1)
            pass

def Sender(Server_Name, PORT):
    global running, Cur_Case_Value, Cur_Angle_Value, Target_Angle_Value, Cur_RSRP_Value
    global sender_client_socket

    host = '127.0.0.1'
    port = PORT

    sender_client_socket[Server_Name] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while running == True:
        try :
            sender_client_socket[Server_Name].connect((host, port))
            break
        except Exception as e : 
            print(f"ERROR : XAPP Server 開未開啟 => 休息一下")
        time.sleep(1)
    sender_client_socket[Server_Name].settimeout(5)

    client_name = NAME
    message_length = len(client_name)
    sender_client_socket[Server_Name].send(struct.pack('!I', message_length))  # 發送消息長度
    sender_client_socket[Server_Name].send(client_name.encode('utf-8'))  # 發送用戶名給服務器


    while running == True:
        if Server_Name == "xApp":
            # =============================================
            #                     xApp
            # =============================================
            CaseData, CaseDataBehind = ReadJsonData()
            Cur_RSRP_Value = CaseDataBehind['Case'][str(Cur_Case_Value)]['Angle'][str(Cur_Angle_Value)]
            Cur_RSRP_Value = Cur_RSRP_Value + (random.randint(Const.RSRP_OFFEST_MIN, Const.RSRP_OFFEST_MAX) if Const.RSRP_FLUCTUATING_ACTIVATE == True else 0) * random.choice([1, -1])

            message = {}
            message["Sender_Name"] = NAME
            message["Receiver_Name"] = []
            message["Receiver_Name"].append("xApp")
            message["Msg"] = "Move Info"
            message['Cur_RSRP_Value'] = Cur_RSRP_Value
            message['Cur_Angle_Value'] = Cur_Angle_Value
            message['Cur_Case_Value'] = Cur_Case_Value
            send_message(sender_client_socket[Server_Name], message)
            
        elif Server_Name == "GUI":
            # =============================================
            #                   GUI
            # =============================================
            message = {}
            message["Sender_Name"] = NAME
            message["Receiver_Name"] = []
            message["Receiver_Name"].append("GUI")
            message["Msg"] = "Move Info"
            message['Cur_Angle_Value'] = Cur_Angle_Value
            message['Target_Angle_Value'] = Target_Angle_Value
            message['Cur_RSRP_Value'] = Cur_RSRP_Value
            send_message(sender_client_socket[Server_Name], message)

        time.sleep(Const.C_SENDER_INTERVAL)

def Client():
    global send_xApp_thread, send_GUI_thread
    send_xApp_thread = threading.Thread(target=Sender, args=("xApp", Const.XAPP_PORT,))
    send_xApp_thread.start()

    send_GUI_thread = threading.Thread(target=Sender, args=("GUI", Const.GUI_PORT,))
    send_GUI_thread.start()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    Client()
    MOVER_Server()

    print("==================================================")
    print("             MoveSimulator Start")
    print("==================================================")
    
    Mover = MoveSimulator()
    Mover.Start()
