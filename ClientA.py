import os, sys, struct, time, json, signal,random
import socket
import threading 

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from PIL import Image
import customtkinter
import datetime
import numpy as np
import Const

# ===========================================
# Socker 參數
NAME = "GUI"

client_sockets = {}
Thread_Record = []
send_mover_thread = 0

# 全域變數
running = True
Cur_Case_Value = 0
Cur_Angle_Value = 0
Target_Angle_Value = 0
Cur_RSRP_Value = 0

server_socket = 0
client_socket = 0

# xApp
Angle_Kalman_PredictResult = 0
Angle_Kalman_PrePredictResult = 0
RSRP_Kalman_PredictResult = 0
RSRP_Kalman_PrePredictResult = 0

# ===========================================

def signal_handler(sig, frame):
    print("接收到Ctrl+C信號，正在停止服務器...")
    global running, Cur_Case_Value, Cur_Angle_Value, Target_Angle_Value, Cur_RSRP_Value, Angle_Kalman_PredictResult, Angle_Kalman_PrePredictResult, RSRP_Kalman_PredictResult, RSRP_Kalman_PrePredictResult
    running = False

    # 執行清理操作，例如關閉套接字和停止線程
    send_mover_thread.join()

    server_socket.close()
    client_socket.close()
    sys.exit(0)
 
customtkinter.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"
class GUI(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        global Cur_Case_Value, Cur_Angle_Value, Target_Angle_Value, Cur_RSRP_Value
        self.CaseData, self.CaseDataBehind = ReadJsonData()

        # ===============  Variable  ===============
        self.resizable_frame_visible = False
        self.UpdataState = 0    # 0 update, 1 arrival 

        # ===============  Image  ===============
        image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "Image")
        self.Menu_image = customtkinter.CTkImage(light_image=Image.open(os.path.join(image_path, "White/menu.png")),
                                                dark_image=Image.open(os.path.join(image_path, "White/menu.png")), size=(20, 20))

        # ===============  Main Frame  ===============
        self.title("Jason Hong Simulator")
        self.geometry(f"{800}x{580}")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.MainFrame = customtkinter.CTkFrame(self, corner_radius=0)
        self.MainFrame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.MainFrame.grid_columnconfigure(0, weight=1)
        self.MainFrame.grid_rowconfigure((1), weight=1)

        # Input Entry
        self.InputFrame = customtkinter.CTkFrame(self.MainFrame)
        self.InputFrame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        self.InputFrame.grid_columnconfigure((0,1), weight=1)
        self.InputFrame.grid_columnconfigure((2), weight=0)
        self.InputFrame.grid_rowconfigure((0), weight=1)

        self.InputEntry = customtkinter.CTkEntry(self.InputFrame, placeholder_text="Input Degree")
        self.InputEntry.grid(row=0, column=0, columnspan=2, padx=10, pady=15, sticky="nsew")

        self.InputConfrimBtn = customtkinter.CTkButton(master=self.InputFrame, border_width=2, text="Input Angle", command=lambda: self.Confirm_event("Input Angle"))
        self.InputConfrimBtn.grid(row=0, column=2, padx=10, pady=15, sticky="nsew")

        self.LogBtn = customtkinter.CTkButton(master=self.InputFrame, border_width=2, text="Log", image=self.Menu_image, compound="left",  command=lambda: self.Log_Frame_event("Log Frame"))
        self.LogBtn.grid(row=0, column=3, padx=10, pady=15, sticky="nsew")

        # ===============  Body Frame  ===============
        self.BodyFrame = customtkinter.CTkFrame(self.MainFrame, corner_radius=0)
        self.BodyFrame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        self.BodyFrame.grid_columnconfigure((0,1), weight=1)
        self.BodyFrame.grid_rowconfigure((0), weight=1)

        # ===============  Left Frame  ===============
        self.LeftFrame = customtkinter.CTkFrame(self.BodyFrame, corner_radius=0)
        self.LeftFrame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.LeftFrame.grid_columnconfigure(0, weight=1)
        self.LeftFrame.grid_rowconfigure((1,2,3,4), weight=1)

        # Target Angle
        self.TargetAngleFrame = customtkinter.CTkFrame(self.LeftFrame, corner_radius=0)
        self.TargetAngleFrame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        self.TargetAngleFrame.grid_columnconfigure(0, weight=1)
        self.TargetAngleFrame.grid_rowconfigure(0, weight=1)

        self.TargetAngleLabel = customtkinter.CTkLabel(self.TargetAngleFrame, text="Target Angle : None", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.TargetAngleLabel.grid(row=0, column=0, padx=(0,0), pady=10, sticky="nsew")

        # Currnt Angle
        self.CurAngleFrame = customtkinter.CTkFrame(self.LeftFrame, corner_radius=0)
        self.CurAngleFrame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
        self.CurAngleFrame.grid_columnconfigure(0, weight=1)
        self.CurAngleFrame.grid_rowconfigure(0, weight=1)

        self.CurAngleLabel = customtkinter.CTkLabel(self.CurAngleFrame, text="Current Angle : None", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.CurAngleLabel.grid(row=0, column=0, padx=(0,0), pady=10, sticky="nsew")

        # Currnt Case
        self.CurCaseFrame = customtkinter.CTkFrame(self.LeftFrame, corner_radius=0)
        self.CurCaseFrame.grid(row=3, column=0, padx=5, pady=5, sticky="nsew")
        self.CurCaseFrame.grid_columnconfigure(0, weight=1)
        self.CurCaseFrame.grid_rowconfigure(0, weight=1)

        self.CurCaseLabel = customtkinter.CTkLabel(self.CurCaseFrame, text="Current Case : None", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.CurCaseLabel.grid(row=0, column=0, padx=(0,0), pady=10, sticky="nsew")

        # Currnt RSRP
        self.RSRPFrame = customtkinter.CTkFrame(self.LeftFrame, corner_radius=0)
        self.RSRPFrame.grid(row=4, column=0, padx=5, pady=5, sticky="nsew")
        self.RSRPFrame.grid_columnconfigure(0, weight=1)
        self.RSRPFrame.grid_rowconfigure(0, weight=1)

        self.RSRPLabel = customtkinter.CTkLabel(self.RSRPFrame, text="Current RSRP (case 0) : None", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.RSRPLabel.grid(row=0, column=0, padx=(0,0), pady=10, sticky="nsew")

        # ===============  Right Frame  ===============
        self.RightFrame = customtkinter.CTkFrame(self.BodyFrame, corner_radius=0)
        self.RightFrame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        self.RightFrame.grid_columnconfigure(0, weight=1)
        self.RightFrame.grid_rowconfigure((1,2,4,5), weight=1)

        # Predict Angle Label
        self.PredictRSRPLabel = customtkinter.CTkLabel(self.RightFrame, text="Angle Prediction : ", font=customtkinter.CTkFont(size=10, weight="bold"))
        self.PredictRSRPLabel.grid(row=0, column=0, padx=(0,0), pady=(2,0), sticky="ew")

        # Predict Angle (Cur)
        self.PredictCurAngleFrame = customtkinter.CTkFrame(self.RightFrame, corner_radius=0)
        self.PredictCurAngleFrame.grid(row=1, column=0, padx=5, pady=(0,5), sticky="nsew")
        self.PredictCurAngleFrame.grid_columnconfigure(0, weight=1)
        self.PredictCurAngleFrame.grid_rowconfigure(0, weight=1)

        self.PredictCurAngleLabel = customtkinter.CTkLabel(self.PredictCurAngleFrame, text="Cur Angle Prediction : None", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.PredictCurAngleLabel.grid(row=0, column=0, padx=(0,0), pady=10, sticky="nsew")

        # Predict Angle (Recent)
        self.PredictPreAngleFrame = customtkinter.CTkFrame(self.RightFrame, corner_radius=0)
        self.PredictPreAngleFrame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
        self.PredictPreAngleFrame.grid_columnconfigure(0, weight=1)
        self.PredictPreAngleFrame.grid_rowconfigure(0, weight=1)

        self.PredictPreAngleLabel = customtkinter.CTkLabel(self.PredictPreAngleFrame, text="Pre Angle Prediction : None", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.PredictPreAngleLabel.grid(row=0, column=0, padx=(0,0), pady=10, sticky="nsew")

        # Predict RSRP Label
        self.PredictRSRPLabel = customtkinter.CTkLabel(self.RightFrame, text="RSRP Prediction : ", font=customtkinter.CTkFont(size=10, weight="bold"))
        self.PredictRSRPLabel.grid(row=3, column=0, padx=(0,0), pady=(2,0), sticky="ew")

        # Predict RSRP (Cur)
        self.PredictCurRSRPFrame = customtkinter.CTkFrame(self.RightFrame, corner_radius=0)
        self.PredictCurRSRPFrame.grid(row=4, column=0, padx=5, pady=(0,5), sticky="nsew")
        self.PredictCurRSRPFrame.grid_columnconfigure(0, weight=1)
        self.PredictCurRSRPFrame.grid_rowconfigure(0, weight=1)

        self.PredictCurRSRPLabel = customtkinter.CTkLabel(self.PredictCurRSRPFrame, text="Cur RSRP Prediction : None", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.PredictCurRSRPLabel.grid(row=0, column=0, padx=(0,0), pady=10, sticky="nsew")

        # Predict RSRP (Recent)
        self.PredictPreRSRPFrame = customtkinter.CTkFrame(self.RightFrame, corner_radius=0)
        self.PredictPreRSRPFrame.grid(row=5, column=0, padx=5, pady=5, sticky="nsew")
        self.PredictPreRSRPFrame.grid_columnconfigure(0, weight=1)
        self.PredictPreRSRPFrame.grid_rowconfigure(0, weight=1)

        self.PredictPreRSRPLabel = customtkinter.CTkLabel(self.PredictPreRSRPFrame, text="Pre RSRP Prediction : None", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.PredictPreRSRPLabel.grid(row=0, column=0, padx=(0,0), pady=10, sticky="nsew")


        # ===============  Resize Component  ===============
        self.resizable_frame = customtkinter.CTkFrame(self, corner_radius=0, width=300)
        # self.resizable_frame.grid(row=0, column=1, padx=(0, 5), pady=5, sticky="nswe")
        self.resizable_frame.grid_columnconfigure(0, weight=1)
        self.resizable_frame.grid_rowconfigure(0, weight=1)

        # Tab
        self.tabview = customtkinter.CTkTabview(self.resizable_frame) #,height=10
        self.tabview.grid(row=0, column=0, sticky="nsew")
        self.tabview.add("Charts")
        self.tabview.add("Charts(BG)")
        self.tabview.add("Angle")
        self.tabview.add("Handover")
        self.tabview.add("Predict(A)")
        self.tabview.add("Predict(R)")

        self.tabview.tab("Charts").grid_columnconfigure(0, weight=1)
        self.tabview.tab("Charts(BG)").grid_columnconfigure(0, weight=1)
        self.tabview.tab("Angle").grid_columnconfigure(0, weight=1)  # configure grid of individual tabs
        self.tabview.tab("Handover").grid_columnconfigure(0, weight=1)
        self.tabview.tab("Predict(A)").grid_columnconfigure(0, weight=1)
        self.tabview.tab("Predict(R)").grid_columnconfigure(0, weight=1)
        self.tabview.tab("Charts").grid_rowconfigure(0, weight=1)
        self.tabview.tab("Charts(BG)").grid_rowconfigure(0, weight=1)
        self.tabview.tab("Angle").grid_rowconfigure(0, weight=1)
        self.tabview.tab("Handover").grid_rowconfigure(0, weight=1)
        self.tabview.tab("Predict(A)").grid_rowconfigure(0, weight=1)
        self.tabview.tab("Predict(R)").grid_rowconfigure(0, weight=1)


        
        # Charts
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.tabview.tab("Charts"))  
        self.canvas.get_tk_widget().grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.tabview.tab("Charts"), pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        x_data = []
        y_data = [] 
        case = Cur_Case_Value
        for ele in self.CaseData['Case'][str(case)]['Angle']:
            x_data.append(int(ele))
            y_data.append(self.CaseData['Case'][str(case)]['Angle'][ele])
        RSRP_var = self.CaseData["Case"][str(case)]["Angle"]["0"]
        x_array = np.array(self.CaseData["Case"][str(case)]["RSRP"][str(RSRP_var)])
        y_array = np.ones((len(x_array))) * float(RSRP_var)
        self.ax.plot(x_data, y_data, color='#87cefa', label='Sampled Data')
        self.ax.plot(x_data, y_data, color='#ff8c00', marker='o', label='Sampled Data', linestyle='', markersize=6)
        self.ax.plot(x_array, y_array, color='#ff00ff', marker='o', label='Sampled Data', linestyle='', markersize=6)
        self.canvas.draw()

        # Charts
        self.fig_BG, self.ax_BG = plt.subplots()
        self.canvas_BG = FigureCanvasTkAgg(self.fig_BG, master=self.tabview.tab("Charts(BG)"))  
        self.canvas_BG.get_tk_widget().grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.toolbar_BG = NavigationToolbar2Tk(self.canvas_BG, self.tabview.tab("Charts(BG)"), pack_toolbar=False)
        self.toolbar_BG.update()
        self.toolbar_BG.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        x_data = []
        y_data = [] 
        case = Cur_Case_Value
        for ele in self.CaseDataBehind['Case'][str(case)]['Angle']:
            x_data.append(int(ele))
            y_data.append(self.CaseDataBehind['Case'][str(case)]['Angle'][ele])
        RSRP_var = self.CaseDataBehind["Case"][str(case)]["Angle"]["0"]
        x_array = np.array(self.CaseDataBehind["Case"][str(case)]["RSRP"][str(RSRP_var)])
        y_array = np.ones((len(x_array))) * float(RSRP_var)
        self.ax_BG.plot(x_data, y_data, color='#87cefa', label='Sampled Data')
        self.ax_BG.plot(x_data, y_data, color='#ff8c00', marker='o', label='Sampled Data', linestyle='', markersize=6)
        self.ax_BG.plot(x_array, y_array, color='#ff00ff', marker='o', label='Sampled Data', linestyle='', markersize=6)
        self.canvas_BG.draw()

        # AngleLogFrame
        self.AngleLogFrame = customtkinter.CTkTextbox(self.tabview.tab("Angle"), font=customtkinter.CTkFont(size=18, weight="bold"), width=400, height=300)   
        self.AngleLogFrame.grid(row=0, column=0, sticky="nswe")
        # HandoverLogFrame
        self.HandoverLogFrame = customtkinter.CTkTextbox(self.tabview.tab("Handover"), font=customtkinter.CTkFont(size=18, weight="bold"), width=400, height=300)   
        self.HandoverLogFrame.grid(row=0, column=0, sticky="nswe")
        # PredictAngleLogFrame
        self.PredictAngleLogFrame = customtkinter.CTkTextbox(self.tabview.tab("Predict(A)"), font=customtkinter.CTkFont(size=18, weight="bold"), width=400, height=300)   
        self.PredictAngleLogFrame.grid(row=0, column=0, sticky="nswe")
        # PredictRSRPLogFrame
        self.PredictRSRPLogFrame = customtkinter.CTkTextbox(self.tabview.tab("Predict(R)"), font=customtkinter.CTkFont(size=18, weight="bold"), width=400, height=300)   
        self.PredictRSRPLogFrame.grid(row=0, column=0, sticky="nswe")

        # ===============  Update Event  ===============
        self.Updata()

    def Log_Frame_event(self, Event):
        def create_resizable_frame(self):
            # Main(Root) Frame Layout
            self.grid_columnconfigure((0,1), weight=1)
            self.geometry(f"{1400}x{580}")

            # Visable
            self.resizable_frame.grid(row=0, column=1, padx=(0, 5), pady=5, sticky="nswe")

            # Variable Setting
            self.resizable_frame_visible = True

        def remove_resizable_frame(self):
            #Unvisable
            self.resizable_frame.grid_forget()
            
            # Main(Root) Frame Layout
            self.grid_columnconfigure(1, weight=0)
            self.geometry(f"{1000}x{580}")

            # Variable Setting
            self.resizable_frame_visible = False

        print(f"Log Bnt Clicked : {Event}")
        if not self.resizable_frame_visible:
            create_resizable_frame(self)
            self.resizable_frame_visible = True
        else:
            remove_resizable_frame(self)
            self.resizable_frame_visible = False

    def Confirm_event(self, Event):
        print(f"Confirm Input Bnt Clicked : {Event}")
        
        try:
            #檢查程式是否退出
            input_value = self.InputEntry.get()
            if(input_value == "exit"):
                self.running = False
                self.CurAngleLabel.configure(text=f"Process Exit")
                self.TargetAngleLabel.configure(text=f"Process Exit")

            # is number check
            int(self.InputEntry.get())

            # main process
            Target_Angle_Value = int(self.InputEntry.get())
        except Exception as e:
            print(f"Input Error : Is Not Number ({e})")
        
        self.TargetAngleLabel.configure(text=f"Target Angle : {Target_Angle_Value}")

    def Updata(self):       # timer ?
        global running, Cur_Case_Value, Cur_Angle_Value, Target_Angle_Value, Cur_RSRP_Value
        self.Cur_Time = datetime.datetime.now().strftime("%S")

        def Basic_GUI_Setting():
            self.TargetAngleLabel.configure(text=f"Current Angle : {Target_Angle_Value}")
            self.CurAngleLabel.configure(text=f"Current Angle : {Cur_Angle_Value}")
            self.PredictCurAngleLabel.configure(text=f"Cur Angle Prediction : {Angle_Kalman_PredictResult}")
            self.PredictPreAngleLabel.configure(text=f"Pre Angle Prediction : {Angle_Kalman_PrePredictResult}")
            self.RSRPLabel.configure(text=f"Current RSRP : {Cur_RSRP_Value}")
            self.PredictCurRSRPLabel.configure(text=f"Cur RSRP Prediction : {RSRP_Kalman_PredictResult:<10}")
            self.PredictPreRSRPLabel.configure(text=f"Pre RSRP Prediction : {RSRP_Kalman_PrePredictResult:<10}")
            self.CurCaseLabel.configure(text=f"Current Case : {Cur_Case_Value}")

        def Angle_TextBox():    
            if(Cur_Angle_Value == Target_Angle_Value and self.UpdataState == 0):
                self.AngleLogFrame.insert('end', f"{self.Cur_Time},    (Arrival) Current Angle : {Cur_Angle_Value}\n")
                self.UpdataState = 1    # At the moment of reaching the target Angle (on)
            else:    # Whenever the Angle value is updated
                self.AngleLogFrame.insert('end', f"{self.Cur_Time},    (Update) Current Angle : {Cur_Angle_Value}\n")
                if(Cur_Angle_Value != Target_Angle_Value): self.UpdataState = 0    
            self.AngleLogFrame.see('end')      # 頁面滑到底 (滑鼠滾輪滑到底)

        def Handover_TextBox():    
            # Handover TextBox
            self.HandoverLogFrame.insert('end', f"{self.Cur_Time}\n")
            if(Cur_RSRP_Value < Const.HANDOVER_THRESHOLD):
                PotentialList = self.CaseData['Case'][str(Cur_Case_Value)]['RSRP'][str(Cur_RSRP_Value)]
                SortPotentialList = sorted(PotentialList, key=lambda x: abs(x - Cur_Case_Value))                   # 收到 RSRP 去找出可能的 CASE

                for ele in SortPotentialList:                  # 如何判別角度是否正確
                    if(self.CaseDataBehind['Case'][str(ele)]['Angle'][str(Cur_Angle_Value)] > Const.HANDOVER_THRESHOLD):           # RIS Case 調整後，對於當前 UE 所在角度所回傳的 RSRP 有沒有變好
                        if(ele == Cur_Angle_Value):
                            self.HandoverLogFrame.insert('end', f"需切換到 {ele} ，且是正確的角度\n")
                            # Cur_Case_Value = ele                   # 切換 Case
                            break
                        else: self.HandoverLogFrame.insert('end', f"需切換到 {ele} ，但並非是 UE 所在的角度\n")

                #實際上在切換角度時，UE 也在移動，也就是第一次切換時

            else: self.HandoverLogFrame.insert('end', f"現在訊號足夠好不用切換 : {Cur_Case_Value}\n")
            self.HandoverLogFrame.insert('end', f"=====================================\n")
            # self.HandoverLogFrame.see('end')      # 頁面滑到底 (滑鼠滾輪滑到底)

        def Predict_TextBox():   
            # Predict TextBox(Angle)
            self.PredictAngleLogFrame.insert('end', f"{self.Cur_Time}\n")
            self.PredictAngleLogFrame.insert('end', f"Cur Angle : {Cur_Angle_Value}\n")
            self.PredictAngleLogFrame.insert('end', f"Cur Angle Prediction : {Angle_Kalman_PredictResult}\n")
            self.PredictAngleLogFrame.insert('end', f"Pre Angle Prediction : {Angle_Kalman_PrePredictResult}\n")
            self.PredictAngleLogFrame.insert('end', f"=====================================\n")
            # self.PredictAngleLogFrame.see('end')      # 頁面滑到底 (滑鼠滾輪滑到底)

            # Predict TextBox(RSRP)
            self.PredictRSRPLogFrame.insert('end', f"{self.Cur_Time}\n")
            self.PredictRSRPLogFrame.insert('end', f"Current RSRP : {Cur_RSRP_Value}\n")
            self.PredictRSRPLogFrame.insert('end', f"Cur RSRP Prediction : {RSRP_Kalman_PredictResult}\n")
            self.PredictRSRPLogFrame.insert('end', f"Pre RSRP Prediction: {RSRP_Kalman_PrePredictResult}\n")
            self.PredictRSRPLogFrame.insert('end', f"=====================================\n")
            # self.PredictRSRPLogFrame.see('end')      # 頁面滑到底 (滑鼠滾輪滑到底)
        
        def Chart():
            self.ax.clear()
            x_data = []
            y_data = [] 
            case = Cur_Case_Value
            for ele in self.CaseData['Case'][str(case)]['Angle']:
                x_data.append(int(ele))
                y_data.append(self.CaseData['Case'][str(case)]['Angle'][ele])
            RSRP_var = self.CaseData["Case"][str(case)]["Angle"][str(case)]
            x_array = np.array(self.CaseData["Case"][str(case)]["RSRP"][str(RSRP_var)])
            y_array = np.ones((len(x_array))) * float(RSRP_var)
            self.ax.plot(x_data, y_data, color='#87cefa', label='Sampled Data')
            self.ax.plot(x_data, y_data, color='#ff8c00', marker='o', label='Sampled Data', linestyle='', markersize=6)
            self.ax.plot(x_array, y_array, color='#ff00ff', marker='o', label='Sampled Data', linestyle='', markersize=6)
            self.canvas.draw()
        
        # function
        Basic_GUI_Setting()
        Angle_TextBox()
        Handover_TextBox()
        Predict_TextBox()
        Chart()

        self.after(int(Const.INTERVAL_TIME*1000), self.Updata) 

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
    global running, Cur_Case_Value, Cur_Angle_Value, Target_Angle_Value, Cur_RSRP_Value, Angle_Kalman_PredictResult, Angle_Kalman_PrePredictResult, RSRP_Kalman_PredictResult, RSRP_Kalman_PrePredictResult
    
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

                    print(f"Angle_Kalman_PredictResult : {received_data['Angle_Kalman_PredictResult']}")
                    Angle_Kalman_PredictResult = received_data['Angle_Kalman_PredictResult']

                    print(f"Angle_Kalman_PrePredictResult : {received_data['Angle_Kalman_PrePredictResult']}")
                    Angle_Kalman_PrePredictResult = received_data['Angle_Kalman_PrePredictResult']

                    print(f"RSRP_Kalman_PredictResult : {received_data['RSRP_Kalman_PredictResult']}")
                    RSRP_Kalman_PredictResult = received_data['RSRP_Kalman_PredictResult']

                    print(f"RSRP_Kalman_PrePredictResult : {received_data['RSRP_Kalman_PrePredictResult']}")
                    RSRP_Kalman_PrePredictResult = received_data['RSRP_Kalman_PrePredictResult']

                elif received_data['Sender_Name'] == "Move_Simulator":
                    print(f"Cur_Angle_Value : {received_data['Cur_Angle_Value']}")
                    Cur_Angle_Value = received_data['Cur_Angle_Value']

                    print(f"Target_Angle_Value : {received_data['Target_Angle_Value']}")
                    Target_Angle_Value = received_data['Target_Angle_Value']

                    print(f"Cur_RSRP_Value : {received_data['Cur_RSRP_Value']}")
                    Cur_RSRP_Value = received_data['Cur_RSRP_Value']

                # time.sleep(Const.SERVER_INTERVAL)
            except socket.timeout:
                print("等待訊息超時 => 休息一下")
                time.sleep(Const.B_RECEIVER_INTERVAL)

    except Exception as e:
        print(f"與客戶端 {client_name} 發生錯誤: {str(e)}")
 
    client_socket.close()
    print(f"與客戶端 {client_name} 斷開連接")

def GUI_Server():
    global server_socket

    host = '0.0.0.0'
    port = Const.GUI_PORT

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.settimeout(5)
    server_socket.listen(10)        # queue

    # 要連線的清單 [xApp, Mover]
    xApp_Checker = False
    Move_Simulator_Checker = False
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
            elif client_name == "Move_Simulator":
                Move_Simulator_Checker = True

            if xApp_Checker == True and Move_Simulator_Checker == True:
                break
        except socket.timeout:
            print("等待 Client 連線時間超過 => 休息一下")
            time.sleep(1)
            pass

def Sender():
    global running, Cur_Case_Value, Cur_Angle_Value, Target_Angle_Value, Cur_RSRP_Value, Angle_Kalman_PredictResult, Angle_Kalman_PrePredictResult, RSRP_Kalman_PredictResult, RSRP_Kalman_PrePredictResult
    global client_socket

    host = '127.0.0.1'
    port = Const.MOVER_PORT

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while running == True:
        try :
            client_socket.connect((host, port))
            break
        except Exception as e : 
            print(f"ERROR : Mover Server 開未開啟 => 休息一下")
        time.sleep(1)
    client_socket.settimeout(5)

    client_name = NAME
    message_length = len(client_name)
    client_socket.send(struct.pack('!I', message_length))  # 發送消息長度
    client_socket.send(client_name.encode('utf-8'))  # 發送用戶名給服務器

    while running == True:
        # =============================================
        #               Move_Simulator
        # =============================================
        message = {}
        message["Sender_Name"] = NAME
        message["Receiver_Name"] = []
        message["Receiver_Name"].append("Move_Simulator")
        message["Msg"] = "GUI Info"
        message['Target_Angle_Value'] = Target_Angle_Value
        # send_message(client_socket, message)

        time.sleep(Const.B_SENDER_INTERVAL)

def MOVER_Client():
    send_thread = threading.Thread(target=Sender, args=())
    send_thread.start()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    MOVER_Client()
    GUI_Server()

    print("==================================================")
    print("             GUI Start")
    print("==================================================")

    # GUI
    GUI_App = GUI()
    GUI_App.mainloop()
    GUI_App.destroy

    print("程式結束")
    running = False



    