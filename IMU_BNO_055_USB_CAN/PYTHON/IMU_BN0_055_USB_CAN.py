import can
import time 

def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return int(value)
vx =int(input("Enter Velocity : "))
vy =int(input("Enter Velocity : "))
yaw=int(input("Enter Velocity : "))       
pt =int(input("Enter point    : "))
# vx=255
# vy=255
# yaw=255
# Pt=255
class can_node:
    def data(self,Vx,Vy,yaw,point):
        filters = [{"can_id": 0x155, "can_mask": 0x0000, "extended": False},]
        self.bus = can.interface.Bus(channel='can0', interface='socketcan',bitrate=1000000)
        self.TxData=[]
        self.TxData.append((Vx & 0xFF00) >> 8)
        self.TxData.append(Vx & 0x00FF)
        self.TxData.append((Vy & 0xFF00) >> 8)
        self.TxData.append(Vy & 0x00FF)
        self.TxData.append((yaw & 0xFF00) >> 8)
        self.TxData.append(yaw & 0x00FF)
        self.TxData.append((point & 0xFF00) >> 8)
        self.TxData.append(point & 0x00FF)
        self.V_Back=[]
    def can_callback(self):
        mssage=can.Message(arbitration_id=0x111,data=self.TxData,is_extended_id=False)
        self.bus.send(mssage,0.1)
        # V_back=[0,0,0,0]
        # for i in range(3):
        # while(True):
        while(True):
            try:
                print("Message sent on {}".format(self.bus.channel_info))
                msg=self.bus.recv(0.01) #time out
                print(msg)
                if (msg != None):
                    if msg.arbitration_id == 0x103:
                        Heading = (msg.data[0] << 8) | msg.data[1]
                        Roll    = (msg.data[2] << 8) | msg.data[3]
                        Pitch   = (msg.data[4] << 8) | msg.data[5]
                        Qua_x   = (msg.data[6] << 8) | msg.data[7]
                        heading = map(Heading,0,65535,0,360)
                        roll = map(Roll,0,65535,0,360)
                        pitch = map(Pitch,0,65535,0,360)
                        # self.V_back=[Heading,Roll,Pitch,Qua_x]
                    
                        print('Heading: %2d, Roll: %2d, Pitch: %2d' %(heading,roll,pitch))
                    elif msg.arbitration_id == 0x140:
                        self.V_back[2] = (msg.data[4] << 8) | msg.data[5]
                        self.V_back[3] = (msg.data[6] << 8) | msg.data[7]
                      
            except can.CanOperationError:
                pass
            

if __name__ == "__main__":
    Vx  = map(vx,0,255,0,65535)
    Vy  = map(vy,0,255,0,65535)
    Yaw = map(yaw,0,255,0,65535)
    Pt  = map(pt,0,255,0,65535)
    
    canode = can_node()
    canode.data(Vx,Vy,Yaw,Pt)
    vback = canode.can_callback()
    
