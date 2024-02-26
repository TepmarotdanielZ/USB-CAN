from asyncio import CancelledError
from email import message
import can 
import time

def map (Input, min_input, max_input, min_output, max_output):
    value = ((input - min_input) * (max_output - min_output) / (max_input - min_input) + min_output)
    return int (value)

vx  = int (input (" ENTER VELOCITY :\n ")) 
vy  = int (input (" ENTER VELOCITY :\n ")) 
yaw = int (input (" ENTER VELOCITY :\n ")) 
pt  = int (input (" ENTER POIONT   :\n "))

class can_node:
    def data (self, vx, vy, yaw, point):
        filter = [{" can_id " : 0x103, "  can_mask ": 0x0000, " exterded " : False},]
        self.bus = can.interface.Bus(channel = ' can0 ', interface = ' socketcan ' , bitrate = 1000000)
        self.TxData.append((vx & 0xFF00)  >> 8)
        self.TxData.append(vx & 0x00FF)
        self.TxData.append((vy & 0xFF00)  >> 8)
        self.TxData.append(vy & 0x00FF)
        self.TxData.append((yaw & 0xFF00) >> 8)
        self.TxData.append(yaw & 0x00FF)
        self.TxData.append((pt & 0xFF00)  >> 8)
        self.TxData.append(pt & 0x00FF)
    def can_callback(self):
        message = can.Message(arditration_id = 0x111, data = self.TxData, is_extended_id = False)
        self.bus.send(message, 0.1)
        v_back = [0,0,0,0]

        for i in range(2):
            try:
                print(" MESSAGE SENT ON {} ".format(self.bus.channel_info))
                msg = self.bus.recv(0.1) # TIME OUT
                if (msg != None):
                    if msg.arbitration_id == 0x103:
                        v_back[0] = (msg.data[0] << 8) | msg.data[1]
                        v_back[1] = (msg.data[2] << 8) | msg.data[3]
                        v_back[2] = (msg.data[4] << 8) | msg.data[5]
                        v_back[3] = (msg.data[6] << 8) | msg.data[7]
                    elif msg.arbitration_id == 0x140:
                        v_back[2] = (msg.data[4] << 8) | msg.data[5]
                        v_back[3] = (msg.data[6] << 8) | msg.data[7]

            except can.Can0perationError:
                pass
        return v_back

if __name__ == " __main__ ":
    vx  = map (vx , 0,255, 0 , 65535)
    vy  = map (vy , 0,255, 0 , 65535)
    yaw = map (yaw, 0,255, 0 , 65535)
    vy  = map (pt , 0,255, 0 , 65535)

    canode = can_node()
    canode.data(vx, vy, yaw, pt)
    vback = canode.can_callback()

    print (vback)

