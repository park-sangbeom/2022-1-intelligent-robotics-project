""" FOR ONROBOT RG2 """
from pymodbus.client.sync import ModbusTcpClient
import time 

def grasped(graspclient):
    slave = 65
    flag = graspclient.read_input_registers(268,1,unit=slave).registers[0]
    flag = (flag&0x02) == 2
    if flag:
        print("Grasp detected: True")
    return flag

def graspable(graspclient):
    slave = 65
    flag = graspclient.read_input_registers(268,1,unit=slave).registers[0]
    flag = (flag&0x08) == 8
    if flag:
        print("Grasp availablity: False")
    return flag

def reset_tool(graspclient):
    print('Tool reseting')
    toolslave = 63
    graspclient.write_register(0,2,unit=toolslave)
    time.sleep(3)
    print("Reset Fininshed", end='\r')

def close_grasp(force,width,graspclient):
    # If grasped, reset&openGrasp
    if grasped(graspclient):
        reset_tool(graspclient)
        open_grasp(400,1000,graspclient)
    # If S1 activated, reset
    if graspable(graspclient):
        reset_tool(graspclient)
    slave = 65
    graspclient.write_registers(0,[force,width,1],unit=slave)
    time.sleep(1)

def open_grasp(force,width,graspclient):
    # If S1 activated, reset
    if graspable(graspclient):
        reset_tool(graspclient)
    slave = 65
    graspclient.write_registers(0,[force,width,1],unit=slave)
    time.sleep(1)
