import smbus
import math
from time import sleep

class IMU:
        
    def __init__(self):
        
        self.power_mgmt_1 = 0x6b
        self.power_mgmt_2 = 0x6c
        self.addr = 0x68
        
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(self.addr, self.power_mgmt_1, 0)
        
        print("[IMU] Initialised.")
    
    # rad/s
    def get_gyro_bias(self, N=100):
        bx = 0.0
        by = 0.0
        bz = 0.0
        
        for i in range(N):
            [gx, gy, gz] = self.get_gyro()
            bx += gx
            by += gy
            bz += gz
            sleep(0.01)
            
        return [bx / float(N), by / float(N), bz / float(N)]            
        
    # rad/s
    def get_gyro(self):
        gx = self.read_word_2c(0x43) * math.pi / (180.0 * 131.0)
        gy = self.read_word_2c(0x45) * math.pi / (180.0 * 131.0)
        gz = self.read_word_2c(0x47) * math.pi / (180.0 * 131.0)
        return [gx, gy, gz]        
        
    # m/s^2
    def get_acc(self):
        ax = self.read_word_2c(0x3b) / 16384.0
        ay = self.read_word_2c(0x3d) / 16384.0
        az = self.read_word_2c(0x3f) / 16384.0
        return [ax, ay, az]
    
    # rad
    def get_acc_angles(self):
        [ax, ay, az] = self.get_acc()
        phi = math.atan2(ay, math.sqrt(ax ** 2.0 + az ** 2.0))
        theta = math.atan2(-ax, math.sqrt(ay ** 2.0 + az ** 2.0))
        return [phi, theta]
      
    def read_byte(self, reg_adr):
        return self.bus.read_byte_data(self.addr, reg_adr)
    
    def read_word(self, reg_adr):
        high = self.bus.read_byte_data(self.addr, reg_adr)
        low = self.bus.read_byte_data(self.addr, reg_adr + 1)
        val = (high << 8) + low
        return val
    
    def read_word_2c(self, reg_adr):
        val = self.read_word(reg_adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
        
