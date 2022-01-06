#!/usr/bin/env python
'''enable automatic addition of UDP clients connected to the local network, just like --out on the cnd line'''
''' 
    Automatically adds all local network clients as outputs
'''
import multiprocessing
import time
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util

class AutoNetworkOutput(mp_module.MPModule):
    def __init__(self, mpstate):
        super(AutoNetworkOutput, self).__init__(mpstate, "auto_network_output", "automatically adds all clients on the network as outputs")
        self.Current_IPs = ['246']
        self.results = multiprocessing.Queue()
        scanner = SCANNER(self.results)
        scanner.start()
        
    def idle_task(self):
        for m in self.mpstate.mav_outputs:
            m.source_system = self.settings.source_system
            m.mav.srcSystem = m.source_system
            m.mav.srcComponent = self.settings.source_component
            
        try:
            New_IPs = self.results.get(False)
            IPs_to_add = list(set(New_IPs) - set(self.Current_IPs))
            for IP in IPs_to_add:
                self.cmd_output_add('192.168.42.' + IP + ':14550')
                self.Current_IPs.append(IP)
        except:
            pass         

    def cmd_output_add(self, args):
        '''add new output'''
        device = args
        print("Adding output %s" % device)
        try:
            conn = mavutil.mavlink_connection(device, input=False, source_system=self.settings.source_system)
            conn.mav.srcComponent = self.settings.source_component
        except Exception:
            print("Failed to connect to %s" % device)
            return
        self.mpstate.mav_outputs.append(conn)
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass

def init(mpstate):
    '''initialise module'''
    return AutoNetworkOutput(mpstate)
    
def get_IPs():
    IPs = []
    try:
        f = open("/proc/net/arp")
        content = f.read()
        addresses = content.split("192.168.42.")
        for address in addresses[1:]:
            IP = address.split(" ", 1)
            IPs.append(IP[0])
    except:
        pass
    return IPs

    
class SCANNER(multiprocessing.Process):
    
    def __init__(self, result_queue):
        multiprocessing.Process.__init__(self)
        self.result_queue = result_queue

    def run(self):
        proc_name = self.name
        while True:
            current_time = time.time()
            answer = get_IPs()
            self.result_queue.put(answer)
            time.sleep(5)
        return    