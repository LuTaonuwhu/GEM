import math
from typing import Dict

class VehiclePoint:
    '''this is a vehicle floating point class, VP, containing attributes:
        id  x   y   angle(h)   type(c)    speed(v)
    '''
    def __init__(self, id='', t=0.0, x=0.0, y=0.0, h=0.0, v=0.0, c='') -> None:
        self.id = id
        self.t = float(t)
        self.x = float(x)
        self.y = float(y)
        self.h = float(h)
        self.v = float(v)
        self.c = c
    
    def distance(self, vp):
        d = math.sqrt((self.x - vp.x)*(self.x - vp.x) + 
                      (self.y - vp.y)*(self.y - vp.y))
        return d


class VehicleTrajectory:
    '''this is a vehicle trajectory class, consists of sequential VehicePoint, VT'''
    def __init__(self, id='') -> None:
        self.data = list()
        self.id = id
    
    def push_back(self, vp) -> None:
        self.data.append(vp)
    
    def clear(self) -> None:
        self.data.clear()
    
    def slice(self, ts_from, ts_to):
        f = ts_from - self.data[0].t
        t = ts_to - self.data[0].t
        index_f = int(f / 0.1)
        index_t = int(t / 0.1)
        if index_t > len(self.data):
            return self.data[index_f:]
        else:
            return self.data[index_f : index_t + 1]



class AllTrajectories:
    '''this class manages all trajectories in a simulation
       this class is not used in latest version (dict is used instead)'''
    def __init__(self) -> None:
        self.all = list()

    
    def getVT(self, id):
        for tra in self.all:
            if tra.id == id:
                return tra
        return None

    '''check whether a trajectory with 'id' exists'''
    def exists(self, id):
        exs = False
        idx = 0
        for tra in self.all:
            if tra.id == id:
                exs = True
                break
            idx += 1
        return exs, idx
    #def exists(self, id):
    #    idx = next((i for i, tra in enumerate(self.all) if tra.id == id), -1)
    #    if idx == -1:
    #        return False, idx
    #    else:
    #        return True, idx 
    
    def push_back(self, vt):
        self.all.append(vt)
    
    def at(self, idx):
        return self.all[idx]

class CollisionEvent:
    '''the collision events reported by SUMO'''
    def __init__(self, id='', av='', hdv='', t=0.0) -> None:
        self.id = id
        self.av = av
        self.hdv = hdv
        self.time = t

class EmergencyBrakingEvent:
    '''the emergency braking event reported by SUMO'''
    def __init__(self, vid='', t=0.0) -> None:
        self.vid = vid
        self.time = t

class CollisionWarningMessage:
    '''the collision warning message class'''
    def __init__(self, id, ts=[], ttcs=[], cpxs=[], cpys=[]) -> None:
        self.id = id
        self.Ts = ts
        self.TTCs = ttcs
        self.CP_Xs = cpxs
        self.CP_Ys = cpys
    
    def append_TTC(self, ttc) -> None:
        self.TTCs.append(ttc)
    
    def append_CPX(self, cpx) -> None:
        self.CP_Xs.append(cpx)
    
    def append_CPY(self, cpy) -> None:
        self.CP_Ys.append(cpy)
    
    def append_T(self, t) -> None:
        self.Ts.append(t)