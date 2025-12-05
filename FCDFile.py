import sumolib
import DataDefinition as DD
import argparse as ap
import pickle
import time



def dump_FCD_list(fcd_file, dump_file):
    '''DO NOT USE THIS FUNCTION ANYMORE'''
    timer_s = time.time()
    AllTraj = DD.AllTrajectories()

    for ts in sumolib.xml.parse(fcd_file,'timestep'):
        t = ts.time
        for vehicle in ts.getChildList():
            id = vehicle.id
            x = vehicle.x
            y = vehicle.y
            h = vehicle.angle
            c = vehicle.type
            v = vehicle.speed
            vp = DD.VehiclePoint(id, t, x, y, h, v, c)
            exs, idx = AllTraj.exists(id)
            if exs:
                AllTraj.at(idx).push_back(vp)
            else:
                vt = DD.VehicleTrajectory(id)
                vt.push_back(vp)
                AllTraj.push_back(vt)
    
    with open(dump_file, 'wb') as fp:
        pickle.dump(AllTraj, fp)
    timer_e = time.time()

    print(f"list time cost: {timer_e - timer_s} s, length: {len(AllTraj.all)}")

def dump_FCD_dict(fcd_file, dump_file):

    timer_s = time.time()
    AllTraj = dict()
    
    for ts in sumolib.xml.parse(fcd_file, 'timestep'):
        t = ts.time
        for vehicle in ts.getChildList():
            id = vehicle.id
            x = vehicle.x
            y = vehicle.y
            h = vehicle.angle
            c = vehicle.type
            v = vehicle.speed
            vp = DD.VehiclePoint(id, t, x, y, h, v, c)
            
            val = AllTraj.get(id)
            if (val != None):
                AllTraj[id].push_back(vp)
            else:
                AllTraj[id] = DD.VehicleTrajectory()
                AllTraj[id].push_back(vp)
    with open(dump_file, 'wb') as fp:
        pickle.dump(AllTraj, fp)

    timer_e = time.time()
    print(f"dict time cost: {timer_e - timer_s} s, length: {len(AllTraj)}")

def load_FCD(picklefile='output/fcd.pickle'):
    with open(picklefile, 'rb') as fp:
        allTraj = pickle.load(fp)
    return allTraj

if __name__ == '__main__':
    parser = ap.ArgumentParser()
    parser.add_argument('-f', '--fcd_file', type=str, default='output/fcd.xml', help='path of FCD file')
    parser.add_argument('-d', '--dump_file', type=str, default='output/fcd.pickle' )
    args =  parser.parse_args()
    print("load FCD file...")
    dump_FCD_dict(args.fcd_file, args.dump_file)
    print("FCD file dump to file!")