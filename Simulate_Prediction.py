import os
import sys
import FND
from shapely import LineString, Point
import DataDefinition as DD
import ErrorModel as EM
import FCDFile as FF
from random import Random
import sumolib
import traci
import traci.constants as tc

rdm_av = Random()
rdm_av.seed(10)#used to generate AV velocity

rdm_error = Random()
rdm_error.seed(20)#used to generate random error

deg2rad = 0.0174533

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))


# ---------------global variables begin
#g_allTraj = DD.AllTrajectories()
g_allTraj = None

g_parameter_threshold_d = 2.0

g_parameter_K = 1.5

g_parameter_mae = 3 # desired mu_fnd folded normal distribution 

g_parameter_m = FND.Function_M(g_parameter_K)

g_parameter_gauss_mu = g_parameter_mae / g_parameter_m

g_parameter_gauss_sigma = g_parameter_K * g_parameter_gauss_mu

#g_parameter_sigma_fnd = math.sqrt(1 + g_parameter_K**2 - g_parameter_m**2)*g_parameter_gauss_mu

g_parameter_sigma_y = 10.0
# ---------------global variables end

def is_AV_avaliable(av_id):
    idlist = traci.vehicle.getIDList()
    return av_id in idlist

def add_AV(av_id):
    #v = random.uniform(3, 9)
    v = rdm_av.uniform(3, 9)
    traci.vehicle.add(av_id, routeID="AV_route1", typeID="AV", departSpeed=str(v), departPos=150)
    traci.vehicle.setSpeedMode(av_id, 32)
    return v

def remove_AV(av_id):
    traci.vehicle.remove(av_id, tc.REMOVE_VAPORIZED)

def tra2Polygon(vt, width):
    xys = [(vp.x, vp.y) for vp in vt]
    ls = LineString(xys)
    buffer = ls.buffer(width)
    #xs, ys = buffer.exterior.coords.xy
    return list(buffer.exterior.coords)

def generate_ConflictArea(x,y):
    p = Point(x, y)
    buffer = p.buffer(g_parameter_threshold_d)#
    return list(buffer.exterior.coords)


def remove_duplicates(ids):
    seen = []
    answer = []
    for ele in ids:
        if ele not in seen:
            seen.append(ele)
            answer.append(ele)
    return tuple(answer)

# this function detect the conflict point between AV trajectory and HDV trajectory
def collision_detection(av_tra, hdv_tra):
    #print("paramter_threshold_d used in detection: %4.2f"%parameter_threshold_d)
    x = 0
    y = 0
    ttc = 0
    d = -1
    if len(av_tra) > len(hdv_tra):
        return False, (0,0), -1, -1
    for idx, p in enumerate(av_tra):
        d = p.distance(hdv_tra[idx])
        if d < g_parameter_threshold_d:
            x = (p.x + hdv_tra[idx].x)*0.5
            y = (p.y + hdv_tra[idx].y)*0.5
            ttc = idx/10
            return True, (x,y), ttc, d
    return False, (0, 0), -2, -2

def get_random_d_and_y():
    #print("parameter_sigma_y in fnction: %4.2f"%parameter_sigma_y)

    max_d_random = rdm_error.gauss(mu=g_parameter_gauss_mu, sigma=g_parameter_gauss_sigma)
    max_y_random = rdm_error.gauss(mu=0, sigma=g_parameter_sigma_y)

    if (abs(max_d_random) < 1e-3 or abs(max_y_random) < 1e-1):
        #print("------------------------------------>in")
        #print("d: %4.2f, y: %4.2f"%(max_d_random, max_y_random))
        return get_random_d_and_y()
    #print("---------->out")
    return max_d_random, max_y_random

def test_get_random_d_and_y():
    i = 0
    while(i < 1000):
        d, y = get_random_d_and_y()
        print("i: %i, d: %4.2f, y: %4.2f"%(i, d, y))
        i += 1



def skew_trajectory(vt):
    tra_id = vt[0].id + "_skewed"
    skewed_tra = DD.VehicleTrajectory(tra_id)

    #max_d_random = rdm_error.gauss(mu=max_d_mu, sigma=max_d_sigma)
    #max_y_random = EM.random_positive_or_negative(rdm_error) * rdm_error.gauss(mu=max_y_mu, sigma=max_y_sigma)
    #max_y_random = rdm_error.gauss(mu=0, sigma=max_y_sigma)

    #if (abs(max_d_random) < 1e-3 or abs(max_y_random) < 1e-1):
    #    return skewed_tra, False

    max_d_random, max_y_random = get_random_d_and_y()
    #print("max d: %4.2f, max alpha: %4.2f"%(max_d_random, max_y_random))

    dde = EM.DistanceDeviationErrorModel(max_d=max_d_random, max_t=3.0)
    yde = EM.YawDeviationErrorModel(max_alpha=max_y_random, max_t=3.0)

    ori_xs = []#p.x for p in vt
    ori_ys = []#p.y for p in vt
    ori_hs = []#p.h for p in vt
    ori_ts = []#p.t for p in vt

    for p in vt:
        ori_xs.append(p.x)
        ori_ys.append(p.y)
        ori_hs.append(p.h)
        ori_ts.append(p.t)

    xs, ys = EM.mix_errors(ori_xs, ori_ys, ori_hs, dde.getAllDD(), yde.getAllYD(), EM.random_true_or_false(rdm_error))
    for i in range(len(xs)):
        sp = DD.VehiclePoint(tra_id, ori_ts[i],xs[i], ys[i], 0, 0, '')
        skewed_tra.push_back(sp)
    return skewed_tra


def run():
    parser = sumolib.options.ArgumentParser()
    parser.add_argument("-k", "--sumoconfig", type=str, default="input/blind_int.sumocfg", help="sumo config file")
    parser.add_argument("-f", "--fcdfile", type=str, default="output/fcd.pickle", help="specify floating car data file path")
    parser.add_argument("-s", "--sleep", action='store_true', default=False, help="sleep or not?")
    parser.add_argument("-g", "--generateFCD", action='store_true', default=False, help="generating FCD mode")
    parser.add_argument("-i", "--gui", action='store_true', default=False, help="use SUMO GUI")
    parser.add_argument("-mae", "--mean_absolute_error", type=float, default=3.0, help="desired prediction MAE in simulation")
    parser.add_argument("-pk", "--parameter_k", type=float, default=1.0, help="parameter k: gauss_sigma = k*gauss_mu")
    parser.add_argument("-sy", "--sigma_yaw", type=float, default=10, help="sigma of yaw deviation error")
    parser.add_argument("-thd", "--threshold_d", type=float, default=2.0, help="threshold distance used in collision detection")
    parser.add_argument("-track", "--track_model", action='store_true', default=False, help="tracking the simulation details")

    options = parser.parse_args()
    #options.mean_absolute_error = 1.0
    #options.parameter_k = 0.7
    #options.sigma_yaw = 10.0
    #options.threshold_d = 2.5
    #options.gui = True
    #options.sleep = True

    #global variables are covered by option values
    global g_parameter_K
    g_parameter_K = options.parameter_k
    if options.track_model:    
        print("k in run:%4.2f"%g_parameter_K)

    global g_parameter_mae
    g_parameter_mae = options.mean_absolute_error
    if options.track_model: 
        print("mae in run:%4.2f"%g_parameter_mae)

    global g_parameter_m
    g_parameter_m = FND.Function_M(g_parameter_K)
    if options.track_model: 
        print("m in run:%4.2f"%g_parameter_m)

    global g_parameter_gauss_mu
    g_parameter_gauss_mu = g_parameter_mae / g_parameter_m
    if options.track_model: 
        print("gauss_mu in run:%4.2f"%g_parameter_gauss_mu)

    global g_parameter_gauss_sigma
    g_parameter_gauss_sigma = g_parameter_K * g_parameter_gauss_mu
    if options.track_model: 
        print("gauss_sigma in run:%4.2f"%g_parameter_gauss_sigma)

    global g_parameter_sigma_y
    g_parameter_sigma_y = options.sigma_yaw
    if options.track_model: 
        print("sigma_y in run:%4.2f"%g_parameter_sigma_y)

    global g_parameter_threshold_d
    g_parameter_threshold_d = options.threshold_d
    if options.track_model: 
        print("threshold_d in run:%4.2f"%g_parameter_threshold_d)

    #global g_parameter_sigma_fnd
    #g_parameter_sigma_fnd = math.sqrt(1 + g_parameter_K**2 - g_parameter_m**2)*g_parameter_gauss_mu
    #print("std in run:%4.2f"%g_parameter_threshold_d)

    # load pre-generated fcd data
    if not options.generateFCD:
        print("loading FCD file...")
        global g_allTraj
        g_allTraj = FF.load_FCD(options.fcdfile)
        print("load FCD file done!")

    if options.gui:
        sumo = sumolib.checkBinary("sumo-gui")
    else:
        sumo = sumolib.checkBinary("sumo")
    
    parameters_str = "MAE-" + ('%.2f'%g_parameter_mae).replace('.','o') + \
        "-K-" + ('%.2f'%g_parameter_K).replace('.','o') + \
        '-SY-' + ('%.2f'%g_parameter_sigma_y).replace('.', 'o') + \
        '-Dth-' + ('%.2f'%g_parameter_threshold_d).replace('.','o')

    
    sumolog = 'output/logs/SUMO/' + parameters_str + '.log'# sumo log
    cwmlog = 'output/logs/CWM/' + parameters_str + '.csv'#collision warning message log

    file_cwm = open(cwmlog, 'w')
    file_cwm.write("id,time,ttc,cp_x,cp_y,d,thd\n")

    sumo_args = ["-d","0", "-l", sumolog, "--collision.action", "remove", "--collision.mingap-factor", "0.0", 
                 "--collision.check-junctions", "--step-length", "0.1"]
    sumo_args_g = ["-d","0", "-l", "output/SUMO.log", "--collision.action", "remove", "--collision.mingap-factor", "0.0", 
                 "--collision.check-junctions", "--step-length", "0.1", "--fcd-output", "output/fcd.xml"]

    if options.generateFCD:
        traci.start([sumo, "-c", options.sumoconfig] + sumo_args_g)
        print("simulation begin...")
    else:
        traci.start([sumo, "-c", options.sumoconfig] + sumo_args)
        print("simulation begin...")
    
    #traci.close()
    #return

    step = 1
    av_v = 0
    av_number = 0
    av_id = 'AV'
    is_AV_removed = False

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        #if step == 275:
            #stop = True
        if options.track_model: 
            print("----------------------------------------------------------------> step: %i, time: %.1f"%(step, traci.simulation.getTime()))

        if (not is_AV_avaliable(av_id)):
            av_number += 1
            av_id = 'AV' + str(av_number)
            av_v = add_AV(av_id)
            is_AV_removed = False

        e1_0_ids = traci.inductionloop.getLastStepVehicleIDs("e1_0")
        if (av_id in e1_0_ids):
            remove_AV(av_id)
            is_AV_removed = True
        
        if not is_AV_removed:
            # print AV information
            x, y = traci.vehicle.getPosition(av_id)
            v = traci.vehicle.getSpeed(av_id)
            a = traci.vehicle.getAcceleration(av_id)
            h = traci.vehicle.getAngle(av_id)
            if options.track_model: 
                print("%s x: %6.2f, y: %6.2f, a: %4.2f, h: %4.2f, v:%6.2f"%(av_id, x, y, a, h, v))

            # control AV with a constant speed
            if (v != av_v):
                traci.vehicle.setSpeed(av_id, av_v)

        # show trajectories of all these vehicles if not in generateFCD mode
        if not options.generateFCD:
            all_vIDs = traci.vehicle.getIDList()
            e20_vIDs = traci.lanearea.getLastStepVehicleIDs("e2_0")
            e21_vIDs = traci.lanearea.getLastStepVehicleIDs("e2_1")
            e22_vIDs = traci.lanearea.getLastStepVehicleIDs("e2_2")
            e23_vIDs = traci.lanearea.getLastStepVehicleIDs("e2_3")

            e24_vIDs = traci.lanearea.getLastStepVehicleIDs("e2_4")
            e25_vIDs = traci.lanearea.getLastStepVehicleIDs("e2_5")

            # here av_id is added to detected vIDs, av_id may duplicate 
            detected_vIDs = e20_vIDs + e21_vIDs + e22_vIDs + e23_vIDs + e24_vIDs + e25_vIDs + (av_id,)
            outrange_vIDs = [id for id in all_vIDs if id not in detected_vIDs]
            
            if options.gui:
                for v in detected_vIDs:
                    if v == av_id:
                        continue
                    traci.vehicle.setColor(v, (255,0,0,255))
                for v in outrange_vIDs:
                    if v == av_id:
                        continue
                    traci.vehicle.setColor(v, (0,0,255,255))
            
            if options.gui:
                current_pIDs = traci.polygon.getIDList()
                d_pIDs = [id for id in current_pIDs if id not in detected_vIDs]
                for did in d_pIDs:
                    # here, (1) the vehicles are not detected, (2) collision area polygons, and
                    # (3) skewed trajectory polygons are removed.
                    traci.polygon.remove(did) 
            
            # before prediction, the duplicated av_id is removed
            detected_vIDs_1 = remove_duplicates(detected_vIDs)
            # prediction simulation 
            ts = traci.simulation.getTime()

            # get AV's trajectory before collision detection
            #av_tra = g_allTraj.getVT(av_id).slice(ts, ts + 3.0)
            av_tra = g_allTraj[av_id].slice(ts, ts + 3.0)
            # tra_skewed is the skewed trajectory of HDV; the AV's trajectory is not skewed as it is known.
            tra_skewed = None

            for vid in detected_vIDs_1:
                #vt = g_allTraj.getVT(vid).slice(ts, ts + 3.0)
                vt = g_allTraj[vid].slice(ts, ts + 3.0)
                if len(vt) > 0 and vid != av_id:
                    tra_skewed = skew_trajectory(vt)
                
                # collision detection
                if(vid != av_id):# do nothing for AV itself
                    is_conflict = False
                    cp = (0, 0)
                    ttc = -1
                    d = -1
                    #is_conflict, cp, ttc, d = collision_detection(av_tra, vt)# -> use error-free trajectory
                    if tra_skewed != None:
                        is_conflict, cp, ttc, d = collision_detection(av_tra, tra_skewed.data)# -> use skewed trajectory
                    #print("collision flag: %s, d: %4.2f, threshold: %4.2f"%(str(is_conflict), d, paramter_threshold_d))
                    
                    if (is_conflict):
                        # conflict is detected, positive prediction
                        cp_id = "CP_" + av_id + "_and_" + vid
                        Warning_msg = cp_id + "," + str(ts) + "," + str(ttc) + "," + str(cp[0]) + "," + str(cp[1]) + "," +\
                             str(d) + ","  + str(g_parameter_threshold_d)
                        #traci.warnings.warn(Warning_msg)
                        #print(Warning_msg)
                        cp_area = generate_ConflictArea(cp[0], cp[1])

                        if options.gui:
                            traci.polygon.add(cp_id, cp_area, (239,61,245,200), fill=True, layer=25)
                            if options.track_model: 
                                print("add polygon: %s" % cp_id)

                        file_cwm.write(Warning_msg + "\n")
                        #stop av; stop AV will change the state of whole simulation
                        #consequently, following simulation state changed and became unpredictable
                        #traci.vehicle.slowDown(av_id,0,1)

                    
                if options.gui:
                    if len(vt) < 2:
                        if vid in current_pIDs:
                            traci.polygon.remove(vid)
                            if options.track_model: 
                                print("remove polygon of %s as it less than 2 points."%vid)
                        continue

                    coords = tra2Polygon(vt, 0.5)
                    if tra_skewed is not None:
                        coords_skewed= tra2Polygon(tra_skewed.data, 0.25)
            
                    if vid in current_pIDs:
                        traci.polygon.remove(vid)
                        if options.track_model: 
                            print("remove polygon: %s"%vid)


                    if "AV" in vid: 
                        traci.polygon.add(vid, coords, (104,245,76), fill= True, layer=20)
                    else:
                        traci.polygon.add(vid, coords, (245,103,126), fill= True, layer=19)
                        # the skewed trajectory
                        if tra_skewed is not None:
                            traci.polygon.add(tra_skewed.id, coords_skewed, (250,200,2), fill=True, layer=21)
                            if options.track_model: 
                                print("add polygon: %s"%tra_skewed.id)
                    if options.track_model: 
                        print("add polygon: %s" %vid)

        # all tasks were done
        step += 1
        if options.sleep:
            traci.time.sleep(0.1)
    
    file_cwm.close()
    traci.close()
    print("Total AVs: %i"%av_number)

if __name__ == "__main__":
    #-----------------test-------------------
    #readFCD('output/fcd.xml')
    #test_get_random_d_and_y()

    #-----------------main process-------------------
    run()
    