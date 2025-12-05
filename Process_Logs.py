import pandas as pd
from DataDefinition import CollisionEvent, EmergencyBrakingEvent, CollisionWarningMessage
import argparse as ap
import numpy as np

def parse_SUMO_log(log_file):
    ces = []
    ebes = []

    with open(log_file, 'r') as log:
        lines = log.readlines()
        for line in lines:
            if "Removing junction collision participants:" in line:
                av_s = line.find('AV', 0)
                av_e = line.find('\'', av_s + 1)
                av = line[av_s: av_e]
                hdv_s = line.find('vehicle \'', 0)
                hdv_e = line.find('\'', hdv_s + 9)
                hdv = line[hdv_s + 9: hdv_e]
                if 'AV' in hdv:
                    hdv_s = line.find('vehicle \'', hdv_e)
                    hdv_e = line.find('\'', hdv_s + 9)
                    hdv = line[hdv_s + 9 : hdv_e] 

                time_s = line.find('time=')
                time_e = line.find(', stage', time_s)
                t = line[time_s + 5: time_e]
                id = 'CP_' + av + '_and_' + hdv
                ce = CollisionEvent(id, av, hdv, float(t))
                ces.append(ce)

            if "performs emergency braking" in line:
                vid_s = line.find('Vehicle \'', 0)
                vid_e = line.find('\'', vid_s + 9)
                vid = line[vid_s + 9 : vid_e]
                time_s = line.find('time=')
                t = line[time_s + 5 : -2]
                ebe = EmergencyBrakingEvent(vid, float(t))
                ebes.append(ebe)

    return ces, ebes
                
def load_CollisionMessages(log_file):
    cwms= []

    df = pd.read_csv(log_file)
    ids = df['id'].unique()
    
    #process each collision warning 
    for id in ids:
        rows = df[df['id'] == id]
        ttcs = rows['ttc'].values.tolist()
        cpxs = rows['cp_x'].values.tolist()
        cpys = rows['cp_y'].values.tolist()
        ts = rows['time'].values.tolist()
        cwm = CollisionWarningMessage(id=id, ts=ts, ttcs=ttcs, cpxs=cpxs, cpys=cpys)
        cwms.append(cwm)
    
    return cwms

'''check whether a collision warning is in the CollisionEvents'''
def is_in_CollisionEvents(collisioin_id, CollisionEvents):
    for idx, ce in enumerate(CollisionEvents):
        if collisioin_id == ce.id:
            return True, idx
    return False, -1

'''check whether an actual collision is predicted'''
def is_in_CollisionWarningMessages(collision_id, CollisionWarningMessages):
    for cwm in CollisionWarningMessages:
        if collision_id == cwm.id:
            return True
    return False

def is_in_CollisionWarningMessages_hold(collision_id, CollisionWarningMessages, holds):
    for cwm in CollisionWarningMessages:
        if collision_id == cwm.id:
            if len(cwm.TTCs) >= holds:
                return True
            else:
                return False
    return False

'''get the involved vehicle id from a collision warning id'''
def get_vehicle_id(collisionwarning_id):
    avid_s = collisionwarning_id.find('AV')
    avid_e = collisionwarning_id.find('_', avid_s)
    avid = collisionwarning_id[avid_s : avid_e]
    hdvid_s = collisionwarning_id.find('and_')
    hdvid = collisionwarning_id[hdvid_s + 4 :]
    return avid, hdvid

def is_in_EmergencyBrakingEvents(hdv_id, EmergencyBrakingEvents):
    for ebe in EmergencyBrakingEvents:
        if hdv_id == ebe.vid:
            return True
    return False

'''this function evaluates the TTC estimation'''
def TTC_evaluate(cwm, ce):
    crashTime = ce.time
    realTTC = -(np.array(cwm.Ts) - crashTime)
    predTTC = np.array(cwm.TTCs)
    max_ttc = np.max(predTTC)
    cnt_cwm = len(cwm.TTCs)# count of collision warning messages
    err_ttc = np.mean(np.abs(realTTC - predTTC))
    return max_ttc, cnt_cwm, err_ttc

def calculate_confusion_matrix(collision_events, collision_waring_messages):
    TP = 0 # True Positive
    FP = 0 # False Positive
    FN = 0 # False Negative
    TN = 0 # True Negative -> not considered in current version

    maxTTC_list = []
    numCWMsg_list = []

    if len(collision_waring_messages) == 0:
        return 0, 0, len(collision_events), 0, 0
    TP1 = 0
    for prediction in collision_waring_messages:
        # number of collosion warning message for each prediction
        numCWMsg = len(prediction.TTCs)
        numCWMsg_list.append(numCWMsg)
        maxTTC = max(prediction.TTCs)
        maxTTC_list.append(maxTTC)

        crash, idx = is_in_CollisionEvents(prediction.id, collision_events)
        if crash:
            TP += 1
        else:
            FP += 1
    for actual in collision_events:
        if is_in_CollisionWarningMessages(actual.id, collision_waring_messages):
            TP1 += 1
        else:
            FN += 1
    print("TP: %i,  FP: %i, FN: %i, TN: %i"%(TP, FP, FN, TN))
    precision = (TP)/(TP + FP)
    print("Precision: %4.2f"%precision)
    recall = TP / (TP + FN)
    print("Recall: %4.2f"%recall)

    maxTTC_mean = np.mean(np.array(maxTTC_list))
    maxTTC_std = np.std(np.array(maxTTC_list))

    numCWMsg_mean = np.mean(np.array(numCWMsg_list))
    numCWMsg_std = np.std(np.array(numCWMsg_list))

    #print("maxTTC_m: %4.2f, cntTTC_m: %4.2f, errTTC_m: %4.2f, numCWMsg_m: %4.2f, numCWMsg_s: %4.2f"%(maxTTC_mean, cntCWM_mean, errTTC_mean, numCWMsg_mean, numCWMsg_std))
    print("maxTTC_m: %4.2f, maxTTC_s: %4.2f, numCWMsg_m: %4.2f, numCWMsg_s: %4.2f"%(maxTTC_mean, maxTTC_std, numCWMsg_mean, numCWMsg_std))
    return TP, FP, FN, precision, recall, maxTTC_mean, maxTTC_std, numCWMsg_mean, numCWMsg_std

def calculate_confusion_matrix_new(collision_events, emergencybraking_events, collision_waring_messages):
    TP = 0.0 # True Positive
    FP = 0.0 # False Positive
    FN = 0.0 # False Negative
    TN = 0.0 # True Negative -> not considered in current version
    if len(collision_waring_messages) == 0:
        return 0, 0, len(collision_events), 0, 0
    TP1 = 0
    for prediction in collision_waring_messages:
        crash, idx = is_in_CollisionEvents(prediction.id, collision_events)
        if crash:
            TP += 1.0 
        else:
            avid, hdvid = get_vehicle_id(prediction.id)
            if is_in_EmergencyBrakingEvents(hdvid, emergencybraking_events):
                #print(f"CW times: {prediction.Ts}")
                TP += 0.5
                FP += 0.5
            else:
                FP += 1.0
    for actual in collision_events:
        if is_in_CollisionWarningMessages(actual.id, collision_waring_messages):
            TP1 += 1.0
        else:
            FN += 1.0
    print("TP_new: %4.2f,  FP_new: %4.2f, FN_new: %4.2f, TN_new: %4.2f"%(TP, FP, FN, TN))
    precision = (TP)/(TP + FP)
    print("Precision_new: %4.2f"%precision)
    recall = TP / (TP + FN)
    print("Recall_new: %4.2f"%recall)
    return TP, FP, FN, precision, recall

def calculate_confusion_matrix_new_hold(collision_events, emergencybraking_events, collision_waring_messages, hold):
    TP = 0.0 # True Positive
    FP = 0.0 # False Positive
    FN = 0.0 # False Negative
    TN = 0.0 # True Negative -> not considered in current version
    if len(collision_waring_messages) == 0:
        return 0, 0, len(collision_events), 0, 0
    TP1 = 0
    for prediction in collision_waring_messages:
        if len(prediction.TTCs) < hold:
            continue # the message number less than hold number, no alerting
        crash, idx = is_in_CollisionEvents(prediction.id, collision_events)
        if crash:
            TP += 1.0 
        else:
            avid, hdvid = get_vehicle_id(prediction.id)
            if is_in_EmergencyBrakingEvents(hdvid, emergencybraking_events):
                TP += 0.5
                FP += 0.5
            else:
                FP += 1.0
    for actual in collision_events:
        if is_in_CollisionWarningMessages_hold(actual.id, collision_waring_messages, hold):
            TP1 += 1.0
        else:
            FN += 1.0
    print("TP_new: %4.2f,  FP_new: %4.2f, FN_new: %4.2f, TN_new: %4.2f"%(TP, FP, FN, TN))
    precision = (TP)/(TP + FP)
    print("Precision_new: %4.2f"%precision)
    recall = TP / (TP + FN)
    print("Recall_new: %4.2f"%recall)
    return TP, FP, FN, precision, recall

def run():
    parser = ap.ArgumentParser()
    parser.add_argument('-s', '--sumo_log', type=str, help='SUMO log file path')
    parser.add_argument('-c', '--cwm_log', type=str, help='Collision Warning Message log file path')
    parser.add_argument('-o', '--output', type=str, default='output/logs/all.csv', help='record all the statistics in this file')

    args = parser.parse_args()

    file_sta = open(args.output, 'a')

    ces, ebes = parse_SUMO_log(args.sumo_log)
    cwms = load_CollisionMessages(args.cwm_log)
    tp, fp, fn, precision, recall, maxTTC_mean, maxTTC_std, numCWMsg_mean, numCWMsg_std = calculate_confusion_matrix(ces, cwms)
    tp_new, fp_new, fn_new, precision_new, recall_new = calculate_confusion_matrix_new(ces, ebes, cwms)

     #row, recording results:
    # SUMOLog, CWMLog, totalCollisionEvents, totalEmergencyBrakingEvents, totalDetectedCollision, TP, FP, FN, Precision, Recall
    # TP_new, FP_new, FN_new, Precision_new, Recall_new, MaxTTC_m, CntCWM_m, ErrTTC_m, numCWMsg_m, numCWMsg_s
    row = args.sumo_log + ',' + args.cwm_log + ',' + \
          str(len(ces)) + ',' + str(len(ebes)) + ',' + str(len(cwms)) + ',' + str(tp) + ',' + str(fp) + ',' + str(fn) + \
          ',' + str(precision) + ',' + str(recall) + ',' + str(tp_new) + ',' + str(fp_new) + ',' + str(fn_new) + \
          ',' + str(precision_new) + ',' + str(recall_new) + ',' + str(maxTTC_mean) + ',' + str(maxTTC_std) + \
          ',' + str(numCWMsg_mean) + ',' + str(numCWMsg_std) + ','

    holds = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    for hold in holds:
        tp_new_h, fp_new_h, fn_new_h, precision_new_h, recall_new_h = calculate_confusion_matrix_new_hold(ces, ebes, cwms, hold)
        new_h = str(tp_new_h) + ',' + str(fp_new_h) + ',' + str(fn_new_h) + ',' + str(precision_new_h) + ',' + str(recall_new_h) + ','
        row += new_h

    row = row[:-1]
    row += '\n'
    
    file_sta.write(row)

    file_sta.close()


if __name__ == "__main__":
    run()