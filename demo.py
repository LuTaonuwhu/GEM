import numpy as np
import matplotlib.pyplot as plt
from ErrorModel import make_demo


data1 = np.load("demo/demo_1.npy", allow_pickle=True)
data2 = np.load("demo/demo_2.npy", allow_pickle=True)
data3 = np.load("demo/demo_3.npy", allow_pickle=True)

make_demo(gt_x=data1[:,1],
         gt_y=data1[:,0],
         desired_mae=2.3,
         k=1,
         mu_yaw=0,
         sigma_yaw=30,
         delta_t=0.1,
         samples=500,
         save_path="demo/demo_1.pdf",
         left_right='right'
         )

make_demo(gt_x=data2[:,1],
         gt_y=data2[:,0],
         desired_mae=4,
         k=1,
         mu_yaw=0,
         sigma_yaw=60,
         delta_t=0.1,
         samples=500,
         save_path="demo/demo_2.pdf",
         left_right='right'
         )

make_demo(gt_x=data3[:,1],
         gt_y=data3[:,0],
         desired_mae=3.5,
         k=1,
         mu_yaw=0,
         sigma_yaw=40,
         delta_t=0.1,
         samples=500,
         save_path="demo/demo_3.pdf",
         left_right='left'
         )