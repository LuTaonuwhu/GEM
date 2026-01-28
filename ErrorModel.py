import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import argparse as ap
import math
import random
import FND

random.seed(10)

font_config = {"font.family": "serif",
               "font.serif": "Times New Roman",
               "mathtext.fontset": 'stix'}
plt.rcParams.update(font_config)


class DistanceDeviationErrorModel:
    '''this model describes the evolution of distance deviation error,
       which is modeled as an exponential function: y = A^x - 1'''
    def __init__(self, max_d, max_t) -> None:
        '''max_d should not be 0'''
        #print("DDE receives max_d: %4.2f"%max_d)
        if max_d > 0:
            self.MaxDistanceDeviation = max_d
        else:
            self.MaxDistanceDeviation = -max_d 
        self.MaxTime = max_t
        self.MaxD = max_d
        self.Xs = np.arange(0, max_t + 0.1, 0.1)
        self.A = (self.MaxDistanceDeviation + 1)**(1.0/self.MaxTime)
        self.Ys = self.A ** self.Xs - 1
    
    def show(self):
        fig, ax = plt.subplots()
        #label = "d = " + str(round(self.A, 2)) + "^t - 1"
        label1 = r"$d={%s}^t-1$"%(str(round(self.A, 2)))
        ax.plot(self.Xs, self.Ys, label=label1)
        mak_x = np.arange(0, self.MaxTime + 1, 1)
        mak_y = [self.getDD(x) for x in mak_x]
        ax.plot(mak_x, mak_y, 'or')
        plt.legend(fontsize=15)
        plt.title("Distance Deviation Error Model")
        plt.xlabel("time [s]")
        plt.ylabel("Distance deviation error d [m]")
        #plt.show()
    
    def getDD(self, t):
        '''get the distance deviation value at a specified time point'''
        idx = int(t/0.1)
        if idx > len(self.Xs):
            return -1
        else:
            return self.Ys[idx]
    
    def getAllDD(self):
        return self.Ys
    
    def getAllXs(self):
        return self.Xs
    
class YawDeviationErrorModel:
    '''this mode describes the evolution of yaw deviation error,
       which is modeled as a logarithmic function:  y = log_B^(x+1)'''
    def __init__(self, max_alpha, max_t) -> None:
        #print("YDE receives max_alpha: %4.2f"%max_alpha)
        '''max_alpha should not be 0'''
        if max_alpha > 0:
            self.MaxYawDeviation = max_alpha
            self.PositiveYaw = True
        else:
            self.MaxYawDeviation = -max_alpha
            self.PositiveYaw = False
        
        self.MaxTime = max_t
        self.B = (self.MaxTime + 1) ** (1.0/self.MaxYawDeviation)
        self.Xs = np.arange(0, max_t + 0.1, 0.1)
        if self.PositiveYaw:
            #self.B = (self.MaxTime + 1) ** (1.0/self.MaxYawDeviation)
            self.Ys = [math.log((x + 1.0), self.B) for x in self.Xs]
        else:
            #self.B = (self.MaxTime + 1) ** (1.0/self.MaxYawDeviation)
            self.Ys = [-math.log((x + 1.0), self.B) for x in self.Xs]
        #self.B = self.MaxYawDeviation / self.MaxTime
        #self.Ys = self.B * self.Xs

    def getYD(self, t):
        '''get the yaw deviation value at a specified time point'''
        idx = int(t/0.1)
        if idx > len(self.Xs):
            return -1
        else:
            return self.Ys[idx]
    
    def getAllYD(self):
        return self.Ys
    
    def getAllXs(self):
        return self.Xs
    
    def show(self):
        fig1, ax1 = plt.subplots()
        #label = "y = log_" + str(round(self.B, 2)) + "^(t+1)"
        if self.PositiveYaw:
            label = r"$y={\log_{%s} {(t+1)}}$"%(str(round(self.B, 2)))
        else:
            label = r"$y={-\log_{%s} {(t+1)}}$"%(str(round(self.B, 2)))
        
        ax1.plot(self.Xs, self.Ys, label=label)
        
        mak_x = np.arange(0, self.MaxTime + 1, 1)
        mak_y = [self.getYD(x) for x in mak_x]
        ax1.plot(mak_x, mak_y, 'or')

        plt.legend(fontsize=15)
        plt.title("Yaw Deviation Error Model")
        plt.xlabel("time [s]")
        plt.ylabel("Yaw deviation error y [degree]")
        #plt.show()


'''this function adds a new dimension front/rear skew '''
def mix_errors(ori_xs, ori_ys, ori_hs, dde, yde, forward):
    xs = []
    ys = []
    for x, y, h, de, ye in zip(ori_xs, ori_ys, ori_hs, dde, yde):
        if forward: # front skew
            x1 = x + de*(math.sin((h + ye)*0.0174533))
            y1 = y + de*(math.cos((h + ye)*0.0174533))
        else: # rear skew
            x1 = x + de*(math.sin((ye - h)*0.0174533))
            y1 = y - de*(math.cos((ye - h)*0.0174533))
        xs.append(x1)
        ys.append(y1)
    return xs, ys


def random_positive_or_negative(rdm):
    if rdm.random() < 0.5:
        return 1
    else:
        return -1

def random_true_or_false(rdm):
    if rdm.random() < 0.5:
        return True
    else:
        return False


def make_samples(desired_mae, k, mu_yaw, sigma_yaw, max_t, samples):
    ori_xs = np.repeat(0, 31)
    ori_hs = np.repeat(0, 31)
    ori_ys = np.linspace(0, 30, 31)

    error_matrix = []
    eh_matrix = []
    ev_matrix = []

    max_d_random_list = []

    fig2, ax2 = plt.subplots(figsize=(7,6))

    parameter_m = FND.Function_M(k)
    gaussian_mu = desired_mae / parameter_m
    
    pk = FND.Function_P(k)

    sigma_Y_theo = np.sqrt(1 + k**2 - parameter_m**2)*gaussian_mu

    print(f"Gaussian_mu: {gaussian_mu}, k: {k}, m(k): {parameter_m}, theoretical sigma_Y: {sigma_Y_theo}")
    
    count = 0
    for i in range(samples):
        max_d_random = random.gauss(mu=gaussian_mu, sigma=k*gaussian_mu)
        max_y_random = random.gauss(mu=mu_yaw, sigma=sigma_yaw)
       
        if(abs(max_d_random) < 1e-3 or abs(max_y_random) < 1e-1):
            continue

        max_d_random_list.append(abs(max_d_random))
        dde = DistanceDeviationErrorModel(max_d=max_d_random, max_t=max_t)
        yde = YawDeviationErrorModel(max_alpha=max_y_random, max_t=max_t)
        xs, ys = mix_errors(ori_xs, ori_ys, ori_hs, dde.getAllDD(), yde.getAllYD(), random_true_or_false(random))#random_true_or_false(random)

        d = np.sqrt((xs - ori_xs)**2 + (ys - ori_ys)**2)
        error_matrix.append(d)

        count += 1

        ax2.plot(xs, ys, linewidth=1)

    error_matrix = np.array(error_matrix)
    error_matrix.reshape(count, 31)

    mae = np.mean(error_matrix, axis=0)
    fde = error_matrix[:, 30]

    sigma_Y_real = np.std(fde)

    t = np.linspace(0, 3.0, 31)

    ax2.plot(ori_xs, ori_ys, '-k', linewidth=3)

    plt.title(r'$MAE_{desired}=%4.2f,k=%4.2f, \mu_{yaw}=%4.2f,\sigma_{yaw}=%4.2f, N_{samples}=%i$'%
              (desired_mae, k, mu_yaw, sigma_yaw, samples), fontsize=12)
    ax2.axis('equal')

    ax2.text(0.7,0.95, r'$desired: \mu_Y=%5.3f$'%(desired_mae), transform=ax2.transAxes)
    ax2.text(0.7,0.9, r'$desired: \sigma_Y=%5.3f$'%(sigma_Y_theo), transform=ax2.transAxes)

    ax2.text(0.7,0.85, r'$real: \mu_Y=%5.3f$'%(mae[30]), transform=ax2.transAxes)
    ax2.text(0.7,0.8, r'$real:\sigma_Y=%5.3f$'%(sigma_Y_real), transform=ax2.transAxes)

    ax2.text(0.7, 0.75, r'$P(k)=\sigma_Y/\mu_Y=%5.3f$'%pk, transform=ax2.transAxes)
    ax2.text(0.7, 0.7, r'$M(k)=\sigma/\mu=%5.3f$'%parameter_m, transform=ax2.transAxes)

    #ax2_1 = inset_axes(ax2, width="40%", height="40%", bbox_to_anchor=(0.1, 0.5, 0.4, 0.4))
    ax2_1 = ax2.inset_axes([0.05, 0.1, 0.4, 0.4])
    ax2_1.plot(t, mae, 'r-')
    ax2_1.plot([-1, 4],[desired_mae, desired_mae],'k--', lw=0.5)
    ax2_1.set_xlim(0, 3)
    ax2_1.set_yticks([0,1,2,3,4,5])

    ax2_1.set_ylim(0, 5)

    ax2_1.text(0.5*max_t, 4.5, "Real MAE", horizontalalignment='center', verticalalignment='center')
    ax2_1.patch.set_alpha(0.5)

    ax2_2 = ax2.inset_axes([0.6, 0.1, 0.38, 0.4])
    ax2_2.hist(max_d_random_list, density=True, bins=30) 
    ax2_2.set_xlim(0, 15)          
    
    #ax2.set_aspect('equal', adjustable='box') #" std " + str(sigma_Y_theo).replace('.', '-') + \
    parameters="mae " + str(desired_mae).replace('.', '-') + \
               " k " + str(k).replace('.', '-') + \
               " mu_yaw " + str(mu_yaw).replace('.', '-') + \
               " sigma_yaw " + str(sigma_yaw).replace('.', '-') + \
               " samples " + str(samples) + '.png'
    figname='output/errormodels/test/'+ parameters
    fig2.savefig(figname)

def make_demo(gt_x, gt_y, desired_mae, k, mu_yaw, sigma_yaw, delta_t, samples, save_path, left_right='left'):
    ori_xs = gt_x
    ori_ys = gt_y

    steps = len(ori_xs)
    max_t = delta_t * steps

    # calculate heading angles from trajectory
    ori_hs = np.repeat(0, steps)
    for i in range(1, len(ori_xs)):
        delta_x = ori_xs[i] - ori_xs[i-1]
        delta_y = ori_ys[i] - ori_ys[i-1]
        ori_hs[i] = math.degrees(math.atan2(delta_x, delta_y))
    ori_hs[0] = ori_hs[1]

    error_matrix = []

    max_d_random_list = []

    fig2, ax2 = plt.subplots(figsize=(5, 4))

    parameter_m = FND.Function_M(k)
    gaussian_mu = desired_mae / parameter_m
    
    pk = FND.Function_P(k)

    sigma_Y_theo = np.sqrt(1 + k**2 - parameter_m**2)*gaussian_mu

    print(f"Gaussian_mu: {gaussian_mu}, k: {k}, m(k): {parameter_m}, theoretical sigma_Y: {sigma_Y_theo}")
    
    count = 0
    for i in range(samples):
        max_d_random = random.gauss(mu=gaussian_mu, sigma=k*gaussian_mu)
        max_y_random = random.gauss(mu=mu_yaw, sigma=sigma_yaw)
       
        if(abs(max_d_random) < 1e-3 or abs(max_y_random) < 1e-1):
            continue

        max_d_random_list.append(abs(max_d_random))
        dde = DistanceDeviationErrorModel(max_d=max_d_random, max_t=max_t)
        yde = YawDeviationErrorModel(max_alpha=max_y_random, max_t=max_t)
        xs, ys = mix_errors(ori_xs, ori_ys, ori_hs, dde.getAllDD(), yde.getAllYD(), random_true_or_false(random))#random_true_or_false(random)

        d = np.sqrt((xs - ori_xs)**2 + (ys - ori_ys)**2)
        error_matrix.append(d)

        count += 1

        ax2.plot(xs, ys, linewidth=1)

    error_matrix = np.array(error_matrix)
    error_matrix.reshape(count, steps)

    mae = np.mean(error_matrix, axis=0)
    fde = error_matrix[:, steps-1]

    sigma_Y_real = np.std(fde)

    t = np.linspace(0, max_t, steps)

    ax2.plot(ori_xs, ori_ys, '-k', linewidth=3)

    plt.title(r'$\mu_D=%4.2f,k=%4.2f, \mu_{\alpha}=%4.2f,\sigma_{\alpha}=%4.2f, N=%i$'%
              (desired_mae, k, mu_yaw, sigma_yaw, samples), fontsize=12)
    ax2.axis('equal')

    if left_right == 'right':
        ax2.text(0.6,0.95, r'$desired: \mu_D=%5.3f$'%(desired_mae), transform=ax2.transAxes, fontsize=12)
        ax2.text(0.6,0.88, r'$desired: \sigma_D=%5.3f$'%(sigma_Y_theo), transform=ax2.transAxes, fontsize=12)

        ax2.text(0.6,0.81, r'$simulated: \mu_D=%5.3f$'%(mae[steps-1]), transform=ax2.transAxes, fontsize=12)
        ax2.text(0.6,0.74, r'$simulated: \sigma_D=%5.3f$'%(sigma_Y_real), transform=ax2.transAxes, fontsize=12)

        ax2.text(0.6, 0.67, r'$P(k)=\sigma_D/\mu_D=%5.3f$'%pk, transform=ax2.transAxes, fontsize=12)
        ax2.text(0.6, 0.60, r'$M(k)=\mu_D/\mu_X=%5.3f$'%parameter_m, transform=ax2.transAxes, fontsize=12)
    else:
        ax2.text(0.05,0.95, r'$desired: \mu_D=%5.3f$'%(desired_mae), transform=ax2.transAxes, fontsize=12)
        ax2.text(0.05,0.88, r'$desired: \sigma_D=%5.3f$'%(sigma_Y_theo), transform=ax2.transAxes, fontsize=12)

        ax2.text(0.05,0.81, r'$simulated: \mu_D=%5.3f$'%(mae[steps-1]), transform=ax2.transAxes, fontsize=12)
        ax2.text(0.05,0.74, r'$simulated: \sigma_D=%5.3f$'%(sigma_Y_real), transform=ax2.transAxes, fontsize=12)

        ax2.text(0.05, 0.67, r'$P(k)=\sigma_D/\mu_D=%5.3f$'%pk, transform=ax2.transAxes, fontsize=12)
        ax2.text(0.05, 0.60, r'$M(k)=\mu_D/\mu_X=%5.3f$'%parameter_m, transform=ax2.transAxes, fontsize=12)

    ax2_1 = ax2.inset_axes([0.05, 0.1, 0.4, 0.4])
    ax2_1.plot(t, mae, 'r-')
    ax2_1.plot([-1, 10],[desired_mae, desired_mae],'k--', lw=0.5)
    ax2_1.set_xlim(0, max_t)
    ax2_1.set_yticks([0,1,2,3,4,5])
    ax2_1.set_xticks(np.arange(0, max_t+0.1, 1.0))

    ax2_1.set_ylim(0, 5)

    ax2_1.text(0.5*max_t, 4.5, "Real MAE", horizontalalignment='center', verticalalignment='center')
    ax2_1.patch.set_alpha(0.5)

    ax2_2 = ax2.inset_axes([0.6, 0.1, 0.38, 0.4])
    ax2_2.hist(max_d_random_list, density=True, bins=30, alpha=0.5) # , alpha=0.5
    ax2_2.set_xlim(0, 15)
    ax2_2.set_ylim(0, 0.3)    
    ax2_2.text(7.6, 0.26, "FDE Distribution", horizontalalignment='center', verticalalignment='center') 
    ax2_2.patch.set_alpha(0.5)        

    figname=save_path
    fig2.savefig(figname)


def run():
    parser = ap.ArgumentParser()
    parser.add_argument('-d', '--distance_deviation_error_model', action='store_true', help='distance deviation model')
    parser.add_argument('-D', '--max_distance_deviation', type=float, default=3.0, help='the max distance deviation error in meters')
    parser.add_argument('-mae', '--mean_absolute_error', type=float, default=3.0, help='the desired MAE in meters')
    parser.add_argument('-t', '--max_time', type=float, default=3.0, help='the time range [0, t] in seconds')
    parser.add_argument('-y', '--yaw_deviation_error_model', action='store_true', help='yaw deviation model')
    parser.add_argument('-Y', '--max_yaw_deviation', type=float, default=30.0, help='the max yaw deviation error in degrees')
    parser.add_argument('-e', '--examples', action='store_true', help='giving some examples')
    parser.add_argument('-s', '--samples', type=int, default=1000, help='show s samples')
    parser.add_argument('-k', '--parameter_k', type=float, default=2.0, help='parameter k: sigma = k * mae')
    parser.add_argument('-my', '--mu_yaw', type=float, default=0.0, help='mu of yaw error')
    parser.add_argument('-sy', '--sigma_yaw', type=float, default=5.0, help='sigma of yaw deviation error')
    
    args = parser.parse_args()
    #args.examples = True
    
    if args.distance_deviation_error_model:
        dde = DistanceDeviationErrorModel(args.max_distance_deviation, args.max_time)
        dde.show()
    if args.yaw_deviation_error_model:
        yde = YawDeviationErrorModel(args.max_yaw_deviation, args.max_time)
        yde.show()
    if args.examples:
        make_samples(args.mean_absolute_error, args.parameter_k, args.mu_yaw, args.sigma_yaw, args.max_time, args.samples)

    plt.show()


if __name__ == "__main__":
    run()