import math
import matplotlib.pyplot as plt
from scipy.stats import norm
import numpy as np

def showMk():
    ks = [(i+1)*0.01 for i in range(500)]
    ms = []
    for k in ks:
        m = Function_M(k)
        ms.append(m)

    fig, ax = plt.subplots()
    ax.plot(ks, ms)
    plt.xlabel('k')
    plt.ylabel('M(k)')
    plt.grid(which='both', axis='both')
    plt.show()

def showPk():
    ks = [(i+1)*0.01 for i in range(500)]
    pks = []
    for k in ks:
        #mk = Function_M(k)
        #nk = math.sqrt(1 + k**2 - mk**2)
        #pk = nk/mk
        pk = Function_P(k)
        pks.append(pk)
    fig, ax = plt.subplots()
    ax.plot(ks, pks)
    plt.xlabel('k')
    plt.ylabel('P(k)')
    plt.grid(which='both', axis='both')
    plt.show()



def compare():
    x = np.linspace(-15, 15, 3000)
    sigma = 3
    mu = 0
    nd_y = (1.0/(np.sqrt(2*np.pi*sigma**2)))*np.exp(-(x - mu)**2/(2*sigma**2))
    s = np.sum(nd_y)
    p = nd_y / s
    #----1
    mu1 = x.dot(p)
    mom2 = np.power(x, 2).dot(p)
    var = mom2 - mu1**2
    sigma1 = np.sqrt(var) 
    #----

    #----2
    nd_mu = np.sum(x * p)
    d = (x - nd_mu)
    dd = d ** 2
    ddy = np.sum(dd * p)
    sigma2 = np.sqrt(ddy)

    fnd_y = nd_y * 2
    fnd_y[: 1500] = 0

    fnd_p = p * 2
    fnd_p[: 1500] = 0
    fnd_mu = np.sum(x * fnd_p)
    d = x - fnd_mu
    dd = d ** 2
    ddy = np.sum(dd * fnd_p)
    fnd_sigma = np.sqrt(ddy)

    fig, ax = plt.subplots()
    ax.plot(x, nd_y, 'r-', label=r'Normal Distribution: $\mu=%4.2f, \sigma=%4.2f$'%(nd_mu, sigma2))
    ax.plot(x, fnd_y, 'g-', label=r'Folded Normal Distribution: $\mu=%4.2f, \sigma=%4.2f$'%(fnd_mu, fnd_sigma))

    mu_y = Mu_Y(mu, sigma)
    print(mu_y)

    #fig2, ax2 = plt.subplots()
    #ax2.plot(x, norm.pdf(x, 0, 3))

    lg = plt.legend()
    lg.get_frame().set_alpha(0)
    plt.show()

'''Given sigma_x = k * mu_x
    this function describes the ratio of mu_x and mu_d (mu_d = m * mu_x),
    where mu_d is the mean of a Folded Normal Distribution, whose mean is mu_x 
    and sigma is sigma_x -> sigma_x = k * mu_x'''
def Function_M(k):
    m = k*math.sqrt(2/math.pi)*math.exp(-1/(2*k*k)) + math.erf(1/(math.sqrt(2)*k))
    return m

'''n(k)*mu_x = sigma_d'''
def Function_N(k):
    n = math.sqrt(1 + k**2 - Function_M(k)**2)
    return n

'''p(k)=sigma_d/mu_d'''
def Function_P(k):
    nk = Function_N(k)
    mk = Function_M(k)
    pk = nk/mk
    return pk   

'''Given mu_y (desired) and a k (determined),
    this function calculate the mu you should feed to Gauss'''
def MuY2Mu(mu_y, k):
    m = Function_M(k)
    mu = mu_y / m
    return mu

def Mu_Y(mu, sigma):
    mu_y = sigma*math.sqrt(2.0/math.pi)*math.exp(-mu**2/2*sigma**2) + \
           mu*math.erf(mu/math.sqrt(2*sigma**2))
    return mu_y


if __name__ == '__main__':
    #showPk()
    showMk()
    #compare()
    