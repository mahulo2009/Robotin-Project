import pickle   
import numpy as np

import matplotlib.pyplot as plt

from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures


def process_duty_calibration(s):
    s = np.array(s)

    u = np.unique(s[:,1],return_index=1)
    d = u[0]
    i = u[1][1:]
    a = np.split(s[:,0], i)
   
    X = [ np.median(v) for v in a]
    X = np.array(X)
    Y = np.array(d)

    X = X[8:15]
    Y = Y[8:15]

    return X,Y

def fit_duty_calibration(X,Y):
    X_ = X.reshape((-1,1))
    Y_ =Y

    transformer = PolynomialFeatures(degree=3,include_bias=False)
    transformer.fit(X_,Y_)

    X_ = transformer.transform(X_)

    model = LinearRegression()
    model.fit(X_, Y_)

    print('gain:', model.coef_)
    print('offset:', model.intercept_)

    return model.coef_,model.intercept_

def plot_duty_calibration(X,Y,coef,intercept):
    x_min = np.min(X)
    x_max = np.max(X)

    print(x_min,x_max)

    y_min = np.min(Y)
    y_max = np.max(Y)

    print("min:",y_min)
    print("max:",y_max)

    x_fit = np.linspace(x_min, x_max, 100)
    y_fit = coef[0] * x_fit + \
            coef[1] * x_fit*x_fit + \
            coef[2] * x_fit * x_fit * x_fit + \
            intercept

    plt.plot(X,Y,x_fit,y_fit)
    plt.show()

def calibration(fname):
    s = []
    with open(fname,'rb') as fp:
        s = pickle.load(fp)

    X,Y = process_duty_calibration(s)

    coef, intercept = fit_duty_calibration(X,Y)

    plot_duty_calibration(X,Y,coef,intercept)

if __name__=="__main__":
    print("Hello Fit")

    calibration('calibration_wheel1.dat')
    calibration('calibration_wheel2.dat')
    calibration('calibration_wheel3.dat')
    calibration('calibration_wheel4.dat')