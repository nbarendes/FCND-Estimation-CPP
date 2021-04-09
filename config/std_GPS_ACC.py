import numpy as np

def std_GPS_ACC():
  gps_x_val = np.loadtxt('/content/sample_data/GPS_X.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
  gps_x_std  = np.std(gps_x_val)
  print("GPS X Standard Deviation (MeasuredStdDev_GPSPosXY):",gps_x_std)

  acc_x_val = np.loadtxt('/content/sample_data/ACC_X.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
  acc_x_std  = np.std(acc_x_val)
  print("ACC X Standard Deviation (MeasuredStdDev_AccelXY):",acc_x_std)