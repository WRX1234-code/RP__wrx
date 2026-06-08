import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

# 定义要拟合的非线性函数
def func_power(data, k0, k1, k2, k3, k4, k5):
    I, ω = data
    return (k0 + k1 * I + k2 * ω + k3 * I * ω + k4 * I**2 + k5 * ω**2)

# 读取数据（使用原始字符串 r'' 避免转义问题）
array_data = pd.read_csv(r'E:\rp\vofa+4.5.csv').values

# 提取数据
array_power_data = array_data[:, 0]
array_I_data = array_data[:, 1]
array_ω_data = array_data[:, 2]

# 筛选条件：同时满足三个条件
# 功率绝对值 <= 500，电流绝对值 <= 18000，转速绝对值 <= 10000
mask = (np.abs(array_power_data) <= 500) & \
       (np.abs(array_I_data) <= 16384) & \
       (np.abs(array_ω_data) <= 10000)

# 应用筛选
array_power_data_filtered = array_power_data[mask]
array_I_data_filtered = array_I_data[mask]
array_ω_data_filtered = array_ω_data[mask]

print(f"原始数据点数: {len(array_power_data)}")
print(f"筛选后数据点数: {len(array_power_data_filtered)}")
print(f"筛除数据点数: {len(array_power_data) - len(array_power_data_filtered)}")

# 使用 curve_fit 进行最小二乘拟合（使用筛选后的数据）
params, covariance = curve_fit(func_power, 
                               (array_I_data_filtered, array_ω_data_filtered), 
                               array_power_data_filtered, 
                               p0=[1, 1, 1, 1, 1, 1])

# 获取拟合的参数
k0, k1, k2, k3, k4, k5 = params

# 输出拟合的参数
print('\n拟合参数:')
print('k0:', k0)
print('k1:', k1)
print('k2:', k2)
print('k3:', k3)
print('k4:', k4)
print('k5:', k5)