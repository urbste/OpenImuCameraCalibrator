import os
import json
from argparse import ArgumentParser
import numpy as np
from sew import knot_spacing_and_variance
import matplotlib.pyplot as plt 

parser = ArgumentParser()
# Cast the input to string, int or float type 
parser.add_argument('--path_to_json', 
                    default='/media/steffen/0F78151A1CEDE4A2/Sparsenet/SparsnetTests2020/GoPro6Calib1080NoStable3_30/imu_calib/bias_calib/GH016385.json', 
                    help="path to metadata json")
parser.add_argument("--gravity_const", help="gravity constant", default=9.81, type=float)
parser.add_argument("--remove_first_s", help="how many seconds to remove from start and end (due to press of button)", default=2.0, type=float)

args = parser.parse_args()

ms_to_sec = 1.0/1000.

accl = []
gyro  = []
timestamps = []
with open(args.path_to_json, 'r') as json_file:
    json_data = json.load(json_file)
    for a in json_data['1']['streams']['ACCL']['samples']:
        timestamps.append(a['cts']*ms_to_sec)
        accl.append([a['value'][1], a['value'][2], a['value'][0]])
    for g in json_data['1']['streams']['GYRO']['samples']:
        gyro.append([g['value'][1], g['value'][2], g['value'][0]])    

ms = timestamps[1] - timestamps[0]
nr_remove = round(args.remove_first_s / ms)

accl = accl[nr_remove:len(timestamps) - nr_remove]
gyro = gyro[nr_remove:len(timestamps) - nr_remove]
timestamps = timestamps[nr_remove:len(timestamps) - nr_remove]

accl_np = np.asarray(accl) - np.array([0.0,0.0,args.gravity_const])
gyro_np = np.asarray(gyro) 

bias_gyro = np.mean(gyro_np,0)
bias_accel = np.mean(accl_np,0)

print("gyro bias: {},{},{}".format(bias_gyro[0],bias_gyro[1],bias_gyro[2]))
print("accel bias: {},{},{}".format(bias_accel[0],bias_accel[1],bias_accel[2]))

biases = {}
biases["gyro_bias"] = {"x":bias_gyro[0], "y": bias_gyro[1], "z":bias_gyro[2] }

biases["accl_bias"] = {"x":bias_accel[0], "y": bias_accel[1], "z":bias_accel[2] }

path = os.path.dirname(os.path.abspath(args.path_to_json))
output_path = os.path.join(path,'imu_bias.json')
print("Writing result to: ", output_path)
with open(output_path, 'w') as json_dump:
    json.dump(biases, json_dump)

# plt.plot(gyro_np[:,0])
# plt.plot(gyro_np[:,1])
# plt.plot(gyro_np[:,2])
# plt.show()

plt.plot(accl_np[:,0],'r',label='x')
plt.plot(accl_np[:,1],'g',label='y')
plt.plot(accl_np[:,2],'b',label='z')

plt.show()