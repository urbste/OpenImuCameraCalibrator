import os
import json
from argparse import ArgumentParser
import numpy as np
from sew import knot_spacing_and_variance
import matplotlib.pyplot as plt 
import glob
from utils import read_imu_data
from telemetry_converter import TelemetryImporter

ms_to_sec = 1.0/1000.

def main():
    parser = ArgumentParser()
    parser.add_argument('--input_json_path', 
                        help="path to metadata json",
                        default='/media/steffen/0F78151A1CEDE4A2/Sparsenet/ActionVISFM_Tests/visfm_test/GX016626.json')
    parser.add_argument('--output_path', 
                        help="output path",
                        default='/media/steffen/0F78151A1CEDE4A2/Sparsenet/ActionVISFM_Tests/visfm_test/GX016626_bias.json')
    parser.add_argument("--gravity_const", 
                        help="gravity constant", 
                        default=9.81, type=float)
    parser.add_argument("--remove_sec", 
                        help="How many seconds to remove from start and end of sequence", 
                        default=0.0, type=float)
    parser.add_argument("--use_gopro_importer", default=0)
    args = parser.parse_args()

    json_importer = TelemetryImporter()
    if args.use_gopro_importer:
            json_importer.read_gopro_telemetry(args.input_json_path, args.remove_sec)
    else:
        json_importer.read_generic_json(args.input_json_path, args.remove_sec)

    accl_np = np.asarray(json_importer.telemetry["accelerometer"])
    gyro_np = np.asarray(json_importer.telemetry["gyroscope"])

    # find z direction of accelerometer, search for maximum acceleration (aroung g)
    mean_accl = np.mean(accl_np,0)
    max_dir = np.where(mean_accl == np.amax(mean_accl))
    
    grav_array = np.zeros((1,3), dtype=np.float32)
    grav_array[0,max_dir] = args.gravity_const
    # remove gravity
    accl_np = accl_np - grav_array

    # negative because we model it with (imu + bias)
    bias_gyro  = -np.mean(gyro_np,0)
    bias_accel = -np.mean(accl_np,0)
    print("Estimated biases:")
    print("gyroscope bias:     {:.5f} rad/s, {:.5f} rad/s, {:.5f} rad/s".format(bias_gyro[0],bias_gyro[1],bias_gyro[2]))
    print("accelerometer bias: {:.5f} m/s2,  {:.5f} m/s2,  {:.5f} m/s2".format(bias_accel[0],bias_accel[1],bias_accel[2]))

    biases = {}
    biases["gyro_bias"] = {"x":bias_gyro[0], "y": bias_gyro[1], "z":bias_gyro[2] }
    biases["accl_bias"] = {"x":bias_accel[0], "y": bias_accel[1], "z":bias_accel[2] }

    print("Writing result to: ", args.output_path)
    with open(args.output_path, 'w') as json_dump:
        json.dump(biases, json_dump)

    # plt.plot(gyro_np[:,0],label='x')
    # plt.plot(gyro_np[:,1],label='y')
    # plt.plot(gyro_np[:,2],label='z')
    # plt.show()

    # plt.plot(accl_np[:,0],'r',label='x')
    # plt.plot(accl_np[:,1],'g',label='y')
    # plt.plot(accl_np[:,2],'b',label='z')
    # plt.show()


if __name__ == "__main__":
    main()