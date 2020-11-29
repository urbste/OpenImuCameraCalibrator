import os
import json
from argparse import ArgumentParser
import numpy as np
from sew import knot_spacing_and_variance

ms_to_sec = 1./1000.


def main():

    parser = ArgumentParser()
    # Cast the input to string, int or float type 
    parser.add_argument('--path_to_json', 
                        default='', 
                        help="path to metadata json")
    parser.add_argument("--output_path",
                        help="output path")
    parser.add_argument("--q_so3", help="quality value for rotational component, i.e. gyro signal", default=0.99, type=float)
    parser.add_argument("--q_r3", help="quality value for translational component, i.e. accelerometer signal", default=0.97, type=float)
    args = parser.parse_args()

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


    camera_fps = json_data['frames/second']
    accl_np = np.asarray(accl)
    gyro_np = np.asarray(gyro)
    timestamps_np = np.asarray(timestamps)
    accl_np = accl_np[0:len(timestamps_np)]
    gyro_np = gyro_np[0:len(timestamps_np)]
    
    r3_dt, r3_var = knot_spacing_and_variance(accl_np.T, timestamps_np, args.q_r3, min_dt=0.01, max_dt=0.3, verbose=False)
    so3_dt, so3_var = knot_spacing_and_variance(gyro_np.T, timestamps_np, args.q_so3, min_dt=0.01, max_dt=0.3, verbose=False)

    print("Knot spacing SO3:               {:.3f} seconds at quality level q_so3={}".format(so3_dt,args.q_so3))
    print("Knot spacing  R3:               {:.3f} seconds at quality level q_r3={}".format(r3_dt,args.q_r3))
    print("Gyroscope weighting factor:     {:.3f} at quality level q_so3={}".format(1./np.sqrt(so3_var),args.q_so3))
    print("Accelerometer weighting factor: {:.3f} at quality level q_r3={}".format(1./np.sqrt(r3_var),args.q_r3))

    spline_weighting = {}
    spline_weighting["so3"] = {"knot_spacing" : so3_dt, "weighting_factor" : np.sqrt(so3_var), "quality_factor" : args.q_so3}
    spline_weighting["r3"] = {"knot_spacing" : r3_dt, "weighting_factor" : np.sqrt(r3_var), "quality_factor" : args.q_r3}
    spline_weighting["fps"] = camera_fps

    print("Writing result to: ", args.output_path)
    with open(args.output_path, 'w') as json_dump:
        json.dump(spline_weighting, json_dump)

if __name__ == "__main__":
    main()