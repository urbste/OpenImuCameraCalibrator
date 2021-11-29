import json
import numpy as np
from csv import reader
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

class TelemetryImporter:
    ''' TelemetryImporter

    '''
    def __init__(self):    
        self.ms_to_sec = 1e-3
        self.us_to_sec = 1e-6
        self.ns_to_sec = 1e-9

        self.telemetry = {}

    def _remove_seconds(self, accl, gyro, timestamps_ns, skip_seconds):
        skip_ns = skip_seconds / self.ns_to_sec

        ds = timestamps_ns[1] - timestamps_ns[0]
        nr_remove = round(skip_ns / ds)

        accl = accl[nr_remove:len(timestamps_ns) - nr_remove]
        gyro = gyro[nr_remove:len(timestamps_ns) - nr_remove]

        timestamps_ns = timestamps_ns[nr_remove:len(timestamps_ns) - nr_remove]

        return accl, gyro, timestamps_ns

    def read_gopro_telemetry(self, path_to_jsons, skip_seconds=0.0):
        '''
        path_to_jsons : path to json file or list of paths for multiple files
        skip_seconds : float
            How many seconds to cut from beginning and end of stream
        '''
        
        if isinstance(path_to_jsons, (list, tuple)):
            accl = []
            gyro = []
            timestamps_ns = []
            last_timestamp = 0.0
            for p in path_to_jsons:
                telemetry = self._read_gopro_telemetry(p, skip_seconds=0.0)
                accl.extend(telemetry["accelerometer"])
                gyro.extend(telemetry["gyroscope"])
                times = last_timestamp + np.asarray(telemetry["timestamps_ns"])
                last_timestamp = times[-1]
                print("setting last timest to: ",last_timestamp)
                timestamps_ns.extend(times.tolist())
            if skip_seconds != 0.0:
                accl, gyro, timestamps_ns = self._remove_seconds(accl, gyro, timestamps_ns, skip_seconds)
                accl = accl[0:len(timestamps_ns)]
                gyro = gyro[0:len(timestamps_ns)]
            
            self.telemetry["accelerometer"] = accl
            self.telemetry["gyroscope"] = gyro
            self.telemetry["timestamps_ns"] = timestamps_ns
            self.telemetry["camera_fps"] = telemetry["camera_fps"]
        else:
            self.telemetry = self._read_gopro_telemetry(path_to_jsons, skip_seconds=skip_seconds)

    '''
    path_to_json : str 
        path to json file
    skip_seconds : float
        How many seconds to cut from beginning and end of stream
    '''
    def _read_gopro_telemetry(self, path_to_json, skip_seconds=0.0):

        json_file = open(path_to_json, 'r')
        json_data = json.load(json_file)

        accl = []
        gyro  = []
        cori = []
        gravity = []
        timestamps_ns = []
        cori_timestamps_ns = []
        for a in json_data['1']['streams']['ACCL']['samples']:
            timestamps_ns.append(a['cts'] * self.ms_to_sec / self.ns_to_sec)
            accl.append([a['value'][1], a['value'][2], a['value'][0]])
        for g in json_data['1']['streams']['GYRO']['samples']:
            gyro.append([g['value'][1], g['value'][2], g['value'][0]])
        # image orientation at framerate
        for c in json_data['1']['streams']['CORI']['samples']:
            # order w,x,z,y https://github.com/gopro/gpmf-parser/issues/100#issuecomment-656154136
            w, x, z, y = c['value'][0], c['value'][1], c['value'][2], c['value'][3]
            cori.append([x, y, z, w])
            cori_timestamps_ns.append(c['cts'] * self.ms_to_sec / self.ns_to_sec)
        
        # gravity vector in camera coordinates at framerate
        for g in json_data['1']['streams']['GRAV']['samples']:
            gravity.append([g['value'][0], g['value'][1], g['value'][2]])
        

        camera_fps = json_data['frames/second']
        if skip_seconds != 0.0:
            accl, gyro, timestamps_ns = self._remove_seconds(accl, gyro, timestamps_ns, skip_seconds)

        accl = accl[0:len(timestamps_ns)]
        gyro = gyro[0:len(timestamps_ns)]

        telemetry = {}
        telemetry["accelerometer"] = accl
        telemetry["gyroscope"] = gyro
        telemetry["timestamps_ns"] = timestamps_ns
        telemetry["camera_fps"] = camera_fps
        telemetry["gravity_vector"] = gravity 
        telemetry["camera_orientation"] = cori
        telemetry["cori_timestamps_ns"] = cori_timestamps_ns

        return telemetry

    def read_pilotguru_telemetry(self, path_to_accl_json, path_to_gyro_json, path_to_cam_json, skip_seconds=0.0):
        accl_json_file = open(path_to_accl_json, 'r')
        accl_data = json.load(accl_json_file)
        gyro_json_file = open(path_to_gyro_json, 'r')
        gyro_data = json.load(gyro_json_file)       
        cam_json_file = open(path_to_cam_json, 'r')
        cam_data = json.load(cam_json_file)

        accl = []
        gyro  = []
        timestamps_ns = []
        # our timestamps should always start at zero for the camera, so we normalize here
        cam_t0 = cam_data['frames'][0]['time_usec']

        # in addition find out which of the two sensors runs faster (if)
        accl_ps = 1./ ((accl_data['accelerations'][1]['time_usec'] - accl_data['accelerations'][0]['time_usec'])*self.us_to_sec)
        gyro_ps = 1./ ((gyro_data['rotations'][1]['time_usec'] - gyro_data['rotations'][0]['time_usec'])*self.us_to_sec)

        if accl_ps > gyro_ps:
            subsample = int(round(accl_ps / gyro_ps))
            for i in range(0,len(accl_data['accelerations']),subsample):
                timestamps_ns.append(
                    (accl_data['accelerations'][i]['time_usec'] - cam_t0)  * self.us_to_sec / self.ns_to_sec)
                accl.append(
                    [accl_data['accelerations'][i]['x'], 
                     accl_data['accelerations'][i]['y'], 
                     accl_data['accelerations'][i]['z']])
            for g in gyro_data['rotations']:
                gyro.append([g['x'], g['y'], g['z']])
        else:
            subsample = int(round(gyro_ps / accl_ps))
            for a in accl_data['accelerations']:
                accl.append([a['x'], a['y'], a['z']])
            for i in range(0,len(gyro_data['rotations']),subsample):
                timestamps_ns.append(
                    (gyro_data['rotations'][i]['time_usec'] - cam_t0)  * self.us_to_sec / self.ns_to_sec)
                gyro.append([gyro_data['rotations'][i]['x'], gyro_data['rotations'][i]['y'], gyro_data['rotations'][i]['z']])
        camera_fps = 1. / ((cam_data['frames'][1]['time_usec'] - cam_data['frames'][0]['time_usec']) *self.us_to_sec)

        if skip_seconds != 0.0:
            accl, gyro, timestamps_ns = self._remove_seconds(accl, gyro, timestamps_ns, skip_seconds)

        accl = accl[0:len(timestamps_ns)]
        gyro = gyro[0:len(timestamps_ns)]

        self.telemetry["accelerometer"] = accl
        self.telemetry["gyroscope"] = gyro
        self.telemetry["timestamps_ns"] = timestamps_ns
        self.telemetry["camera_fps"] = camera_fps

    def read_csv(self, path_to_csv, skip_seconds=0.0):
        accl = []
        gyro  = []
        timestamps_ns = []

        # open file in read mode
        with open(path_to_csv, 'r') as read_obj:
            csv_reader = reader(read_obj)
            for row in csv_reader:
                accl.append([float(row[4]),float(row[5]),float(row[6])])
                gyro.append([float(row[1]),float(row[2]),float(row[3])])
                timestamps_ns.append(float(row[0]))
        # our timestamps should always start at zero for the camera, so we normalize here

        if skip_seconds != 0.0:
            accl, gyro, timestamps_ns = self._remove_seconds(accl, gyro, timestamps_ns, skip_seconds)

        accl = accl[0:len(timestamps_ns)]
        gyro = gyro[0:len(timestamps_ns)]

        self.telemetry["accelerometer"] = accl
        self.telemetry["gyroscope"] = gyro
        self.telemetry["timestamps_ns"] = timestamps_ns
        self.telemetry["camera_fps"] = 0.0

    def read_generic_json(self, path_to_json, skip_seconds=0.0):
        json_file = open(path_to_json, 'r')
        json_data = json.load(json_file)

        accl = []
        gyro  = []
        timestamps_ns = []
        for a in json_data['accelerometer']:
            accl.append([a[0], a[1], a[2]])
        for g in json_data['gyroscope']:
            gyro.append([g[0], g[1], g[2]])
        for t in json_data['timestamps_ns']:
            timestamps_ns.append(t)
        
        if skip_seconds != 0.0:
            accl, gyro, timestamps_ns = self._remove_seconds(accl, gyro, timestamps_ns, skip_seconds)

        accl = accl[0:len(timestamps_ns)]
        gyro = gyro[0:len(timestamps_ns)]

        self.telemetry["accelerometer"] = accl
        self.telemetry["gyroscope"] = gyro
        self.telemetry["timestamps_ns"] = timestamps_ns
        self.telemetry["camera_fps"] = json_data["camera_fps"] 

    def get_camera_quaternions_at_frametimes(self):
        # interpolate camera quaternions to frametimes
        frame_rots = R.from_quat(self.telemetry["camera_orientation"]) 
        frame_times = np.array(self.telemetry["cori_timestamps_ns"]) * self.ns_to_sec
        slerp = Slerp(frame_times.tolist(), frame_rots)
        cam_hz = 1 / self.telemetry["camera_fps"]
        interp_frame_times = np.round(
            np.arange(
                np.round(frame_times[0],2), 
                np.round(frame_times[-1],2) - cam_hz, cam_hz) ,3) / self.ns_to_sec

        camera_quaternions = dict(zip( np.array(interp_frame_times), frame_rots.as_quat()))

        return camera_quaternions

class TelemetryConverter:
    ''' TelemetryConverter

    '''
    def __init__(self):
        self.output_dict = {}
        self.telemetry_importer = TelemetryImporter()

    def _dump_final_json(self, output_path):
        with open(output_path, "w") as f:
            json.dump(self.telemetry_importer.telemetry, f)
    def _dump_kalibr_csv(self, output_path):
        with open(output_path, "w") as f:
            for i in range(len(self.telemetry_importer.telemetry["timestamps_ns"])):
                t = self.telemetry_importer.telemetry["timestamps_ns"][i]
                g = self.telemetry_importer.telemetry["gyroscope"][i]
                a = self.telemetry_importer.telemetry["accelerometer"][i]
                f.write(format(int(t), '09d')+','+str(g[0])+','+str(g[1])+','+str(g[2])+','+str(a[0])+','+str(a[1])+','+str(a[2])+'\n')
            f.close()

    def convert_gopro_telemetry_file(self, input_telemetry_json, output_path, skip_seconds=0.0):
        self.telemetry_importer.read_gopro_telemetry(
            input_telemetry_json, skip_seconds=skip_seconds)
        self._dump_final_json(output_path)

    def convert_gopro_telemetry_file_to_kalibr(self, input_telemetry_json, output_path, skip_seconds=0.0):
        self.telemetry_importer.read_gopro_telemetry(
            input_telemetry_json, skip_seconds=skip_seconds)
        self._dump_kalibr_csv(output_path)

    def convert_pilotguru_telemetry_file(self, input_accl_json, input_gyro_json, input_cam_json, output_path, skip_seconds=0.0):
        self.telemetry_importer.read_pilotguru_telemetry(
            input_accl_json, input_gyro_json, input_cam_json, skip_seconds=skip_seconds)
        self._dump_final_json(output_path)

    def convert_csv_telemetry_file(self, csv_file, output_path, skip_seconds=0.0):
        self.telemetry_importer.read_csv(csv_file, skip_seconds=skip_seconds)
        self._dump_final_json(output_path)
    