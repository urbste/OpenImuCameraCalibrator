import json
from xmlrpc.client import INVALID_XMLRPC
import numpy as np
from csv import reader


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
            image_timestamps_ns = []
            last_timestamp, last_img_timestamp = 0.0, 0.0

            for p in path_to_jsons:
                telemetry = self._read_gopro_telemetry(p, skip_seconds=0.0)
                accl.extend(telemetry["accelerometer"])
                gyro.extend(telemetry["gyroscope"])
                times = last_timestamp + np.asarray(telemetry["timestamps_ns"])
                img_times = last_img_timestamp + np.asarray(telemetry["img_timestamps_ns"])

                last_img_timestamp = img_times[-1]
                last_timestamp = times[-1]
                print("Setting last sensor time to: ",last_timestamp)
                print("Setting last image time to: ",last_img_timestamp)

                timestamps_ns.extend(times.tolist())
                image_timestamps_ns.extend(img_times.tolist())
            if skip_seconds != 0.0:
                accl, gyro, timestamps_ns = self._remove_seconds(accl, gyro, timestamps_ns, skip_seconds)
                accl = accl[0:len(timestamps_ns)]
                gyro = gyro[0:len(timestamps_ns)]
            
            self.telemetry["accelerometer"] = accl
            self.telemetry["gyroscope"] = gyro
            self.telemetry["timestamps_ns"] = timestamps_ns
            self.telemetry["img_timestamps_ns"] = image_timestamps_ns
            self.telemetry["camera_fps"] = telemetry["camera_fps"]
        else:
            self.telemetry = self._read_gopro_telemetry(path_to_jsons, skip_seconds=skip_seconds)


    def _read_gopro_telemetry(self, path_to_json, skip_seconds=0.0):
        '''
        path_to_json : str 
            path to json file
        skip_seconds : float
            How many seconds to cut from beginning and end of stream
        '''
        with open(path_to_json, 'r') as f:
            json_data = json.load(f)

        accl, gyro, cori, gravity  = [], [], [], []
        timestamps_ns, cori_timestamps_ns, gps_timestamps_ns = [], [], []
        gps_llh, gps_prec = [], []

        stream =  json_data["1"]["streams"]

        for a in stream['ACCL']['samples']:
            timestamps_ns.append(a['cts'] * self.ms_to_sec / self.ns_to_sec)
            accl.append([a['value'][1], a['value'][2], a['value'][0]])
        for g in stream['GYRO']['samples']:
            gyro.append([g['value'][1], g['value'][2], g['value'][0]])
        # image orientation at framerate
        for c in stream['CORI']['samples']:
            # order w,x,z,y https://github.com/gopro/gpmf-parser/issues/100#issuecomment-656154136
            w, x, z, y = c['value'][0], c['value'][1], c['value'][2], c['value'][3]
            cori.append([x, y, z, w])
            cori_timestamps_ns.append(c['cts'] * self.ms_to_sec / self.ns_to_sec)
        
        # gravity vector in camera coordinates at framerate
        for g in json_data['1']['streams']['GRAV']['samples']:
            gravity.append([g['value'][0], g['value'][1], g['value'][2]])
        
        # GPS is optional
        if "GPS5" in stream:
            for g in stream["GPS5"]["samples"]:
                gps_timestamps_ns.append(g['cts'] * self.ms_to_sec / self.ns_to_sec)
                lat, long, alt = g["value"][0], g["value"][1], g["value"][2]
                gps_llh.append([lat,long,alt])
                gps_prec.append(g["precision"])

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
        telemetry["gravity"] = gravity 
        telemetry["camera_orientation"] = cori
        telemetry["img_timestamps_ns"] = cori_timestamps_ns

        telemetry["gps_llh"] = gps_llh
        telemetry["gps_precision"] = gps_prec
        telemetry["gps_timestamps_ns"] = gps_timestamps_ns
        return telemetry

    def read_pilotguru_telemetry(self, path_to_accl_json, path_to_gyro_json, path_to_cam_json, skip_seconds=0.0):
        with open(path_to_accl_json, 'r') as accl_json_file:
            accl_data = json.load(accl_json_file)
        with open(path_to_gyro_json, 'r') as gyro_json_file:
            gyro_data = json.load(gyro_json_file)       
        with open(path_to_cam_json, 'r') as cam_json_file:
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
        self.telemetry["img_timestamps_ns"] = []

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
        self.telemetry["img_timestamps_ns"] = []

    def read_generic_json(self, path_to_json, skip_seconds=0.0):
        with open(path_to_json, 'r') as f:
            json_data = json.load(f)

        accl = []
        gyro  = []
        timestamps_ns = []
        img_timestamps_ns = []
        for a in json_data['accelerometer']:
            accl.append([a[0], a[1], a[2]])
        for g in json_data['gyroscope']:
            gyro.append([g[0], g[1], g[2]])
        for t in json_data['timestamps_ns']:
            timestamps_ns.append(t)
        for t in json_data['img_timestamps_ns']:
            img_timestamps_ns.append(t)

        if skip_seconds != 0.0:
            accl, gyro, timestamps_ns = self._remove_seconds(accl, gyro, timestamps_ns, skip_seconds)

        accl = accl[0:len(timestamps_ns)]
        gyro = gyro[0:len(timestamps_ns)]

        self.telemetry["accelerometer"] = accl
        self.telemetry["gyroscope"] = gyro
        self.telemetry["timestamps_ns"] = timestamps_ns
        self.telemetry["camera_fps"] = json_data["camera_fps"] 
        self.telemetry["img_timestamps_ns"] = img_timestamps_ns

    def read_zed_jsonl(self, path_to_jsonl, skip_seconds=0.0):
        with open(path_to_jsonl, 'r') as f:
            json_list = [json.loads(line) for line in f]

        accl = []
        gyro  = []
        imu_timestamps_s = []
        frametimes_s = []
        for json_str in json_list:
            if "sensor" in json_str:
                if json_str["sensor"]["type"] == "gyroscope":
                    gyro.append(json_str["sensor"]["values"])
                    imu_timestamps_s.append(json_str["time"])
                elif json_str["sensor"]["type"] == "accelerometer":
                    accl.append(json_str["sensor"]["values"])
                
            elif "frames" in json_str:
                frametimes_s.append(json_str["time"])
        # now get imu samples inside camera times
        imu_timestamps_s = np.array(imu_timestamps_s)
        frametimes_s = np.array(frametimes_s)
        gyro = np.array(gyro)
        accl = np.array(accl)
        # get only IMU data in the frametime range
        imu_ids = np.equal(
            imu_timestamps_s >= frametimes_s[0], imu_timestamps_s <= frametimes_s[-1])

        gyro = gyro[imu_ids,:]
        accl = accl[imu_ids,:]
        imu_timestamps_ns = imu_timestamps_s[imu_ids] / self.ns_to_sec
        imu_timestamps_ns -= imu_timestamps_ns[0]
        
        if skip_seconds != 0.0:
            accl, gyro, imu_timestamps_ns = self._remove_seconds(accl, gyro, imu_timestamps_ns, skip_seconds)

        accl = accl[0:len(imu_timestamps_ns)]
        gyro = gyro[0:len(imu_timestamps_ns)]

        frametimes_s = np.array(frametimes_s)
        self.telemetry["accelerometer"] = accl.tolist()
        self.telemetry["gyroscope"] = gyro.tolist()
        self.telemetry["timestamps_ns"] = imu_timestamps_ns.tolist()
        self.telemetry["camera_fps"] = 1/np.mean(np.array(frametimes_s[1:] - frametimes_s[:-1]))
        self.telemetry["img_timestamps_ns"] = []

    def read_pygpmf_json(self, path_to_json, skip_seconds=0.0):
        with open(path_to_json, 'r') as f:
            json_data = json.load(f)

        accl, gyro = [], []
        gravity, cori = [], []
        gps_llh, gps_prec = [], []
        timestamps_ns = []
        img_timestamps_ns = []
        gps_timestamps_ns = []

        for a in json_data['ACCL']['data']:
            accl.append([a[1], a[2], a[0]])
        for g in json_data['GYRO']['data']:
            gyro.append([g[1], g[2], g[0]])
        for t in json_data['ACCL']['timestamps_s']:
            timestamps_ns.append(t/self.ns_to_sec)
        for t in json_data['img_timestamps_s']:
            img_timestamps_ns.append(t/self.ns_to_sec)
        # image orientation at framerate
        if "CORI" in json_data:
            for c in json_data['CORI']['data']:
                # order w,x,z,y https://github.com/gopro/gpmf-parser/issues/100#issuecomment-656154136
                w, x, z, y = c[0], c[1], c[2], c[3]
                cori.append([x, y, z, w])
        
        if "GRAV" in json_data:
            # gravity vector in camera coordinates at framerate
            for g in json_data['GRAV']['data']:
                gravity.append([g[0], g[1], g[2]])
            
        # GPS is optional
        if "GPS5" in json_data:
            for g in json_data["GPS5"]["data"]:
                lat, long, alt = g[0], g[1], g[2]
                gps_llh.append([lat,long,alt])
            for g in json_data["GPSP"]["data"]:    
                gps_prec.append(g[0])
            for t in json_data["GPS5"]["timestamps_s"]:
                gps_timestamps_ns.append(t/self.ns_to_sec)

        camera_fps = 1 /  (np.mean(np.diff(img_timestamps_ns))*self.ns_to_sec)
        if skip_seconds != 0.0:
            accl, gyro, timestamps_ns = self._remove_seconds(accl, gyro, timestamps_ns, skip_seconds)

        accl = accl[0:len(timestamps_ns)]
        gyro = gyro[0:len(timestamps_ns)]

        self.telemetry = {}
        self.telemetry["accelerometer"] = accl
        self.telemetry["gyroscope"] = gyro
        self.telemetry["timestamps_ns"] = timestamps_ns
        self.telemetry["camera_fps"] = camera_fps
        self.telemetry["gravity"] = gravity 
        self.telemetry["camera_orientation"] = cori
        self.telemetry["img_timestamps_ns"] = img_timestamps_ns

        self.telemetry["gps_llh"] = gps_llh
        self.telemetry["gps_precision"] = gps_prec
        self.telemetry["gps_timestamps_ns"] = gps_timestamps_ns

    def get_gps_pos_at_frametimes(self, img_times_ns=None):
        '''
        Interpolate a GPS coordinate for each frame.
        Probably not very accurate but ...
        '''
        import pymap3d
        # interpolate camera gps info at frametimes
        frame_gps_ecef = [
            pymap3d.geodetic2ecef(llh[0],llh[1],llh[2]) for llh in self.telemetry["gps_llh"]]
        frame_gps_ecef = np.array(frame_gps_ecef)
        gps_times = np.array(self.telemetry["gps_timestamps_ns"]) * self.ns_to_sec
        if img_times_ns is not None:
            frame_times = np.array(img_times_ns) * self.ns_to_sec
        else:
            frame_times = np.array(self.telemetry["image_timestamps_ns"]) * self.ns_to_sec
        
        # find valid interval (interpolate only where we actually have gps measurements)
        start_frame_time_idx = np.where(gps_times[0] < frame_times)[0][0]
        end_frame_time_idx = np.where(gps_times[-1] <= frame_times)[0]
        if not end_frame_time_idx:
            end_frame_time_idx = len(frame_times)

        cam_hz = 1 / self.telemetry["camera_fps"]
        if img_times_ns is not None:
            interp_frame_times = frame_times[start_frame_time_idx:end_frame_time_idx]
        else:
            interp_frame_times = np.round(
            np.arange(
                np.round(frame_times[start_frame_time_idx],2), 
                np.round(frame_times[end_frame_time_idx],2) - cam_hz, cam_hz) ,3).tolist()

        x_interp = np.interp(interp_frame_times, gps_times, frame_gps_ecef[:,0])
        y_interp = np.interp(interp_frame_times, gps_times, frame_gps_ecef[:,1])
        z_interp = np.interp(interp_frame_times, gps_times, frame_gps_ecef[:,2])
        prec_interp = np.interp(interp_frame_times, gps_times, self.telemetry["gps_precision"])
        xyz_interp = np.stack([x_interp,y_interp,z_interp],1)

        camera_gps = dict(zip((np.array(interp_frame_times)*1e9).astype(np.int), xyz_interp.tolist()))

        return camera_gps, prec_interp

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
        from utils import time_to_s_nsec
        with open(output_path, "w") as f:
            for i in range(len(self.telemetry_importer.telemetry["timestamps_ns"])):
                t = self.telemetry_importer.telemetry["timestamps_ns"][i]
                g = self.telemetry_importer.telemetry["gyroscope"][i]
                a = self.telemetry_importer.telemetry["accelerometer"][i]
                t_s, t_ns = time_to_s_nsec(t*1e-9)
                f.write(str(t_s)+format(int(t_ns), '09d')+','+str(g[0])+','+str(g[1])+','+str(g[2])+','+str(a[0])+','+str(a[1])+','+str(a[2])+'\n')
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
    
    def convert_zed_recorder_files(self, jsonl_file, output_path, skip_seconds=0.0):
        self.telemetry_importer.read_zed_jsonl(jsonl_file, skip_seconds=skip_seconds)
        self._dump_final_json(output_path)
    
    def convert_pygpmf_telemetry(self, input_json, output_path, skip_seconds=0.0):
        self.telemetry_importer.read_pygpmf_json(input_json, skip_seconds=skip_seconds)
        self._dump_final_json(output_path)