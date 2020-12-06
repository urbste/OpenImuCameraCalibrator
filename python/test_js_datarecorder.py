import json
import matplotlib.pyplot as plt


with open("/home/steffen/Downloads/test_js_sensor.json","r") as f:
    data = json.load(f)


accx = []
accy = []
accz = []
for a in data["accelerometer"]:
    time = a
    accx.append(data["accelerometer"][a][0])
    accy.append(data["accelerometer"][a][1])
    accz.append(data["accelerometer"][a][2])

plt.plot(accx)
plt.plot(accy)
plt.plot(accz)
plt.show()