import numpy as np
from sklearn.linear_model import LinearRegression

actual_angles = np.array([])
analog_fbs = np.array([])
with open("analog_servo_log.txt", "r") as f: 
    lines = f.readlines()

for line in lines: 
    if line != "\n":
        line_ls = line.split("sent:")
        parts = line_ls[1].split(" |")
        sent = int(parts[0])
        print(line.split("original measurement: ")[1].split("|")[0])
        # measured = int(line.split("original measurement: ")[1].split("|")[0])
        # measured = int(line.split("original measurement: ")[1])
        measured = float(line.split("processed measured: ")[1].split("\n")[0])  #see how good our prediction is
        actual_angles = np.append(actual_angles, int(sent))
        analog_fbs = np.append(analog_fbs, int(measured))

analog_fbs = analog_fbs.reshape((-1,1))
model = LinearRegression()
model.fit(analog_fbs, actual_angles)
print('intercept:', model.intercept_)
print('slope:', model.coef_)
print(f'R^2: {model.score(analog_fbs, actual_angles)}')

model_angles = []
for analog_fb in analog_fbs.flatten(): 
    model_angles.append((model.coef_ * analog_fb + model.intercept_)[0])
#

import matplotlib.pyplot as plt
plt.plot(analog_fbs, actual_angles, "bo")
plt.plot(analog_fbs, model_angles, "go")
plt.show()
