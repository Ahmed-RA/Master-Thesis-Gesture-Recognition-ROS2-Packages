import numpy as np
import pandas as pd

td = np.load("test_fpfh_data.npy")
tl = np.load("test_fpfh_labels.npy")



counter = [0,0,0,0,0,0]
g1 = np.zeros((6,10,14,33))


for i ,v in enumerate(td):
    cls = np.argmax(tl[i])
    if cls == 0 and counter[0] < 10:
        g1[0,counter[0],:,:] = v
        counter[0] += 1
    elif cls == 1 and counter[1] < 10:
        g1[1,counter[1],:,:] = v
        counter[1] += 1
    elif cls == 2 and counter[2] < 10:
        g1[2,counter[2],:,:] = v
        counter[2] += 1
    elif cls == 3 and counter[3] < 10:
        g1[3,counter[3],:,:] = v
        counter[3] += 1
    elif cls == 4 and counter[4] < 10:
        g1[4,counter[4],:,:] = v
        counter[4] += 1
    elif cls == 5 and counter[5] < 10:
        g1[5,counter[5],:,:] = v
        counter[5] += 1
    elif sum(counter) == 60:
        break

np.save("test_feed_fpfh.npy",g1)

print("end")