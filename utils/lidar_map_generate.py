import matplotlib.pyplot as plt
import json

# plt.figure(figsize=(200,200))

# with open("/home/amit-singh/Downloads/qudacopter/map_data.json","r") as file_data :
# 	map_data = json.load(file_data)

# plt.scatter(map_data["x"],map_data["y"],c="#fd72c0", marker=".", s=5)
# # plt.plot([-4.34, -4.23],[-3.39, -3.01],c="blue")
# plt.xlim(-20,20)
# plt.ylim(-20,20)
# plt.show()

import time
import random


# plt.ion()  # Enable interactive mode
# plt.figure(figsize=(100,100))
fig, ax = plt.subplots()
# line, = ax.plot([], [])  # Create an empty line object

# ax.set_xlim(-40, 40)
# ax.set_ylim(-40, 40)

# x_data, y_data = [], []

# try:
# 	while True:
# 		try :
# 			# with open("/home/amit-singh/Downloads/qudacopter/map_data.json","r") as file_data :
# 			with open("/home/amit-singh/Downloads/qudacopter/tmp_map_data.json","r") as file_data :
# 				map_data = json.load(file_data)

# 			plt.scatter(map_data["x"],map_data["y"],c="#fd72c0", marker=".", s=2)   

# 			plt.scatter(map_data["drone_x"],map_data["drone_y"],c="black", marker=".", s=2)       
			
# 			fig.canvas.draw()
# 			fig.canvas.flush_events()
			
# 			time.sleep(0.1)
		
# 		except json.decoder.JSONDecodeError :

# 			time.sleep(0.1)

# except KeyboardInterrupt:
#     print("Plotting stopped by user")

with open("/home/amit-singh/Downloads/qudacopter/map_data.json","r") as file_data :
	map_data = json.load(file_data)

	plt.scatter(map_data["x"],map_data["y"],c="#fd72c0", marker=".", s=2)   

	plt.scatter(map_data["drone_x"],map_data["drone_y"],c="black", marker=".", s=2)       
	plt.show()
	
# 	fig.canvas.draw()
# 	fig.canvas.flush_events()

# fig = plt.figure()
# ax = plt.axes(projection="3d")

# x_data, y_data, z_data = [], [], []

# ax.set_xlim(20, -20)
# ax.set_ylim(20, -20)
# ax.set_zlim(0, 12)

# try:
# 	while True:
# 		try :
# 			with open("/home/amit-singh/Downloads/qudacopter/map_data.json","r") as file_data :
# 				map_data = json.load(file_data)

# 			ax.scatter(map_data["x"],map_data["y"],map_data["z"],marker=".",s=2,c="#fd72c0")   

# 			ax.scatter(map_data["drone_x"],map_data["drone_y"],map_data["drone_z"],marker=".",s=2,c="black")       
			
# 			fig.canvas.draw()
# 			fig.canvas.flush_events()
			
# 			time.sleep(0.1)
		
# 		except json.decoder.JSONDecodeError :

# 			time.sleep(0.1)

# except KeyboardInterrupt:
#     print("Plotting stopped by user")