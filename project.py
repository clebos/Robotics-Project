from __future__ import division
import numpy as np
import scipy.signal as sig
import scipy.spatial.distance as dist
import random as rand
import matplotlib.pyplot as plt
import math

# Function to represent the robots mental percepton of where the drifter is after
# 1 time step
def blur_probability(ocean, kernel):
	ocean_convolved = sig.convolve2d(ocean, kernel)
	return ocean_convolved

# Function to pick a motion for the boat given what it knows about the probable
# location of the drifter.
# Initially this is set to choose a step in the most probable direction
def choose_motion(boat, ocean):
	most_likely_x, most_likely_y = np.unravel_index(ocean.argmax(), ocean.shape)
	current_x = boat['X_POS']
	current_y = boat['Y_POS']

	# Choose a step toward the most likely x and y position and then add noise
	# In this case, we use 1 as the maximal amount that the boat can move, 
	# taken from the paper where the robot's swims at 1 m/s
	# *****IN FUTURE***** might want to change the step size (based on boat speed etc)

	# Use the idea of similar triangles to 
	dist_x = most_likely_x - current_x
	dist_y = most_likely_y - current_y
	theta = math.atan(dist_y/dist_x)
	# Because of the way angles work, if x is less than 0 we have to add pi to get the 
	# correct angle
	if dist_x < 0:
		theta = theta + math.pi

	# We know that the distance of the hypoteneuse must be 1, so using this and the angle we can
	# calculate the distance we need to move in the x and y direction 
	new_y = current_y + math.sin(theta)
	new_x = current_x + math.cos(theta)

	new_boat = {'X_POS': new_x, 'Y_POS':new_y}	
	return new_boat 

# In this distance formuatlion, we calculate distance as the Euclidian distance 
# between the two points. We may want to change this later
def distance(boat, beacon):
	boat_array = (boat['X_POS'], boat['Y_POS'])
	beacon_array = (beacon['X_POS'], beacon['Y_POS'])
	distance = dist.euclidean(boat_array, beacon_array)
	return distance

# Function to look around the boat and see if it can see the beacon
# Assume the boat can see the beacon if it is within a distance certain distance
# calculated by a distance function
# Return NONE if it finds the beacon within it's RComm, which, taken from the 
# paper is RCommm = 5, return the updated ocean otherwise   
def observe(boat, ocean, beacon):
	dist = distance(boat, beacon)
	# Found the beacon, so report back "None"
	if abs(dist) < 5:
		return None
	# If not found the beacon, we know that all squares around the boat are 0
	else:
		boat_x = int(boat['X_POS'])
		boat_y = int(boat['Y_POS'])
		for i in range(-5,5):
			for j in range (-5, 5):
				added_x = boat_x + i
				added_y = boat_y + j
				ocean[added_x, added_y] = 0
		return ocean

def drift_beacon(beacon):
	new_beacon_x = beacon['X_POS'] + rand.uniform(-0.2, 0.2)
	new_beacon_y = beacon['Y_POS'] + rand.uniform(-0.2,0.2)
	new_beacon = {'X_POS': new_beacon_x, 'Y_POS':new_beacon_y}
	return new_beacon

if __name__ == '__main__':

	times = []

	for i in range(0,1000):
		print i
		#### SET UP OF THE INTITIAL VARIABLES #### 

		# Set up the ocean as an array of floats, intially all intialized to 0
		# Initial array = 99 x 99 
		X_SIZE = 99
		Y_SIZE = 99
		ocean = np.zeros((X_SIZE, Y_SIZE), dtype=float)

		# Set up the initial blurring kernel.
		# At first, let the center value be 0.5 and all surrounding values be 0.1
		# *****IN FUTURE***** may want to change how we set the kernel to reflect 
		# 		wind and current factors
		# 		May want to add a def propagation_model(wind_direction, wind_velocity): 
		# 		Which has input parameters which modify the kernel to deal with
		#		wind and current
		kernel = np.array([[0.1, 0.1, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1]])

		# Sets the timer to count the number of iterations 
		timer = 0

		# Initially set the beacon to be in the middle of the ocean
		# *****IN FUTURE***** we may want to change this to be a random point
		#		May want to add additional beacons
		beacon = {'X_POS': (X_SIZE-1)/2 , 'Y_POS': (Y_SIZE-1)/2}
		ocean[beacon['X_POS'], beacon['Y_POS']] = 1.0

		# Define the boat as a dictionary with key-value pairs corresponding to position:
		# *****IN FUTURE***** we may want to add velocity to the boat's defintion 
		boat = {'X_POS': 0.0, 'Y_POS':0.0} 

		## Set up some arrays for plotting later ## 
		boat_locations = [(boat['X_POS'], boat['Y_POS'])]
		beacon_locations = [(beacon['X_POS'], beacon['Y_POS'])]
		Rcomms = [plt.Circle((boat['X_POS'], boat['Y_POS']),5,color='grey',fill=False)]

		#print "done setting up ...... "

		# Increment the time step by 1 

		while True:
			timer = timer + 1
			#print "timer is : ", timer
			# drift the beacon a little bit 
			new_beacon = drift_beacon(beacon)
			#print 'new beacon: ', new_beacon
			# blur the probability distribution
			new_ocean = blur_probability(ocean, kernel)
			#print 'convolved ocean is: ', new_ocean
			# choose a direction for the boat to travel in
			new_boat = choose_motion(boat,ocean)
			#print 'new boat: ', new_boat
			# see whether or not you've found the beacon
			ocean_after_exploration = observe(new_boat, new_ocean, new_beacon)
			
			### UPDATE ALL THE VARIABLES ### 
			beacon = new_beacon
			beacon_location = (beacon['X_POS'], beacon['Y_POS'])
			beacon_locations.append(beacon_location)
			ocean = ocean_after_exploration	
			boat = new_boat
			boat_location = (boat['X_POS'], boat['Y_POS'])
			boat_locations.append(boat_location)
			
			if (ocean_after_exploration == None):
				Rcomm_circle = plt.Circle((boat['X_POS'], boat['Y_POS']),5,color='red',fill=False)
				Rcomms.append(Rcomm_circle)
				print timer
				#print 'found beacon!!! '
				break
			else: 
				Rcomm_circle = plt.Circle((boat['X_POS'], boat['Y_POS']),5,color='grey',fill=False)
				Rcomms.append(Rcomm_circle)
				#print 'ocean after exploration', ocean_after_exploration

		times.append(timer)

	avg_time = np.average(times)
	print 'average time is: ', avg_time


	# fig = plt.figure(figsize=(10,10))
	# ax = fig.add_subplot(111)
	# ax.plot(*zip(*boat_locations), color="blue", marker='s', label="Location of the AUV")
	# ax.plot(*zip(*beacon_locations), color="green", marker='8', label="Location of the Drifter")
	# for radius in Rcomms:
	# 	ax.add_artist(radius)
	# ax.set_xlim([0,X_SIZE])
	# ax.set_xlabel('X coordinate')
	# ax.set_ylim([0,Y_SIZE])
	# ax.set_ylabel('Y coordinate')
	# ax.set_title('Relative locations of the AUV and the Drifter')
	# ax.legend(loc=4)
	# plt.show()

	# fig = plt.figure(figsize=(10,10))
	# ax = fig.add_subplot(111)
	# ax.plot(*zip(*boat_locations), color="blue", marker='s', label="Location of the AUV")
	# ax.plot(*zip(*beacon_locations), color="green", marker='8', label="Location of the Drifter")
	# for radius in Rcomms:
	# 	ax.add_artist(radius)
	# ax.set_xlim([X_SIZE/2 - 5, X_SIZE/2 + 5])
	# ax.set_xlabel('X coordinate')
	# ax.set_ylim([Y_SIZE/2 - 5, Y_SIZE/2 + 5])
	# ax.set_ylabel('Y coordinate')
	# ax.set_title('Close-Up of Drifter Motion')
	# ax.legend(loc=4)
	# plt.show()



	


