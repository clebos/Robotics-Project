from __future__ import division
import numpy as np
import pylab as pl
import scipy.signal as sig
import scipy.spatial.distance as dist
import random as rand
import matplotlib.pyplot as plt
import math

# Function to represent the robots mental percepton of where the drifter is after
# 1 time step
def blur_probability(ocean, kernel):
	ocean_convolved = sig.convolve2d(ocean, kernel, mode='same')
	# print "type of ocean convolved ", type(ocean_convolved)
	return ocean_convolved

# Function to pick a motion for the boat given what it knows about the probable
# location of the drifter.
# Initially this is set to choose a step in the most probable direction
def choose_motion(boat, ocean, previous_max_x, previous_max_y, previous_action, previous_scalar, spiral_distance_left, spiralling,):
	# print 'chosing motion '
	# print 'VALUES of the following variables'
	# print 'VALUES of boat ', boat
	# print 'VALUES of ocean ', ocean
	# print 'VALUES of previous_max_x ', previous_max_x
	# print 'VALUES of previous_max_y ', previous_max_y
	# print 'VALUES of previous_action ', previous_action
	# print 'VALUES of previous_scalar ', previous_scalar
	# print 'VALUES of spiralling ', spiralling

	most_likely_x, most_likely_y = np.unravel_index(ocean.argmax(), ocean.shape)
	current_x = boat['X_POS']
	current_y = boat['Y_POS']

	# If the location of the previous max is different from the location of the current max,
	# that clearly means that we have set the location at the previous max to zero - ie. we 
	# are in the correct location, but we haven't yet found the drifter so we should start looking
	# around in the area via a spiral instead of blindly heading to the max likelihood location.
	if (previous_max_x != most_likely_x and  previous_max_y != most_likely_y and previous_max_x != -1 and previous_max_y != -1) or spiralling == True:
		# print 'trying to spiral'
		spiralling = True
		new_x, new_y, action, scalar, distance_left = outward_spiral(current_x, current_y, previous_action, previous_scalar, spiral_distance_left) 

		# Check to make sure that you're not accidentally spiralling out of the world boundary 
		if new_x > len(ocean):
			new_x = len(ocean)
		if new_x < 0:
			new_x = 0
		if new_y > len(ocean):
			new_y = len(ocean)
		if new_y < 0:
			new_y = 0

		new_boat = {'X_POS': new_x, 'Y_POS':new_y}
	
		return new_boat, most_likely_x, most_likely_y, action, scalar, distance_left, spiralling 
	
	else:
		# print 'entering else '
		# Spiralling parameters don't change: 
		action = previous_action
		scalar = previous_scalar
		distance_left = spiral_distance_left

		# print "current x ", current_x
		# print "current y", current_y

		# Choose a step toward the most likely x and y position and then add noise
		# In this case, we use 1 as the maximal amount that the boat can move, 
		# taken from the paper where the robot's swims at 1 m/s
		# *****IN FUTURE***** might want to change the step size (based on boat speed etc)

		# Use the idea of similar triangles to 
		dist_x = most_likely_x - current_x
		# print "x dist ", dist_x
		dist_y = most_likely_y - current_y
		# print "y dist ", dist_y
		theta = math.atan(dist_y/dist_x)
		# print "theta ", theta
		# Because of the way angles work, if x is less than 0 we have to add pi to get the 
		# correct angle
		if dist_x < 0:
			theta = theta + math.pi

		# print "corrected theta ", theta

		# We know that the distance of the hypoteneuse must be 1, so using this and the angle we can
		# calculate the distance we need to move in the x and y direction 
		new_y = current_y + math.sin(theta)
		new_x = current_x + math.cos(theta)
		# print "new x ", new_x
		# print "new y ", new_y

		new_boat = {'X_POS': new_x, 'Y_POS':new_y}	

		return new_boat, most_likely_x, most_likely_y, action, scalar, distance_left, spiralling 

# Write a method to decode the parts of the spiral
# Our previous action tells us what to do next 
# 1 = right, 2 = down, 3 = left, 4 = up
def action_decoder (previous_action, previous_scalar):
	# If the previous action is 4, that means that we are at the end of our cycle and we want to start
	# at 0 again
	if previous_action == 4:
		previous_action = 0

	# If the previous action is 0 that means we want to move right by the previous scalar + 1
	if previous_action == 0:
		action = 1
		scalar = previous_scalar + 1

	# If the previous action is 1 (right) that means we want to move down by the previous scalar
	elif previous_action == 1:
		action = 2
		scalar = previous_scalar

	# If the previous action is 2 (down) that means we want to move left by the previous scalar+1
	elif previous_action == 2:
		action = 3
		scalar = previous_scalar + 1

	# If the previous action is 3 we want to move up by the previous scalar
	elif previous_action == 3:
		action = 4
		scalar = previous_scalar

	return action, scalar

def outward_spiral(current_x, current_y, previous_action, previous_scalar, spiral_distance_left):
	# Get which action we should perform next
	# If the spiral distance left is greater than 0, we have to continue on our current path
	# Otherwise we can chose a new direction to go in

	new_x = current_x
	new_y = current_y

	if spiral_distance_left > 0:
		current_action  = previous_action
		current_scalar = previous_scalar
		distance_to_go = spiral_distance_left - 1
	else:
		current_action, current_scalar = action_decoder(previous_action, previous_scalar)
		distance_to_go = current_scalar*2

	# If current action is 1 we want to go right by 4*value of current scalar 
	if current_action == 1:
		new_x = current_x + 1
		new_y = current_y

	# If current action is 2 we want to go down by 4*value of current scalar 
	elif current_action == 2:
		new_x = current_x
		new_y = current_y - 1

	# If current action is 3 we want to go left by 4*value of current scalar 
	elif current_action == 3:
		new_x = current_x - 1
		new_y = current_y

	# if current action is 4 we want to go up by 4*value of current scalar 
	elif current_action == 4:
		new_x = current_x
		new_y = current_y + 1

	return new_x, new_y, current_action, current_scalar, distance_to_go

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
	# print 'distance between boat and beacon ', dist
	# Found the beacon, so report back "None"
	if abs(dist) <= 2.5:
		return None
	# If not found the beacon, we know that all squares around the boat are 0
	else:
		boat_x = int(boat['X_POS'])
		boat_y = int(boat['Y_POS'])
		# print 'boat x ', boat_x
		# print 'boat_y ', boat_y
		added_x = boat_x
		added_y = boat_y

		# print 'ocean length ', len(ocean)
		for i in range(-2,2):
			for j in range (-2, 2):
				if ((boat_x + i) >= len(ocean)) or ((boat_x + i) < 0):
					# print " x too big!  skipping... ", boat_x + i
					pass
				else: 
					added_x = boat_x + i
				if ((boat_y + j) >= len(ocean)) or ((boat_y + j) < 0):
					# print "y too big! skipping ... ", boat_y + j
					pass
				else:
					added_y = boat_y + j
				# print "added x ", added_x
				# print "added y ", added_y
				ocean[added_x, added_y] = 0
		return ocean

def drift_beacon(beacon):
	new_beacon_x = beacon['X_POS'] + rand.uniform(-0.2, 0.2)
	new_beacon_y = beacon['Y_POS'] + rand.uniform(-0.2,0.2)
	new_beacon = {'X_POS': new_beacon_x, 'Y_POS':new_beacon_y}
	return new_beacon

if __name__ == '__main__':

	p_zeroes = []
	p_ones = []
	misses = []

		# for 100 Trials: 
	for j in range (0, 100):
		print 'j is ', j
		# 100 Trials
		times_per_trial = []
		initial_distances = []
		miss = 0

		for i in range(0,100):
			# print i
			#### SET UP OF THE INTITIAL VARIABLES #### 
				# Set up the ocean as an array of floats, intially all intialized to 0
			# Initial array = 99 x 99 
			X_SIZE = 100
			Y_SIZE = 100
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
			beacon = {'X_POS': rand.uniform(0,X_SIZE) , 'Y_POS': rand.uniform(0,Y_SIZE)}
			# beacon = {'X_POS': (X_SIZE-1)/2 , 'Y_POS': (Y_SIZE-1)/2}
			# print "initial becaon position: ", (beacon['X_POS'], beacon['Y_POS'])
			ocean[beacon['X_POS'], beacon['Y_POS']] = 1.0
			
			# Define the boat as a dictionary with key-value pairs corresponding to position:
			# *****IN FUTURE***** we may want to add velocity to the boat's defintion 
			boat = {'X_POS': rand.uniform(0,X_SIZE), 'Y_POS':rand.uniform(0,Y_SIZE)}
			# boat = {'X_POS': X_SIZE, 'Y_POS':0}  
			# print "initial boat position: ", (boat['X_POS'], boat['Y_POS'])
			
			initial_distance = distance(boat, beacon)
			
			## Set up some arrays for plotting later ## 
			boat_locations = [(boat['X_POS'], boat['Y_POS'])]
			beacon_locations = [(beacon['X_POS'], beacon['Y_POS'])]
			Rcomms = [plt.Circle((boat['X_POS'], boat['Y_POS']),2.5,color='grey',fill=False)]
				
			# Initializing the spiralling vairables
			previous_max_x = -1
			previous_max_y = -1
			previous_action = 0
			previous_scalar = 0
			spiral_distance_left = 0
			spiralling = False
			#print "done setting up ...... "
			
			# Increment the time step by 1 
			while timer < 1000:
				# print('in the while loop ')
				# print'timer in loop', timer
				# # print"boat ", boat
				# print"beacon ", beacon
				# print "distance, ", distance(boat, beacon) 
				# print "ocean length ", len(ocean)
				# print "entering while loop"
				timer = timer + 1
				# print "timer is : ", timer
				# drift the beacon a little bit 
				new_beacon = drift_beacon(beacon)
				# print 'new beacon: ', new_beacon
				# blur the probability distribution
				new_ocean = blur_probability(ocean, kernel)
				# print "type of new ocean ", type(new_ocean)
				# print "new ocean length ", len(new_ocean)
				# print 'convolved ocean is: ', new_ocean
				# choose a direction for the boat to travel in
				# print 'types of the following variables'
				# print 'type of boat ', type(boat)
				# print 'type of ocean ', type(ocean)
				# print 'type of previous_max_x ', type(previous_max_x)
				# print 'type of previous_max_y ', type(previous_max_y)
				# print 'type of previous_action ', type(previous_action)
				# print 'type of previous_scalar ', type(previous_scalar)
				# print 'type of spiralling ', type(spiralling)

				new_boat, most_likely_x, most_likely_y, action, scalar, distance_left, spiralling = choose_motion(boat,ocean, previous_max_x, previous_max_y, previous_action, previous_scalar, spiral_distance_left, spiralling)
				# print 'new boat: ', new_boat
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
				previous_max_x = most_likely_x
				previous_max_y = most_likely_y
				previous_action = action
				previous_scalar = scalar
				spiral_distance_left = distance_left
				
				if (ocean_after_exploration == None):
					Rcomm_circle = plt.Circle((boat['X_POS'], boat['Y_POS']),2.5,color='red',fill=False)
					Rcomms.append(Rcomm_circle)
					# print timer
					# print 'found beacon!!! '
					break
				else: 
					Rcomm_circle = plt.Circle((boat['X_POS'], boat['Y_POS']),2.5,color='grey',fill=False)
					Rcomms.append(Rcomm_circle)
					# print 'ocean after exploration', ocean_after_exploration

			if (timer == 1000):
				print "timer too large "
				miss = miss + 1
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
				# ax.legend(loc=0)
				# plt.show()

			else:
				initial_distances.append(initial_distance)
				times_per_trial.append(timer)


		p = np.polyfit(initial_distances,times_per_trial,1)
		p_zeroes.append(p[0])
		p_ones.append(p[1])
		misses.append(miss)

	average_number_misses = np.average(misses)
	average_slope = np.average(p_zeroes)
	average_intercept = np.average(p_ones)
	print 'm = ', average_slope
	print 'b = ', average_intercept
	print "average misses ", average_number_misses


# ### PLOTTING AUV AND DRIFTER #### 

# 	fig = plt.figure(figsize=(10,10))
# 	ax = fig.add_subplot(111)
# 	ax.plot(*zip(*boat_locations), color="blue", marker='s', label="Location of the AUV")
# 	ax.plot(*zip(*beacon_locations), color="green", marker='8', label="Location of the Drifter")
# 	for radius in Rcomms:
# 		ax.add_artist(radius)
# 	ax.set_xlim([0,X_SIZE])
# 	ax.set_xlabel('X coordinate')
# 	ax.set_ylim([0,Y_SIZE])
# 	ax.set_ylabel('Y coordinate')
# 	ax.set_title('Relative locations of the AUV and the Drifter')
# 	ax.legend(loc=4)
# 	plt.show()

#### CLOSE UP OF DRIFTER ####
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

# #### PLOTTING THE TIMES FOR VARIOUS WORLD SIZES ####
# 	fig = plt.figure(figsize=(10,10))
# 	ax = fig.add_subplot(111)
# 	ax.plot(range(50, 1000, 50), times_per_size)
# 	ax.set_xlabel('Search Space Size (m)')
# 	ax.set_ylabel('Time (s)')
# 	ax.set_title('Search time vs. World Size')
# 	plt.show()

# #### PLOTTING RANDOMIZED AUV AND DRIFTER time vs distance#### 

# 	fig = plt.figure(figsize=(10,10))
# 	ax = fig.add_subplot(111)
# 	p = np.polyfit(initial_distances,times_per_trial,1)
# 	ax.scatter(initial_distances, times_per_trial)
# 	ax.plot(initial_distances, p[0]*initial_distances+p[1], color='red')
# 	print 'm = ', p[0]
# 	print 'b = ', p[1]
# 	ax.set_xlabel('Initial Distance between AUV and Drifter (m)')
# 	ax.set_ylabel('Time (s)')
# 	ax.set_title('Distance vs. Time for Random Initial Distances')
# 	plt.show()




	


