import uRAD_RP_SDK11		# import uRAD library

# input parameters
mode = 2					# sawtooth mode
f0 = 5						# starting at 24.005 GHz
BW = 240					# using all the BW available = 240 MHz
Ns = 200					# 200 samples
Ntar = 3					# 3 target of interest
Rmax = 100					# searching along the full distance range
MTI = 0						# MTI mode disable because we want information of static and moving targets
Mth = 0						# parameter not used because "movement" is not requested
Alpha = 10					# signal has to be 10 dB higher than its surrounding
distance_true = True 		# Request distance information
velocity_true = False		# mode 2 does not provide velocity information
SNR_true = True 			# Signal-to-Noise-Ratio information requested
I_true = False 				# In-Phase Component (RAW data) not requested
Q_true = False 				# Quadrature Component (RAW data) not requested
movement_true = False 		# Not interested in boolean movement detection

# Method to correctly turn OFF and close uRAD
def closeProgram():
	# switch OFF uRAD
	return_code = uRAD_RP_SDK11.turnOFF()
	exit()

# switch ON uRAD
return_code = uRAD_RP_SDK11.turnON()
if (return_code != 0):
	closeProgram()

# loadConfiguration uRAD
return_code = uRAD_RP_SDK11.loadConfiguration(mode, f0, BW, Ns, Ntar, Rmax, MTI, Mth, Alpha, distance_true, velocity_true, SNR_true, I_true, Q_true, movement_true)
if (return_code != 0):
	closeProgram()

# infinite detection loop
while True:

	# target detection request
	return_code, results, raw_results = uRAD_RP_SDK11.detection()
	if (return_code != 0):
		closeProgram()

	# Extract results from outputs
	NtarDetected = results[0]
	distance = results[1]
	SNR = results[3]

	# Iterate through desired targets
	for i in range(NtarDetected):
		# If SNR is big enough
		if (SNR[i] > 0):
			# Prints target information
			print("Target: %d, Distance: %1.2f m, SNR: %1.1f dB" % (i+1, distance[i], SNR[i]))

	# If number of detected targets is greater than 0 prints an empty line for a smarter output
	if (NtarDetected > 0):
		print(" ")
