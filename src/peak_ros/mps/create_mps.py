#!/usr/bin/env python3

elements = 64
aperture = 1


# Creates a 0 Deg no wedge or couplant mps script

# Add these to the beginning of the script
# ECON 0 1 1 0
# DXN 0 0
# DOF 4

for i in range(elements-aperture +1):
	print(f'TXF {i+1} 0 -1')

	for j in range(aperture):
		print(f'TXF {i+1} {j+1+i} 0')

	print(f'RXF {i+1} 0 -1 0')
	# print("")

	for k in range(aperture):
		print(f'RXF {i+1} {k+1+i} 0 0')

	# print("")
	print(f'RTD {i+1} 0')
	print(f'TXN {256+i} {i+1}')
	print(f'RXN {256+i} {i+1}')
	# print("")
print('')
print(f'SWP 1 256 - {256 + elements - aperture}')

# Add these to the end of the script 
# GANS 1 226  #Adjust Gain
# TRMS 1 0
# GATS 1 500 8000 #Adjust Gate
# DLYS 1 2308
# FRDS -1 1 1
# FRDS 1 0 3
# ENAS 1
# SGAS 1 0
# OLMS 1 0 0
# ETMS 1 0
# DTGS 1 0
# AMPS 1 3
# PAV 1 125 120
# PAW 1 125 100
# FRQS 1 2 7
# DCMS 1 0
# AWFS 1 1
# PRF 1500
