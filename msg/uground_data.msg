# GPS position in WGS84 coordinates.
uint64 usvTimestamp		#timestamp for communication
uint8 setting			#setting: 1.rotors start, 2.take off, 3.follow usv, 4.land, 5.go around, 6.shut down

float64 targetLatitude			# Latitude in 1E-7 degrees
float64 targetLongitude			# Longitude in 1E-7 degrees 
float64 targetAltitude			# Altitude in 1E-3 meters above MSL, (millimetres)

float32 targetHeading
float32 targetVelocity
float32 desiredAltitude