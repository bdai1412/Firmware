# GPS position in WGS84 coordinates.
float64 uavTimestamp		#timestamp for communication: seconds
uint8 status			#status: 1.rotors start, 2.take off, 3.follow usv, 4.land, 5.go around, 6.shut down

float64 latitude			# Latitude in 1E-7 degrees
float64 longitude			# Longitude in 1E-7 degrees 
float64 altitude			# Altitude in 1E-3 meters above MSL, (millimetres)

float32 roll
float32 pitch
float32 yaw

float32 velocityNorth
float32 velocityEast
float32 velocityDown

float32 gyroscopeX
float32 gyroscopeY
float32 gyroscopeZ

float32 accelerometerX
float32 accelerometerY
float32 accelerometerZ

uint8 satellitesUsed
uint8 fixType
float32 batteryRemain