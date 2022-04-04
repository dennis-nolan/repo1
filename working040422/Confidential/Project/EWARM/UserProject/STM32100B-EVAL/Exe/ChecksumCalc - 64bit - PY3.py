import sys, struct, array, os

data = array.array('L')

#read entire binary
with open(sys.argv[1], 'rb') as f:
	fileString = bytearray(f.read())
	
	#ensure it is a multiple of 8 bytes, pad with 0
	while(len(fileString)%8 > 0):
		fileString.append(0)
	
	data.fromstring(fileString)
	#print data
	print ("Read file. Length: " + str(len(data)*4))
	
	#find start marker to fill out checksum and length
	start = data.index(0x89ABCDEF)
	data[start+2] = len(data) * 4
	
	#compute checksum
	checksum1 = 0xA6A6A6A6
	checksum2 = 0xC8C8C8C8
	legacyStart = data.index(0xFEDCBA98)+1
	skip = data.index(0xFEDCBA98)-1
	skip2 = data.index(0x89ABCDEF) + 1
	#print str(skip)
	#print hex(data[skip])
	
	for i in range(0, len(data)):
		if (i != skip and i != skip2):
			checksum2 = (checksum2 + data[i]) & 0xFFFFFFFF
		if (i >= legacyStart):
			checksum1 = (checksum1 + data[i]) & 0xFFFFFFFF
	checksum2 = (checksum2 + checksum1) & 0xFFFFFFFF
	print ("Computed checksums: " + hex(checksum1) + " " + hex(checksum2))
	
	data[start+1] = checksum1
	#data[start+2] = len(data) * 4
	data[skip] = checksum2
	print ("Modified binary in memory")
	

if (len(data) > 0):
	#write result to disk
	with open(sys.argv[1], 'wb') as f:
		data.tofile(f)
		if len(fileString)%8 > 0:
			f.write(fileString[len(fileString)%8 * -1:])
		print ("Wrote file to disk")
	

os.system('pause')