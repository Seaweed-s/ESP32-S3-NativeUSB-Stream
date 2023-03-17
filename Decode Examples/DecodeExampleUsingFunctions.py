from math import floor
import serial.tools.list_ports
import serial
import cv2
import numpy as np
import time

# Function for initiating a serial connection
def openSerialConnection(port):
    serialInst = serial.Serial()
    print("setting baudrate")
    serialInst.baudrate = 1000000
    print("baudrate set")
    serialInst.port = port
    serialInst.setDTR(False)
    serialInst.setRTS(False)
    serialInst.open()
    print("port open")
    return serialInst

# Function from getting a frame from the given serial connection
def getFrame(serialInst, byteBuffer):
    if serialInst.in_waiting:
        #print("Serial in waiting")
        byteBuffer += serialInst.read(2048)

        imgStart = byteBuffer.find(b'\xff\xd8') # Find the start of the jpeg image 
        imgEnd = byteBuffer.find(b'\xff\xd9')   # Find the end of the jpeg image

        while imgStart > imgEnd: # Continue reading in data until an image is found in the buffer
            #print("Valid image not found, reading more data")
            byteBuffer = byteBuffer[imgStart:]
            imgStart = byteBuffer.find(b'\xff\xd8')
            imgEnd = byteBuffer.find(b'\xff\xd9')
            print(imgStart)
            if imgStart == -1 or imgEnd == -1:
                byteBuffer += serialInst.read(2048)
        
        if imgStart != -1 and imgEnd != -1:
            size = len(byteBuffer)  # Find the buffer size to calculate bandwidth
            jpg = byteBuffer[imgStart:imgEnd+2]     # Grab jpeg data from buffer
            byteBuffer = byteBuffer[imgStart+2:]    # Remove the jpeg data from the buffer
            print(len(jpg))
            
            if jpg:
                try:
                    image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)  # Decode jpeg data into a CV2 image
                    return image, byteBuffer
                except:
                    print("failed to decode (likely corrupt JPEG data)")
                    return False, byteBuffer

# Prints COM ports
ports = serial.tools.list_ports.comports()
portList = []
for onePort in ports:
    portList.append(str(onePort))
    print(str(onePort))

portSelection = input("Select Port: ")  # User port selection

serialConnection = openSerialConnection(portSelection)  # Open serial connection
byteBuffer = b''

while True:
    timer = time.time() # For calculating FPS
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break
    
    
    try:    # Attempt to show image and calculate FPS
        image, byteBuffer = getFrame(serialConnection, byteBuffer)  # Get CV2 image
        cv2.imshow('jpeg decode', image)
        FPS = 1/(time.time() - timer)
        print("FPS: ", FPS)
        timer = time.time()
    except:
        print("Could not show image (image data may not be valid)")

serialConnection.close()
cv2.destroyAllWindows()
