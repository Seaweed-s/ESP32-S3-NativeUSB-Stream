from math import floor
import serial.tools.list_ports
import serial
import cv2
import numpy as np
import time


ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

portList = []

for onePort in ports:   # Prints COM ports
    portList.append(str(onePort))
    print(str(onePort))

# Initalise serial connection
print("setting baudrate")
serialInst.baudrate = 2000000
print("baudrate set")
serialInst.port = "COM8"
serialInst.setDTR(False)
serialInst.setRTS(False)
serialInst.open()
print("port open")

byteBuffer = b''
timer = time.time() # For calculating FPS and bandwidth
while True:
    #print("loop")
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

    if serialInst.in_waiting:
        print("Serial in waiting")
        byteBuffer += serialInst.read(2048)

        imgStart = byteBuffer.find(b'\xff\xd8') # Find the start of the jpeg image 
        imgEnd = byteBuffer.find(b'\xff\xd9')   # Find the end of the jpeg image

        while imgStart > imgEnd: # Continue reading in data until an image is found in the buffer
            print("Valid image not found, reading more data")
            byteBuffer = byteBuffer[imgStart:]
            imgStart = byteBuffer.find(b'\xff\xd8')
            imgEnd = byteBuffer.find(b'\xff\xd9')
            if imgStart == -1 or imgEnd == -1:
                byteBuffer += serialInst.read(2048)
            #print(a)
            #print(b)
        
        #print(a)
        #print(b)
        if imgStart != -1 and imgEnd != -1:
            size = len(byteBuffer)  # Find the buffer size to calculate bandwidth
            jpg = byteBuffer[imgStart:imgEnd+2]     # Grab jpeg data from buffer
            byteBuffer = byteBuffer[imgStart+2:]    # Remove the jpeg data from the buffer
            print("Image size: ", len(jpg),"B")
            
            if jpg:
                try:
                    image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)  # Decode jpeg data into a CV2 image
                    cv2.imshow('jpeg decode', image)
                    print("-----Frame-----")    # Data about the frame
                    FPS = 1/(time.time() - timer)
                    print("FPS: ", FPS)
                    print("Bandwidth: ", floor(size * FPS / 1000), "KB/s")
                    print("Image: ", floor(len(jpg) * FPS / 1000), "KB/s")
                    print("---------------")
                    timer = time.time()
                except:
                    print("failed to decode (likely corrupt JPEG data)")
                #serialInst.reset_input_buffer

serialInst.close()
cv2.destroyAllWindows()
