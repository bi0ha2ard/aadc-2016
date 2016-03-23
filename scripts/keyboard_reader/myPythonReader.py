#!/usr/bin/python
import struct
import time
import sys
import thread
import commands

myDelay=0.05
outfile_path="myKeyboardRemote.txt"
#myDeviceName="AT Translated Set 2 keyboard"#for Cezar DELL standard keyboard
#myDeviceName="Logitech K400"#for Logitech keyboard
myDeviceName="  Mini Keyboard"#for Mini keyboard

myKeyCodes=[]
myKeyState=[]
myKeyCodes.append(16)  #left
myKeyCodes.append(17)  #right
myKeyCodes.append(103)  #up
myKeyCodes.append(108)  #down
myKeyCodes.append(28)   #space
myKeyCodes.append(41)   #0
myKeyCodes.append(2)    #2
myKeyCodes.append(3)    #3
myKeyCodes.append(4)    #4
myKeyCodes.append(5)    #5
myKeyCodes.append(6)    #6
myKeyCodes.append(7)    #7
myKeyCodes.append(8)    #8
myKeyCodes.append(9)    #9
myKeyCodes.append(10)   #10
myKeyCodes.append(11)   #10
myKeyCodes.append(33)   #F
myKeyCodes.append(34)   #G
myKeyCodes.append(35)   #H
myKeyCodes.append(36)   #J
myKeyCodes.append(37)   #K

for i in range(0,len(myKeyCodes)):
    myKeyState.append(0)

def printKeys(threadName,delay):
    while 1:
        outfile = open(outfile_path, "wb")
        myOutString=""
        for c in range(0,len(myKeyCodes)):
            myOutString+="%d " % (myKeyState[c])
        outfile.write(myOutString)
        outfile.close()
        time.sleep(delay)

#start search for the device
print "Searching for device ["+myDeviceName+"]"
myCommands, out =commands.getstatusoutput('cat /proc/bus/input/devices')
myComponents=out.splitlines()
myCheckName='N: Name="'+myDeviceName+'"'
myDeviceFound=False
myDeviceEvent=""
for j in myComponents:
    #print "Checking for ["+myCheckName+"]"
    #print j
    if myCheckName==j:
        print "Found ["+myDeviceName+"]"
        myDeviceFound=True
    if myDeviceFound:
        if j.startswith("H: Handlers="):
            print "Found handlers ["+j+"]"
            myWords=j.split()
            for k in myWords:
                #print k
                if k.startswith("event"):
                  myDeviceEvent=k
                  print "Extracted event ID ["+myDeviceEvent+"]"  
            break

if not myDeviceFound:
    print "Device/event handler not found!"
    sys.exit()

thread.start_new_thread(printKeys, ("Thread-1", myDelay ))


infile_path = "/dev/input/" + myDeviceEvent
print "Going to open ["+infile_path+"]"

#long int, long int, unsigned short, unsigned short, unsigned int
FORMAT = 'llHHI'
EVENT_SIZE = struct.calcsize(FORMAT)

#open file in binary mode
in_file = open(infile_path, "rb")
print "Writing to "+outfile_path+" every "+str(myDelay)+"s"
event = in_file.read(EVENT_SIZE)

while event:
    (tv_sec, tv_usec, type, code, value) = struct.unpack(FORMAT, event)
    if type != 0 or code != 0 or value != 0:
        #print("Event type %u, code %u, value: %u at %d, %d" % \
        #    (type, code, value, tv_sec, tv_usec))
    #else:
        # Events with code, type and value == 0 are "separator" events
    #    print("===========================================")
        for c in range(0,len(myKeyCodes)):
            #print "check with %d" % c
            if code==myKeyCodes[c] and type==1:
                myKeyState[c]=value    
    event = in_file.read(EVENT_SIZE)
in_file.close()
