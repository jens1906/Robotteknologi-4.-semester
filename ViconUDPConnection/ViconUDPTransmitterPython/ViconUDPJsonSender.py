from vicon_dssdk import ViconDataStream
import time
import socket
import json
import datetime

#Define raspberry access point
raspberryIp = "192.168.1.34"  
raspberryPort = 8081  

# Create a UDP socket
raspberrySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Create a connection to the Vicon DataStream system(Known to be the localhost at port 801)
viconClient = ViconDataStream.RetimingClient()
viconClient.Connect("localhost:801")
print(f"Version:{viconClient.GetVersion()}")

# Set the axis mapping aka define which directions that is up and down and so on
viconClient.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp)

#Lets Keep the data coming
while True:
    try:
        viconClient.UpdateFrame() # Update the frame

        #The way this works is that you access and get information about the different subjects within the system, so you need to access them, therefore we neew the list
        subjectNames = viconClient.GetSubjectNames()        # Get the list of subjects

        #Go through all subjects basicly groups in vicon.
        for subjectName in subjectNames:
            #Same concept as before, but we only have 1 so it is fine
            segmentNames = viconClient.GetSegmentNames(subjectName)

            #Go through all segments in the subject, which is individual parts, here the drone.
            for segmentName in segmentNames:
                #Now we can get local and global translation, which is the same because we are within a confined space, so we do global
                viconGlobalPosition = tuple(round(x / 1000, 6) for x in viconClient.GetSegmentGlobalTranslation(subjectName, segmentName)[0])
                viconGlobalEuler = tuple(round(x, 4) for x in viconClient.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName)[0])

                formattedJsonData = json.dumps({
                    #"Time": datetime.datetime.now().strftime("%H:%M:%S")+ f":{datetime.datetime.now().microsecond // 1000:03d}",
                    "Position(M)":  (viconGlobalPosition[0],
                                    viconGlobalPosition[1],
                                    viconGlobalPosition[2]),
                    "Euler(rad)":   (viconGlobalEuler[0],
                                    viconGlobalEuler[1],
                                    viconGlobalEuler[2])
                })
                print(formattedJsonData)
                raspberrySocket.sendto(formattedJsonData.encode(), (raspberryIp, raspberryPort))

        # Sleep for a short period to avoid overwhelming the system
        time.sleep(0.000001)

    except ViconDataStream.DataStreamException as e:
        print('Error:', e)

