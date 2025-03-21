from vicon_dssdk import ViconDataStream
import time
import socket
import json
import datetime

#Define raspberry access point
RaspberryIp = "192.168.1.34"  
RaspberryPort = 8081  

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Create a connection to the Vicon DataStream system
client = ViconDataStream.RetimingClient()
client.Connect("localhost:801")
print(f"Version:{client.GetVersion()}")

# Set the axis mapping aka define which directions that is up and down and so on
client.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp)

#Lets Keep the data coming
while True:
    try:
        client.UpdateFrame() # Update the frame

        #The way this works is that you access and get information about the different subjects within the system, so you need to access them, therefore we neew the list
        subject_names = client.GetSubjectNames()        # Get the list of subjects

        #Go through all
        for subject_name in subject_names:
            #Same concept as before, but we only have 1 so it is fine
            segment_names = client.GetSegmentNames(subject_name)
            for segment_name in segment_names:
                #Now we can get local and global translation, which is the same because we are within a confined space, so we do global
                ViconGlobalPosition = tuple(round(x / 1000, 6) for x in client.GetSegmentGlobalTranslation(subject_name, segment_name)[0])
                ViconGlobalEuler = tuple(round(x, 4) for x in client.GetSegmentGlobalRotationEulerXYZ(subject_name, segment_name)[0])
                
                #Then we want the message sen via UDP so we change it to a better format so
                current_time = datetime.datetime.now()

                FormattedJsonData = json.dumps({
                    #"Time": datetime.datetime.now().strftime("%H:%M:%S")+ f":{datetime.datetime.now().microsecond // 1000:03d}",
                    "Position(M)": (ViconGlobalPosition[0],
                                 ViconGlobalPosition[1],
                                 ViconGlobalPosition[2]),
                    "Euler(rad)": (ViconGlobalEuler[0],
                              ViconGlobalEuler[1],
                              ViconGlobalEuler[2])
                })
                print(FormattedJsonData)
                
                sock.sendto(FormattedJsonData.encode(), (RaspberryIp, RaspberryPort))
                #sock.sendto("Test".encode(), (RaspberryIp, RaspberryPort))

        # Sleep for a short period to avoid overwhelming the system
        time.sleep(0.000001)

    except ViconDataStream.DataStreamException as e:
        print('Error:', e)

