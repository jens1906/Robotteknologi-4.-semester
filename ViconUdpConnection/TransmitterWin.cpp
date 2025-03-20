// Client side implementation of UDP client-server model with Vicon DataStream SDK
#include <iostream>
#include <string>
#include <thread>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <DataStreamClient.h>

#pragma comment(lib, "Ws2_32.lib")

#define PORT     8080 
#define MAXLINE  1024 
#define IP       "172.26.51.94"

using namespace ViconDataStreamSDK::CPP;

void sendData(SOCKET sockfd, struct sockaddr_in servaddr, const char* message) {
    int time = 0;
    std::string finalMessageStr;
    finalMessageStr.reserve(MAXLINE); // Preallocate memory for the final message
    while (true) {
        finalMessageStr = std::string(message) + std::to_string(time);
        const char* FinalMessage = finalMessageStr.c_str();
        sendto(sockfd, FinalMessage, strlen(FinalMessage), 0, (const struct sockaddr *) &servaddr, sizeof(servaddr)); 
        std::cout << "Message sent: " << FinalMessage << std::endl; // Comment out or remove this line
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Reduce sleep duration
        time++;
    }
}

// Driver code 
int main() { 
    WSADATA wsaData;
    SOCKET sockfd; 
    const char *hello = ""; 
    struct sockaddr_in servaddr; 

    // Initialize Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed with error: " << WSAGetLastError() << std::endl;
        return EXIT_FAILURE;
    }

    // Creating socket file descriptor 
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) { 
        std::cerr << "Socket creation failed with error: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return EXIT_FAILURE;
    } 

    memset(&servaddr, 0, sizeof(servaddr)); 
    
    // Filling server information 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_port = htons(PORT); 
    inet_pton(AF_INET, IP, &servaddr.sin_addr); 

    // Initialize Vicon DataStream SDK
    Client MyClient;
    std::string HostName = "localhost:801";
    MyClient.Connect(HostName);

    // Check connection status
    while (!MyClient.IsConnected().Connected) {
        std::cout << "Connecting to Vicon DataStream..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Connected to Vicon DataStream." << std::endl;

    // Enable data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableDeviceData();

    // Set the streaming mode
    MyClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    // Main data retrieval loop
    while (true) {
        // Get a frame
        MyClient.GetFrame();

        // Retrieve and process data here
        // For example, get the subject count
        unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
        std::cout << "Number of subjects: " << SubjectCount << std::endl;

        // Send data over UDP
        sendData(sockfd, servaddr, hello);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    MyClient.Disconnect();

    closesocket(sockfd); 
    WSACleanup();
    return 0; 
}