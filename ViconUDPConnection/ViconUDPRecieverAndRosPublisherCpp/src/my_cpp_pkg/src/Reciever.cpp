// Client side implementation of UDP client-server model 
#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <thread>
#include <nlohmann/json.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Define the port and the maximum message length/buffer size
#define PORT	 8081
#define MAXLINE 1024 

void receiveData(int udpSocket, struct sockaddr_in serverAdress, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rosPublisher) {
    // Create a buffer to store the data
    char buffer[MAXLINE];
    socklen_t messageLength = sizeof(serverAdress);

    // Loop to receive the data
    while (rclcpp::ok()) {
        // Receive the data from the UDP socket
        int n = recvfrom(udpSocket, buffer, MAXLINE, MSG_WAITALL, (struct sockaddr *) &serverAdress, &messageLength); 
        // Check for fail
        if (n < 0) {
            perror("recvfrom failed");
            break;
        }
        buffer[n] = '\0'; 
        nlohmann::json RecievedData = nlohmann::json::parse(buffer);

        // Create the ROS data format
        std_msgs::msg::Float64MultiArray viconMessage;
        // Save the data in the ROS message
        for (int i = 0; i < 2; ++i) {
            viconMessage.data.push_back(RecievedData["Time"][i]);  // Time
        }
        for (int i = 0; i < 3; ++i) {
            viconMessage.data.push_back(RecievedData["Position(M)"][i]);  // Position
        }
        for (int i = 0; i < 3; ++i) {
            viconMessage.data.push_back(RecievedData["Euler(rad)"][i]);  // Euler
        }

        // Print the data to the console
        //msg.data.push_back(RecievedData["Time"]);'
        std::cout << "Server: " << std::endl;
        std::cout << "Time: " << RecievedData["Time"] << std::endl;
        std::cout << "Position: " << RecievedData["Position(M)"] << std::endl;
        std::cout << "Euler: " << RecievedData["Euler(rad)"] << std::endl;
        std::cout << "Array: ";
        
        // Print the Vicon data to the console
        for (size_t i = 0; i < viconMessage.data.size(); ++i) {
            std::cout << viconMessage.data[i] << " ";
        }
        std::cout << std::endl;

        // Publish the data to the ROS topic
        rosPublisher->publish(viconMessage);
    }
}

//The main loop which start the code 
int main(int argc, char **argv) { 
    // Initialize the ROS node
    rclcpp::init(argc, argv);
    auto rosNode = rclcpp::Node::make_shared("udp_receiver");

    // Create a ROS publisher which publishes a float array
    auto rosPublisher = rosNode->create_publisher<std_msgs::msg::Float64MultiArray>("vicon_udp_data", 10);
    
    //Variables for creating socket
    int udpSocket; 
    struct sockaddr_in serverAdress; 

    // Creating a socket file descriptor. Socket=(IPv4, UDP protocol, default), if this fails it will become -1 and fail
    if ((udpSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 

    // Creating and clearing the server address, basicly set all to 0 and then set the port and IP later to ensure the right data
    memset(&serverAdress, 0, sizeof(serverAdress)); 
    
    // Set the server address to the right values 
    serverAdress.sin_family = AF_INET; 
    serverAdress.sin_port = htons(PORT); 
    serverAdress.sin_addr.s_addr = INADDR_ANY; 

    // Bind the socket with the server address 
    if (bind(udpSocket, (const struct sockaddr *)&serverAdress, sizeof(serverAdress)) < 0) { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 

    std::thread receiveThread(receiveData, udpSocket, serverAdress, rosPublisher);

    // Spin the node in a separate thread to allow signal handling
    std::thread rosSpinThread([&]() {
        rclcpp::spin(rosNode);
    });

    // Wait for the signal to stop the program
    rosSpinThread.join();
    receiveThread.join();

    close(udpSocket); 
    return 0; 
}