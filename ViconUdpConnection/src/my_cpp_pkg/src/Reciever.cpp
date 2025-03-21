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

#define PORT	 8081
#define MAXLINE 1024 

void receiveData(int sockfd, struct sockaddr_in servaddr, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub) {
    char buffer[MAXLINE];
    socklen_t len = sizeof(servaddr);
    while (rclcpp::ok()) {
        int n = recvfrom(sockfd, buffer, MAXLINE, MSG_WAITALL, (struct sockaddr *) &servaddr, &len); 
        if (n < 0) {
            perror("recvfrom failed");
            break;
        }
        buffer[n] = '\0'; 
        nlohmann::json RecievedData = nlohmann::json::parse(buffer);
        // Create a ROS message and populate it with the received data
        std_msgs::msg::Float64MultiArray msg;
        for (int i = 0; i < 3; ++i) {
            msg.data.push_back(RecievedData["Position(M)"][i]);  // Position
        }
        for (int i = 0; i < 3; ++i) {
            msg.data.push_back(RecievedData["Euler(rad)"][i]);  // Euler
        }
        //msg.data.push_back(RecievedData["Time"]);
        std::cout << "Server: " << std::endl;
        std::cout << "Position: " << RecievedData["Position(M)"] << std::endl;
        std::cout << "Euler: " << RecievedData["Euler(rad)"] << std::endl;
        std::cout << "Array: ";
        for (size_t i = 0; i < msg.data.size(); ++i) {
            std::cout << msg.data[i] << " ";
        }
        std::cout << std::endl;

        // Create a ROS message and publish it

        pub->publish(msg);
    }
}

// Driver code 
int main(int argc, char **argv) { 
    // Initialize the ROS node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("udp_receiver");

    // Create a ROS publisher
    auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("vicon_udp_data", 10);
    
    int sockfd; 
    struct sockaddr_in servaddr; 

    // Creating socket file descriptor 
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 

    memset(&servaddr, 0, sizeof(servaddr)); 
    
    // Filling server information 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_port = htons(PORT); 
    servaddr.sin_addr.s_addr = INADDR_ANY; 

    // Bind the socket with the server address 
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 

    std::thread receiveThread(receiveData, sockfd, servaddr, pub);

    // Spin the node in a separate thread to allow signal handling
    std::thread rosSpinThread([&]() {
        rclcpp::spin(node);
    });

    // Wait for the signal to stop the program
    rosSpinThread.join();
    receiveThread.join();

    close(sockfd); 
    return 0; 
}