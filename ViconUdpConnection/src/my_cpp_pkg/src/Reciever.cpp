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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define PORT	 8080 
#define MAXLINE 1024 

void receiveData(int sockfd, struct sockaddr_in servaddr, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub) {
    char buffer[MAXLINE];
    socklen_t len = sizeof(servaddr);
    while (rclcpp::ok()) {
        int n = recvfrom(sockfd, buffer, MAXLINE, MSG_WAITALL, (struct sockaddr *) &servaddr, &len); 
        if (n < 0) {
            perror("recvfrom failed");
            break;
        }
        buffer[n] = '\0'; 
        std::cout << "Server: " << buffer << std::endl;

        // Create a ROS message and publish it
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = buffer;
        pub->publish(*msg);
    }
}

// Driver code 
int main(int argc, char **argv) { 
    // Initialize the ROS node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("udp_receiver");

    // Create a ROS publisher
    auto pub = node->create_publisher<std_msgs::msg::String>("vicon_udp_data", 10);
    
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

    rclcpp::spin(node);

    receiveThread.join();

    close(sockfd); 
    rclcpp::shutdown();
    return 0; 
}