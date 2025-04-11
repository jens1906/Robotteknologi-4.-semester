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
#include <fcntl.h>


#define PORT	 8080 
#define MAXLINE 1024 
#define IP "172.26.51.94" 

void sendData(int sockfd, struct sockaddr_in servaddr, const char* message) {
    int time = 0;
    std::string finalMessageStr;
    finalMessageStr.reserve(MAXLINE); // Preallocate memory for the final message
    while (true) {
        finalMessageStr = std::string(message) + std::to_string(time);
        const char* FinalMessage = finalMessageStr.c_str();
        sendto(sockfd, FinalMessage, strlen(FinalMessage), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr)); 
        std::cout << "Message sent: " << FinalMessage << std::endl; // Comment out or remove this line
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Reduce sleep duration
        time++;
    }
}

// Driver code 
int main() { 
    int sockfd; 
    const char *hello = ""; 
    struct sockaddr_in servaddr; 

    // Creating socket file descriptor 
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 

    // Set socket to non-blocking mode
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    memset(&servaddr, 0, sizeof(servaddr)); 
    
    // Filling server information 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_port = htons(PORT); 
    servaddr.sin_addr.s_addr = inet_addr(IP); 

    std::thread sendThread(sendData, sockfd, servaddr, hello);

    sendThread.join();

    close(sockfd); 
    return 0; 
}