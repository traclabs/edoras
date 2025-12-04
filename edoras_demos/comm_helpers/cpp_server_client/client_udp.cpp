#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
   
#define PORT     8080 
#define MAXLINE 1024 
   
int main() { 
    int sockfd; 
    char buffer[MAXLINE]; 
    const char *msg_to_server = "Hello from client"; 
    struct sockaddr_in     server_address_; 
   
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
   
    memset(&server_address_, 0, sizeof(server_address_)); 
       
    // Filling server information 
    server_address_.sin_family = AF_INET; 
    server_address_.sin_port = htons(PORT); 
    server_address_.sin_addr.s_addr = INADDR_ANY; 
       
    int n;
    socklen_t len; 
       
    sendto(sockfd, (const char *)msg_to_server, strlen(msg_to_server), 
        MSG_CONFIRM, (const struct sockaddr *) &server_address_,  
            sizeof(server_address_)); 
    std::cout<<"Message to server sent."<<std::endl; 
           
    n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                MSG_WAITALL, (struct sockaddr *) &server_address_, 
                &len); 
    buffer[n] = '\0'; 
    std::cout<<"Server sent this message:"<<buffer<<std::endl; 
   
    close(sockfd); 
    return 0; 
}
