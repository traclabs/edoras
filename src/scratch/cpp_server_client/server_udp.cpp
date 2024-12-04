#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT 8080
#define MAXLINE 1024

int main() {

  int sockfd;
  char buffer[MAXLINE];
  const char *msg_to_client = "Grenoble, France";
  struct sockaddr_in server_address_;
  struct sockaddr_in client_address_;
  
  // Create socket
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sockfd < 0)
  {
   perror("Socket creationg failed \n");
   exit(EXIT_FAILURE);
  }
  
  memset(&server_address_, 0, sizeof(server_address_));
  memset(&client_address_, 0, sizeof(client_address_));
  
  // Fill server information
  server_address_.sin_family = AF_INET;
  server_address_.sin_addr.s_addr = INADDR_ANY;
  server_address_.sin_port = htons(PORT);
  
  // Bind the socket
  int res = bind(sockfd, (const struct sockaddr*)&server_address_, sizeof(server_address_));
  if( res < 0 )
  {
     perror("Bind failed");
     exit(EXIT_FAILURE);
  }
  
  socklen_t len;
  int n;
  
  len = sizeof(client_address_);
  n = recvfrom(sockfd, (char*) buffer, MAXLINE, MSG_WAITALL, (struct sockaddr*) &client_address_, &len);
  buffer[n] = '\0';
  
  printf("Client sent this message: %s \n", buffer);
  
  printf("Client info: family: %d (AFINET: %d) \n", client_address_.sin_family, AF_INET);
  printf("Client info: s_addr: %lu (INADDR_ANY: %lu) \n", client_address_.sin_addr.s_addr, INADDR_ANY);
  printf("Client info: port: %lu (htons port: %lu) \n", client_address_.sin_port, server_address_.sin_port);

  
  sendto(sockfd, (const char*)msg_to_client, strlen(msg_to_client), MSG_CONFIRM, (const struct sockaddr *)&client_address_, len);
  
  printf("Server sent message back to client \n");
 
  return 0;   
}
