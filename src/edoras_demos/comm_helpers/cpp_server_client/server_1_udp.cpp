#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define OWN_PORT 8080
#define MAXLINE 1024
#define OTHER_PORT 9090
int main() {

  int sockfd;
  char buffer[MAXLINE];
  struct sockaddr_in own_address_;
  struct sockaddr_in other_address_;
  
  // Create socket
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sockfd < 0)
  {
   perror("Socket creationg failed \n");
   exit(EXIT_FAILURE);
  }
  
  memset(&own_address_, 0, sizeof(own_address_));
  memset(&other_address_, 0, sizeof(other_address_));
  
  // Fill server information
  own_address_.sin_family = AF_INET;
  own_address_.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
  own_address_.sin_port = htons(OWN_PORT);
  
  // Bind the socket
  int res = bind(sockfd, (const struct sockaddr*)&own_address_, sizeof(own_address_));
  if( res < 0 )
  {
     perror("Bind failed");
     exit(EXIT_FAILURE);
  }
  
  socklen_t len;
  int n;

  other_address_.sin_family = AF_INET;
  other_address_.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
  other_address_.sin_port = htons(OTHER_PORT);

  int N = 100;
  for(int i = 0; i < N; ++i)
  {
    char temp[50];
    sprintf(temp, "Grenoble, France %d", i); 
    const char* msg_to_client = temp;

    sendto(sockfd, (const char*)msg_to_client, strlen(msg_to_client), 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));
    
    n = recvfrom(sockfd, (char*) buffer, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(n > 0)
    {
      buffer[n] = '\0';
      printf("SERVER 1: Received this message from server_2: %s \n", buffer);
    }
    
    sleep(1);
  }
  
  return 0;   
}
