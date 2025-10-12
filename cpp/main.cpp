#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <string>
#include <sys/socket.h>
#include <vector>

#define MAXBUF 1024

int main() {
  int sockfd, len;
  unsigned char buf[MAXBUF];
  struct sockaddr_in udp_server_addr;
  struct sockaddr_in udp_client_addr;
  struct sockaddr_storage server_storage;
  socklen_t addr_size, client_addr_size;

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  if (sockfd < 0) {
    std::cout << "socket error!\n" << std::endl;
    return 0;
  }
  udp_server_addr.sin_family = AF_INET;
  udp_server_addr.sin_port = htons(8001);
  udp_server_addr.sin_addr.s_addr = inet_addr("0.0.0.0");
  memset(udp_server_addr.sin_zero, '\0', sizeof(udp_server_addr.sin_zero));
  bind(sockfd, (struct sockaddr *)&udp_server_addr, sizeof(udp_server_addr));
  addr_size = sizeof(server_storage);
  while (true) {
    len = recvfrom(sockfd, buf, MAXBUF, 0, (struct sockaddr *)&server_storage,
                   &addr_size);
    std::cout << "Received from server: " << len << "bytes" << std::endl;
    for (int i = 0; i < len; i++) {
      std::cout << buf[i] << std::endl;
    }
    std::cout << "\n" << std::endl;
  }
  return 0;
}
