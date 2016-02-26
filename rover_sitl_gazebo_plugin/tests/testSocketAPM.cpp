#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>

#include <stdio.h>
#include <string.h>

int fd;

ssize_t recv(void *buf, size_t size, uint32_t timeout_ms)
{
	fd_set fds;
	struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;
    if (select(fd+1, &fds, NULL, NULL, &tv) != 1) {
        return -1;
    }
    return ::recv(fd, buf, size, 0);
}

int main () { 

	//Initialize
	fd = socket(AF_INET, SOCK_DGRAM, 0);
	fcntl(fd, F_SETFD, FD_CLOEXEC);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    //Connect (+make_sockaddr)
    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(9003);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (::connect(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
        printf("Connection failed\n");
    }
    printf("Connection successful\n");

    //Set blocking false
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    //reuse address
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));


    //Receive APM input
    bool receive = false;
    struct servo_packet {
        float servos[16];    // ranges from 0 (no rotation) to 1 (full throttle)
    };
    servo_packet pkt;
    int szRecv;
    ssize_t st;

    while(!receive){
	    st = recv(&pkt, sizeof(pkt), 100);
	    printf("%zd\n", st);
	    receive=(st == sizeof(servo_packet));
    }
    printf("Receive!\n");

}

