#include <stdio.h>

#include "morse.h"
#include "socket.h"

int main() {
    
    // Set the address and the port for the socket communication
    char address[] = "127.0.0.1";
    int serverPort = 4000;

    // Set the name of the robot
    char parent[] = "robot";

    // Open the communication (here all the components use the same port)
    int sock;
    sock = client_doSock(address, serverPort);
    if ( sock == -1) {
        printf("Error 1\n");
        return 1;
    }
    FILE * f = fdopen(sock, "r+");

    // Set the robot linear and angular speed
    int flag = setSpeed(f, parent, "motion", 1, 0);
    if ( flag < 0 ) {
        printf("Error 2\n");
        fclose(f);
        return 2;
    }

    // Get the pose measurement
    Pose pose;
    flag = getPose(f, parent, "pose", &pose);
    if ( flag < 0 ) {
        printf("Error 3\n");
        fclose(f);
        return 2;
    }

    // Get the proximity measurement
    proxMeas proxmeas;
    flag = getProx(f, parent, "prox", &proxmeas);
    if ( flag < 0 ) {
        printf("Error 5\n");
        fclose(f);
        return 3;
    }

    // Get the ir #1 measurement
    irMeas irmeas1;
    flag = getIR(f, parent, "IR1", &irmeas1);
    if ( flag < 0 ) {
        printf("Error 4\n");
        fclose(f);
        return 3;
    }
    
    // Get the ir #2 measurement
    irMeas irmeas2;
    flag = getIR(f, parent, "IR2", &irmeas2);
    if ( flag < 0 ) {
        printf("Error 4\n");
        fclose(f);
        return 3;
    }

    // Get the ir #3 measurement
    irMeas irmeas3;
    flag = getIR(f, parent, "IR3", &irmeas3);
    if ( flag < 0 ) {
        printf("Error 4\n");
        fclose(f);
        return 3;
    }
    
    // Get the ir #4 measurement
    irMeas irmeas4;
    flag = getIR(f, parent, "IR4", &irmeas4);
    if ( flag < 0 ) {
        printf("Error 4\n");
        fclose(f);
        return 3;
    } 
    
    // Close the socket
    fclose(f);

    return 0;
}
