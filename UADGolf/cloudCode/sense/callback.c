
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <parse.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <errno.h>
#include <signal.h>
#include <setjmp.h>

#define TRY do{ jmp_buf ex_buf__; if( !setjmp(ex_buf__) ){
#define CATCH } else {
#define ETRY } }while(0)
#define THROW longjmp(ex_buf__, 1)
#define PORT_SENSE 50014
#define HOST "127.0.0.1"
#define MAXPENDING 5    /* Maximum outstanding connection requests */

char dataUUID[] = "bb946c14-758a-4448-8b78-69b04ba1bb8b"; /** Please update this value **/


/* This is a callback function (function pointer) that registers itself with the Parse app to receive push notifications
 *  from the cloud. Whenever an explicit change is made on the cloud by an end user from the web site, we will over ride any
 *  existing changes and update the bulb accordingly.
 */
void puttCallback(ParseClient client, int error, const char *buffer)
{

	if(error != 0 || buffer == NULL) {
		printf("Error on callback\n");
		return;
	}

  char *temp = strtok(buffer, "\"");
  if(temp == NULL) {
    printf("bad push value\n");
    return;
  }

  int i = 0;
  for(;i<5;i++) {
    temp = strtok(NULL, "\"");
  }

  clientSendSocket(PORT_SENSE, temp);
}

// This function pushes data onto the Parse Application in cloud.
void *pushNotifications()
{
   // pthread_detach(pthread_self());
    //ParseClient client = parseInitialize("LLXKP3xsmyHpEsZiYo6b8i9kHhsHDKyrlkW5lNrP", "D8XJySU9yqmTTLQkMDLEebVfKmLjp1ApNtWuFyxN");
	  ParseClient client = parseInitialize("iAFEw9XwderX692l0DGIwoDDHcLTGMGtcBFbgMqb", "xKXmwOUoW31IlKBuoygAPADlXl01z1fsOtK27lcO");
    char *installationId = parseGetInstallationId(client);
    
    /* We need to set the InstallationId forcefully. Setting installationId to dataUUID based on null string is incorrect
     logic as there is a possibility that the installationId was previously set to some junk value.
     Typically this will break the push notification subscription */
    parseSetInstallationId(client, dataUUID);
    parseSetPushCallback(client, puttCallback);
    parseStartPushService(client);
    parseRunPushLoop(client);
}




// Here we establish socket connectivity with another endpoint/program on 'port' argument
void clientSendSocket(int port, char *buffer)
{
    int sockfd = 0;
    struct sockaddr_in servAddr;
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("clientSocket():ERROR : Could not create socket \n");
    }
    memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(port);
    servAddr.sin_addr.s_addr = inet_addr(HOST);
    if (connect(sockfd, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0)
    {
        printf("clientSendSocket(): ERROR : Connect Failed on Port: %d\n", port);
    }
    else
    {
        send(sockfd, buffer, strlen(buffer), 0);
        printf("clientSendSocket(): Data sent Successfully on Port: %d\n", port);
    }
    close(sockfd);
}

void sigHandler(int sig) {
  exit(0);
}

int main() {
    printf("starting callback process\n");
    //pthread_create(&threadPush, NULL, threadPushNotifications, NULL);
    signal(SIGINT, &sigHandler);
    signal(SIGTERM, &sigHandler);
   
    TRY
   {

      pushNotifications();
   }
   CATCH
   {
      printf("Got Exception!\n");
   }
   ETRY;
    
    /*while(1)
    {	
        clientSendSocket(PORT_INTENSITY, str);
    }*/
    
    return 0;
}
