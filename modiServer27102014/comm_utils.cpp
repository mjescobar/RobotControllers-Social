#include "comm_utils.h"

DWORD getTimeInMs(void)
{
	struct timeval tv;
	DWORD result=0;
	if (gettimeofday(&tv,NULL)==0)
		result=(tv.tv_sec*1000+tv.tv_usec/1000)&0x03ffffff;
	return(result);
}
DWORD getTimeDiffInMs(DWORD lastTime)
{
	DWORD currentTime=getTimeInMs();
	if (currentTime<lastTime)
		return(currentTime+0x03ffffff-lastTime);
	return(currentTime-lastTime);
}

void reusePort(int s)
{
  int one=1;

  if ( setsockopt(s,SOL_SOCKET,SO_REUSEADDR,(char *) &one,sizeof(one)) == -1 )
  {
    printf("error in setsockopt,SO_REUSEPORT \n");
    exit(-1);
  }
}

int simpleReuseSocket(int port)
{
	unsigned int len = sizeof(struct sockaddr_in);;
    struct sockaddr_in name;
    int s = socket(AF_INET, SOCK_STREAM, 0);
	reusePort(s);
    name.sin_family = AF_INET;
    name.sin_port = htons(port);
    name.sin_addr.s_addr = htonl(INADDR_ANY); /* Use the wildcard address.*/
	if( bind(s, (struct sockaddr *) &name, len))	printf("bind modi socket error");
	listen(s, MAXCONNECTIONS);
   	printf("\tport %d open and listening\n",port);
   	return s;
}
int simpleAcceptConn(int s)
{
    unsigned int len = sizeof(struct sockaddr_in);
    struct sockaddr_in saddr;
    return accept(s, (struct sockaddr *) &saddr, &len);
}

int _receiveSimplePacket(std::vector<char>& packet, int acc_conn)
{
    //1. Read the header and packet size:
    char headerAndSize[HEADER_LENGTH];
    int totalReceived=0;
    DWORD startT=getTimeInMs();
    while(totalReceived!=HEADER_LENGTH)
    {
        int nb=recv(acc_conn,headerAndSize+totalReceived,HEADER_LENGTH-totalReceived,0);
        if (nb<1)
            break;
        totalReceived+=nb;
        if (getTimeDiffInMs(startT)>3000)
            break;
    }
    // 2. Check if the header is consistent:
    if (totalReceived!=HEADER_LENGTH)
        return(-2); // Error reading
    if ( (headerAndSize[0]!=(char)HEADER_ID1)||(headerAndSize[1]!=(char)HEADER_ID2) )
        return(-2); // Error, wrong header
    WORD dataLength=((WORD*)(headerAndSize+2))[0];
    // 3. Read the data with correct length:
    packet.clear();
    packet.resize(dataLength,0);
    totalReceived=0;
    startT=getTimeInMs();
    while(totalReceived!=dataLength)
    {
        int nb=recv(acc_conn,&packet[0]+totalReceived,dataLength-totalReceived,0);
        if (nb<1)
            break;
        totalReceived+=nb;
        if (getTimeDiffInMs(startT)>3000)
            break;
    }
    if (totalReceived!=dataLength)
        return(-2); // wrong size or nothing received
    return(int(((WORD*)(headerAndSize+2))[1]));
}

bool _sendSimplePacket(char* packet,int packetLength,WORD packetsLeft,int comm_addr)
{
	if (packetLength==0)
		return(false);

	// Insert the header:
	WORD s=WORD(packetLength);
	char header[HEADER_LENGTH];
	header[0]=HEADER_ID1;
	header[1]=HEADER_ID2;
	((WORD*)(header+2))[0]=s;
	((WORD*)(header+2))[1]=packetsLeft;

	std::vector<char> toSend;
	for (int i=0;i<HEADER_LENGTH;i++)
		toSend.push_back(header[i]);
	for (int i=0;i<packetLength;i++)
		toSend.push_back(packet[i]);
	// Send the packet:
	return(send(comm_addr,&toSend[0],packetLength+HEADER_LENGTH,0)==packetLength+HEADER_LENGTH);
}

bool replyToReceivedData(char* data,int dataSize, int comm_addr)
{
	if (dataSize==0)
		return(false);

	// In Following we make sure we don't send too big packets (we might send the data in several packets)
	int packetCount=0;
	int s=dataSize;
	while (s!=0)
	{
		packetCount++;
		if (s>MAX_PACKETSIZE-HEADER_LENGTH)
			s-=MAX_PACKETSIZE-HEADER_LENGTH;
		else
			s=0;
	}
	s=dataSize;
	int ptr=0;
	while (s!=0)
	{
		packetCount--;
		int sizeToSend=s;
		if (s>MAX_PACKETSIZE-HEADER_LENGTH)
			sizeToSend=MAX_PACKETSIZE-HEADER_LENGTH;
		s-=sizeToSend;
		if (!_sendSimplePacket(data+ptr,sizeToSend,packetCount,comm_addr))
			return(false);
		ptr+=sizeToSend;
	}
	return(true);
}

char* receiveData(int& dataSize,int acc_conn)
{ // Returns the data size if >0, 0=we had a read time out, -1=we have an error
	std::vector<char> receivedData;
	while (true)
	{
		std::vector<char> inDat;
		int result=_receiveSimplePacket(inDat,acc_conn);
		if (result<0)
		{
			dataSize=result+1; // error or read time-out
			return(NULL);
		}
		receivedData.insert(receivedData.end(),inDat.begin(),inDat.end());
		if (result==0)
		{ // success
			dataSize=int(receivedData.size());
			char* retBuff=new char[dataSize];
			for (int i=0;i<dataSize;i++)
				retBuff[i]=receivedData[i];
			return(retBuff);
		}
	}
}
