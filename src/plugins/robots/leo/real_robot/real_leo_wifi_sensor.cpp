#include "real_leo_wifi_sensor.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <unistd.h>
#include <fcntl.h>
#include <ifaddrs.h>

/****************************************/
/****************************************/

void CRealLeoWiFiSensor::Init(TConfigurationNode& t_node) {
   DEBUG_FUNCTION_ENTER;
   /* Just to test */
   struct ifaddrs* ptIFAddrs;
   int nRes2 = getifaddrs(&ptIFAddrs);
   if(nRes2 < 0) {
      THROW_ARGOSEXCEPTION("getifaddrs() in wifi sensor failed: " << strerror(errno));
   }
   for(struct ifaddrs* i = ptIFAddrs; i; i = i->ifa_next) {
      DEBUG("%s: %s %s\n",
            i->ifa_name,
            inet_ntoa(reinterpret_cast<sockaddr_in*>(i->ifa_addr)->sin_addr),
            inet_ntoa(reinterpret_cast<sockaddr_in*>(i->ifa_netmask)->sin_addr));
   }
   freeifaddrs(ptIFAddrs);
   /* Parse XML configuration for multicast */
   std::string strInterfaceAddr;
   GetNodeAttribute(t_node, "ip_address", strInterfaceAddr);
   // TODO
   std::string strMulticastAddr;
   GetNodeAttribute(t_node, "multicast_address", strMulticastAddr);
   // TODO
   uint16_t nMulticastPort;
   GetNodeAttribute(t_node, "multicast_port", nMulticastPort);
   /* Setup UDP socket */
   m_nMulticastSocket = ::socket(AF_INET, SOCK_DGRAM, 0);
   fcntl(m_nMulticastSocket, F_SETFL, O_NONBLOCK);
   memset(&m_tMulticastAddr, 0, sizeof(m_tMulticastAddr));
   m_tMulticastAddr.sin_family = AF_INET;
   m_tMulticastAddr.sin_port = htons(nMulticastPort);
   m_tMulticastAddr.sin_addr.s_addr = htonl(INADDR_ANY);
   int nRes = bind(m_nMulticastSocket,
                   reinterpret_cast<sockaddr*>(&m_tMulticastAddr),
                   sizeof(m_tMulticastAddr));
   if(nRes < 0) {
      DEBUG_FUNCTION_EXIT;
      THROW_ARGOSEXCEPTION("bind() in wifi sensor failed: " << strerror(errno));
   }
   struct ip_mreq tIPMReq;
   tIPMReq.imr_multiaddr.s_addr = inet_addr(strMulticastAddr.c_str());
   tIPMReq.imr_interface.s_addr = inet_addr(strInterfaceAddr.c_str());
   /* Launch listening thread */
   nRes = pthread_create(&m_tListeningThread,
                         nullptr,
                         &CRealLeoWiFiSensor::StartListeningThread,
                         this);
   if(nRes < 0) {
      DEBUG_FUNCTION_EXIT;
      THROW_ARGOSEXCEPTION("pthread_create() in wifi sensor failed: " << strerror(errno));
   }
   m_tListeningMutex = PTHREAD_MUTEX_INITIALIZER;
   DEBUG_FUNCTION_EXIT;
}

/****************************************/
/****************************************/

void CRealLeoWiFiSensor::Destroy() {
   DEBUG_FUNCTION_ENTER;
   /* Close the listening socket
    * This also forced recvfrom() to fail, and then a call to pthread_exit()
    * follows
    */
   close(m_nMulticastSocket);
   /* Wait for listening thread to finish */
   pthread_join(m_tListeningThread, nullptr);
   LOG << "WiFi Listening thread stopped" << std::endl;
   DEBUG_FUNCTION_EXIT;
}

/****************************************/
/****************************************/

void CRealLeoWiFiSensor::GetMessages(std::vector<CCI_LeoWiFiSensor::SMessage>& vec_messages) {
   DEBUG_FUNCTION_ENTER;
   pthread_mutex_lock(&m_tListeningMutex);
   vec_messages.swap(m_vecMsgQueue);
   m_vecMsgQueue.clear();
   pthread_mutex_unlock(&m_tListeningMutex);
   DEBUG_FUNCTION_EXIT;
}

/****************************************/
/****************************************/

void CRealLeoWiFiSensor::FlushMessages() {
   DEBUG_FUNCTION_ENTER;
   pthread_mutex_lock(&m_tListeningMutex);
   m_vecMsgQueue.clear();
   pthread_mutex_unlock(&m_tListeningMutex);
   DEBUG_FUNCTION_EXIT;
}

/****************************************/
/****************************************/

void* CRealLeoWiFiSensor::ListeningThread() {
   DEBUG_FUNCTION_ENTER;
   LOG << "WiFi Listening thread started" << std::endl;
   DEBUG("WiFi Listening thread started\n");
   /* Message buffer */
   UInt8 punBuffer[1024];
   /* Sender address */
   struct sockaddr tSenderAddr;
   socklen_t tSenderAddrLen = sizeof(tSenderAddr);
   /* Set up non-blocking listen that polls every 10 milliseconds */
   fd_set tSocket;
   struct timeval tv;
   tv.tv_sec = 0;
   tv.tv_usec = 10000;
   while(1) {
      /* Poll message */
      FD_ZERO(&tSocket);
      FD_SET(m_nMulticastSocket, &tSocket);
      DEBUG("Calling select()...\n");
      int nReady = select(m_nMulticastSocket + 1, &tSocket, nullptr, nullptr, &tv);
      if(nReady > 0) {
         /* Receive message */
         DEBUG("Calling recvfrom()...\n");
         ssize_t nRecvd;
         do {
            nRecvd = recvfrom(m_nMulticastSocket,
                              punBuffer,
                              sizeof(punBuffer),
                              0, // flags
                              &tSenderAddr,
                              &tSenderAddrLen);
            DEBUG("recvfrom() received %zd bytes\n", nRecvd);
            /* In case of error, it's fine to exit the thread */
            if(nRecvd < 0) {
               DEBUG("nRecvd < 0\n");
               DEBUG_FUNCTION_EXIT;
               pthread_exit(nullptr);
            }
         } while(nRecvd > 0);
         /* Add message to queue */
         pthread_mutex_lock(&m_tListeningMutex);
         m_vecMsgQueue.push_back({
               inet_ntoa(reinterpret_cast<sockaddr_in*>(&tSenderAddr)->sin_addr),
               CByteArray(punBuffer, nRecvd)
            });
         pthread_mutex_unlock(&m_tListeningMutex);
      }
      else if(nReady < 0) {
         /* In case of error, it's fine to exit the thread */
         DEBUG("nReady < 0\n");
         DEBUG_FUNCTION_EXIT;
         pthread_exit(nullptr);
      }
   }
   DEBUG_FUNCTION_EXIT;
   return nullptr;
}

/****************************************/
/****************************************/
