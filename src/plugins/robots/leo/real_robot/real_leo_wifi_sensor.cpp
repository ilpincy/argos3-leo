#include "real_leo_wifi_sensor.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <unistd.h>

/****************************************/
/****************************************/

void CRealLeoWiFiSensor::Init(TConfigurationNode& t_node) {
   /* Parse XML configuration for multicast */
   // TODO
   std::string strInterfaceAddr("192.168.1.243");
   // TODO
   std::string strMulticastAddr("224.0.0.10");
   // TODO
   uint16_t nMulticastPort = 8888;
   /* Setup UDP socket */
   m_nMulticastSocket = ::socket(AF_INET, SOCK_DGRAM, 0);
   memset(&m_tMulticastAddr, 0, sizeof(m_tMulticastAddr));
   m_tMulticastAddr.sin_family = AF_INET;
   m_tMulticastAddr.sin_port = htons(nMulticastPort);
   m_tMulticastAddr.sin_addr.s_addr = htonl(INADDR_ANY);
   int nRes = bind(m_nMulticastSocket,
                   reinterpret_cast<sockaddr*>(&m_tMulticastAddr),
                   sizeof(m_tMulticastAddr));
   if(nRes < 0) {
      THROW_ARGOSEXCEPTION("Can't bind listening port " << nMulticastPort << ": "<< ::strerror(nRes));
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
      THROW_ARGOSEXCEPTION("Can't run WiFi listening thread: " << ::strerror(nRes));
   }
   m_tListeningMutex = PTHREAD_MUTEX_INITIALIZER;
}

/****************************************/
/****************************************/

void CRealLeoWiFiSensor::Destroy() {
   /* Close the listening socket
    * This also forced recvfrom() to fail, and then a call to pthread_exit()
    * follows
    */
   close(m_nMulticastSocket);
   /* Wait for listening thread to finish */
   pthread_join(m_tListeningThread, nullptr);
   LOG << "WiFi Listening thread stopped" << std::endl;
}

/****************************************/
/****************************************/

void CRealLeoWiFiSensor::GetMessages(std::vector<CCI_LeoWiFiSensor::SMessage>& vec_messages) {
   pthread_mutex_lock(&m_tListeningMutex);
   vec_messages.swap(m_vecMsgQueue);
   m_vecMsgQueue.clear();
   pthread_mutex_unlock(&m_tListeningMutex);
}

/****************************************/
/****************************************/

void CRealLeoWiFiSensor::FlushMessages() {
   pthread_mutex_lock(&m_tListeningMutex);
   m_vecMsgQueue.clear();
   pthread_mutex_unlock(&m_tListeningMutex);
}

/****************************************/
/****************************************/

void* CRealLeoWiFiSensor::ListeningThread() {
   LOG << "WiFi Listening thread started" << std::endl;
   while(1) {
      /* Receive message */
      ssize_t nRecvd = recvfrom();
      /* In case of error, it's fine to exit the thread */
      if(nRecvd < 0)
         pthread_exit(nullptr);
      /* Add message to queue */
      pthread_mutex_lock(&m_tListeningMutex);
      pthread_mutex_unlock(&m_tListeningMutex);
   }
   return nullptr;
}

/****************************************/
/****************************************/
