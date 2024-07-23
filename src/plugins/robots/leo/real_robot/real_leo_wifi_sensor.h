#ifndef REAL_LEO_WIFI_SENSOR_H
#define REAL_LEO_WIFI_SENSOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_sensor.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_device.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>

using namespace argos;

class CRealLeoWiFiSensor : public CCI_LeoWiFiSensor,
                           public CRealLeoDevice {

public:

   virtual ~CRealLeoWiFiSensor() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void Destroy();

   virtual void Do(Real f_elapsed_time) {}

   virtual void GetMessages(std::vector<CCI_LeoWiFiSensor::SMessage>& vec_messages);

   virtual void FlushMessages();

protected:

   ssize_t ReceiveDataMultiCast(unsigned char* pt_buf, size_t un_size, struct sockaddr* pt_sender_addr);

protected:

   int m_nMulticastSocket;
   struct sockaddr_in m_tMulticastAddr;

   pthread_t m_tListeningThread;
   pthread_mutex_t m_tListeningMutex;

   std::vector<CCI_LeoWiFiSensor::SMessage> m_vecMsgQueue;

private:

   void* ListeningThread();

   static void* StartListeningThread(void* pt_context) {
      return reinterpret_cast<CRealLeoWiFiSensor*>(pt_context)->ListeningThread();
   }
   
};

#endif // REAL_LEO_WIFI_SENSOR_H
