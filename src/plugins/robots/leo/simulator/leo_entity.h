/**
 * @file <argos3/plugins/robots/leo/simulator/leo_entity.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef LEO_ENTITY_H
#define LEO_ENTITY_H

namespace argos {
   class CBatteryEquippedEntity;
   class CControllableEntity;
   class CEmbodiedEntity;
   class CWiFiEquippedEntity;
   class CProximitySensorEquippedEntity;
   class CLeoEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CLeoEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CLeoEntity();

      CLeoEntity(const std::string& str_id,
                 const std::string& str_controller_id,
                 const CVector3& c_position = CVector3(),
                 const CQuaternion& c_orientation = CQuaternion(),
                 Real f_rab_range = 3.0f,
                 size_t un_wifi_data_size = 10,
                 const std::string& str_bat_model = "");
      
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();
      
      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CBatteryEquippedEntity& GetBatterySensorEquippedEntity() {
          return *m_pcBatteryEquippedEntity;
      }

      inline CWiFiEquippedEntity& GetRABEquippedEntity() {
         return *m_pcWiFiEquippedEntity;
      }

      inline CProximitySensorEquippedEntity& GetProximitySensorEquippedEntity() {
         return *m_pcProximitySensorEquippedEntity;
      }

      inline Real GetLinearVelocity() const {
         return m_fLinearVelocity;
      }

      inline void SetLinearVelocity(Real f_velocity) {
         m_fLinearVelocity = f_velocity;
      }

      inline CRadians GetAngularVelocity() const {
         return m_cAngularVelocity;
      }

      inline void SetAngularVelocity(CRadians c_velocity) {
         m_cAngularVelocity = c_velocity;
      }

      virtual std::string GetTypeDescription() const {
         return "leo";
      }

   private:

      CControllableEntity*            m_pcControllableEntity;
      CEmbodiedEntity*                m_pcEmbodiedEntity;
      CBatteryEquippedEntity*         m_pcBatteryEquippedEntity;
      CWiFiEquippedEntity*             m_pcWiFiEquippedEntity;
      CProximitySensorEquippedEntity* m_pcProximitySensorEquippedEntity;
      Real     m_fLinearVelocity;
      CRadians m_cAngularVelocity;
   };

}

#endif
