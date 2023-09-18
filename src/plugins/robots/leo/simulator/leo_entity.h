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

      virtual std::string GetTypeDescription() const {
         return "leo";
      }

   private:

      CControllableEntity*    m_pcControllableEntity;
      CEmbodiedEntity*        m_pcEmbodiedEntity;
      CBatteryEquippedEntity* m_pcBatteryEquippedEntity;
   };

}

#endif
