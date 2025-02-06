#ifndef WIFI_EQUIPPED_ENTITY_H
#define WIFI_EQUIPPED_ENTITY_H

namespace argos {
   class CWiFiEquippedEntity;
   class CWiFiMedium;
   class CEmbodiedEntity;
   struct SAnchor;
}

#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/utility/datatypes/set.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/space/positional_indices/space_hash.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_sensor.h>

namespace argos {

   class CWiFiEquippedEntity : public CPositionalEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CWiFiEquippedEntity*> TVector;
      typedef CSet<CWiFiEquippedEntity*> TSet;

      

   public:

      CWiFiEquippedEntity(CComposableEntity* pc_parent);

      CWiFiEquippedEntity(CComposableEntity* pc_parent,
                         const std::string& str_id,
                         size_t un_msg_size,
                         Real f_range,
                         SAnchor& s_anchor,
                         CEmbodiedEntity& c_entity_body,
                         const CVector3& c_position = CVector3(),
                         const CQuaternion& c_orientation = CQuaternion());

      virtual void Init(TConfigurationNode& t_tree);

      virtual ~CWiFiEquippedEntity() {}

      virtual void Reset();

      virtual void Update();

      virtual void SetEnabled(bool b_enabled);

      inline CEmbodiedEntity& GetEntityBody() {
         return *m_pcEntityBody;
      }

    //   inline size_t GetMsgSize() const {
    //      return m_cData.Size();
    //   }

    //   inline CByteArray& GetData() {
    //      return m_cData;
    //   }

      void RetrieveData(std::vector<CCI_LeoWiFiSensor::SMessage>& vecMsgQueue);

      void AppendData(CCI_LeoWiFiSensor::SMessage& robot);

      void ClearData();

      inline Real GetRange() const {
         return m_fRange;
      }

      inline void SetRange(Real f_range) {
         m_fRange = f_range;
      }

      inline const SAnchor& GetAnchor() const {
         return *m_psAnchor;
      }

      virtual std::string GetTypeDescription() const {
         return "wifi";
      }

      inline CWiFiMedium& GetMedium() {
         return *m_pcMedium;
      }

      inline void SetMedium(CWiFiMedium& c_medium) {
         m_pcMedium = &c_medium;
      }

   protected:

      SAnchor* m_psAnchor;
      CVector3 m_cPosOffset;
      CQuaternion m_cRotOffset;
      std::vector<CCI_LeoWiFiSensor::SMessage> m_vecMsgQueue;
      Real m_fRange;
      CEmbodiedEntity* m_pcEntityBody;
      CWiFiMedium* m_pcMedium;

   };

   /****************************************/
   /****************************************/

   class CWiFiEquippedEntitySpaceHashUpdater : public CSpaceHashUpdater<CWiFiEquippedEntity> {

   public:

      virtual void operator()(CAbstractSpaceHash<CWiFiEquippedEntity>& c_space_hash,
                              CWiFiEquippedEntity& c_element);

   private:

      SInt32 m_nCenterI, m_nCenterJ, m_nCenterK;

   };

   /****************************************/
   /****************************************/

   class CWiFiEquippedEntityGridCellUpdater : public CGrid<CWiFiEquippedEntity>::CCellOperation {

   public:

      CWiFiEquippedEntityGridCellUpdater(CGrid<CWiFiEquippedEntity>& c_grid);

      virtual bool operator()(SInt32 n_i,
                              SInt32 n_j,
                              SInt32 n_k,
                              CGrid<CWiFiEquippedEntity>::SCell& s_cell);

      void SetEntity(CWiFiEquippedEntity& c_entity);

   private:

      CGrid<CWiFiEquippedEntity>& m_cGrid;
      CWiFiEquippedEntity* m_pcEntity;
   };

   class CWiFiEquippedEntityGridEntityUpdater : public CGrid<CWiFiEquippedEntity>::COperation {

   public:

      CWiFiEquippedEntityGridEntityUpdater(CGrid<CWiFiEquippedEntity>& c_grid);
      virtual bool operator()(CWiFiEquippedEntity& c_entity);

   private:

      CGrid<CWiFiEquippedEntity>& m_cGrid;
      CWiFiEquippedEntityGridCellUpdater m_cCellUpdater;
   };

   /****************************************/
   /****************************************/

}

#endif
