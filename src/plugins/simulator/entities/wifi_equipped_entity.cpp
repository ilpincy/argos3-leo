#include "wifi_equipped_entity.h"
#include <argos3/core/utility/string_utilities.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/media/wifi_medium.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_sensor.h>

namespace argos {

   /****************************************/
   /****************************************/

   CWiFiEquippedEntity::CWiFiEquippedEntity(CComposableEntity* pc_parent) :
      CPositionalEntity(pc_parent),
      m_psAnchor(nullptr),
      m_fRange(0.0f),
      m_pcEntityBody(nullptr),
      m_pcMedium(nullptr) {
      Disable();
   }

   /****************************************/
   /****************************************/

   CWiFiEquippedEntity::CWiFiEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id,
                                          size_t un_msg_size,
                                          Real f_range,
                                          SAnchor& s_anchor,
                                          CEmbodiedEntity& c_entity_body,
                                          const CVector3& c_pos_offset,
                                          const CQuaternion& c_rot_offset) :
      CPositionalEntity(pc_parent,
                        str_id),
      m_psAnchor(&s_anchor),
      m_cPosOffset(c_pos_offset),
      m_cRotOffset(c_rot_offset),
      m_fRange(f_range),
      m_pcEntityBody(&c_entity_body),
      m_pcMedium(nullptr) {
      Disable();
      CVector3 cPos = c_pos_offset;
      cPos.Rotate(s_anchor.Orientation);
      cPos += s_anchor.Position;
      SetInitPosition(cPos);
      SetPosition(cPos);
      SetInitOrientation(s_anchor.Orientation * c_rot_offset);
      SetOrientation(GetInitOrientation());
   }

   /****************************************/
   /****************************************/

   void CWiFiEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /*
          * Init entity.
          * Here we explicitly avoid to call CPositionalEntity::Init() because that
          * would also initialize position and orientation, which, instead, must
          * be calculated from reference entity and offsets.
          */
         CEntity::Init(t_tree);
         /* Get offsets */
         GetNodeAttributeOrDefault(t_tree, "pos_offset", m_cPosOffset, m_cPosOffset);
         std::string strRotOffset;
         GetNodeAttributeOrDefault(t_tree, "rot_offset", strRotOffset, strRotOffset);
         if(strRotOffset != "") {
            CDegrees cRotOffsetEuler[3];
            ParseValues(strRotOffset, 3, cRotOffsetEuler, ',');
            m_cRotOffset.FromEulerAngles(ToRadians(cRotOffsetEuler[0]),
                                         ToRadians(cRotOffsetEuler[1]),
                                         ToRadians(cRotOffsetEuler[2]));
         }
         /* Parse and look up the anchor */
         std::string strAnchorId;
         GetNodeAttribute(t_tree, "anchor", strAnchorId);
         /*
          * NOTE: here we get a reference to the embodied entity
          * This line works under the assumption that:
          * 1. the WiFiEquippedEntity has a parent;
          * 2. the parent has a child whose id is "body"
          * 3. the "body" is an embodied entity
          * If any of the above is false, this line will bomb out.
          */
         m_pcEntityBody = &GetParent().GetComponent<CEmbodiedEntity>("body");
         m_psAnchor = &m_pcEntityBody->GetAnchor(strAnchorId);
         /* Get transmission range */
         GetNodeAttribute(t_tree, "range", m_fRange);
         /* Set init position and orientation */
         Update();
         SetInitPosition(GetPosition());
         SetInitOrientation(GetOrientation());
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing a range and bearing entity \"" << GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CWiFiEquippedEntity::Update() {
      CVector3 cPos = m_cPosOffset;
      cPos.Rotate(m_psAnchor->Orientation);
      cPos += m_psAnchor->Position;
      SetPosition(cPos);
      SetOrientation(m_psAnchor->Orientation * m_cRotOffset);
   }

   /****************************************/
   /****************************************/

   void CWiFiEquippedEntity::Reset() {
   }

   /****************************************/
   /****************************************/

   void CWiFiEquippedEntity::SetEnabled(bool b_enabled) {
      /* Perform generic enable behavior */
      CEntity::SetEnabled(b_enabled);
      /* Perform specific enable behavior */
      if(b_enabled) {
         /* Enable body anchor */
         if(m_psAnchor)
            m_psAnchor->Enable();
         /* Enable entity in medium */
         if(m_pcMedium && GetIndex() >= 0)
            m_pcMedium->AddEntity(*this);
      }
      else {
         /* Disable body anchor */
         if(m_psAnchor)
            m_psAnchor->Disable();
         /* Disable entity in medium */
         if(m_pcMedium)
            m_pcMedium->RemoveEntity(*this);
      }
   }

   /****************************************/
   /****************************************/

   void CWiFiEquippedEntity::RetrieveData(std::vector<CCI_LeoWiFiSensor::SMessage>& vecMsgQueue) {
    //   std::cout << __FILE__ << " " << __func__ << std::endl;
      for(auto& item : m_vecMsgQueue) {
         vecMsgQueue.push_back(item);
      }
   }

   /****************************************/
   /****************************************/

   void CWiFiEquippedEntity::AppendData(CCI_LeoWiFiSensor::SMessage& msg) {
    // std::cout << __FILE__ << " " << __func__ << std::endl;
      m_vecMsgQueue.push_back(msg);

   }

   /****************************************/
   /****************************************/

   void CWiFiEquippedEntity::ClearData() {
      m_vecMsgQueue.clear();
   }

   /****************************************/
   /****************************************/

   void CWiFiEquippedEntitySpaceHashUpdater::operator()(CAbstractSpaceHash<CWiFiEquippedEntity>& c_space_hash,
                                                       CWiFiEquippedEntity& c_element) {
      /* Calculate the position of the center of the WiFi equipped entity in the space hash */
      c_space_hash.SpaceToHashTable(m_nCenterI,
                                    m_nCenterJ,
                                    m_nCenterK,
                                    c_element.GetPosition());
      /* Update the cells in a sphere around it */
      SInt32 nRangeI = c_space_hash.SpaceToHashTable(c_element.GetRange(), 0);
      SInt32 nRangeJ;
      SInt32 nRangeK;
      for(SInt32 i = 0; i <= nRangeI; ++i) {
         nRangeJ =
            c_space_hash.SpaceToHashTable(
               ::sqrt(
                  Square(c_element.GetRange()) -
                  Square(c_space_hash.HashTableToSpace(i, 0))
                  ),
               1);
         for(SInt32 j = 0; j <= nRangeJ; ++j) {
            nRangeK =
               c_space_hash.SpaceToHashTable(
                  ::sqrt(
                     Square(c_element.GetRange()) -
                     Square(c_space_hash.HashTableToSpace(j, 1))
                     ),
                  2);
            for(SInt32 k = 0; k <= nRangeK; ++k) {
               if(i > 0) {
                  /*
                   * i > 0
                   */
                  if(j > 0) {
                     /*
                      * i > 0
                      * j > 0
                      */
                     if(k > 0) {
                        /*
                         * i > 0
                         * j > 0
                         * k > 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ + j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ + j, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ - j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ - j, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ + j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ + j, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ - j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ - j, m_nCenterK - k, c_element);
                     }
                     else {
                        /*
                         * i > 0
                         * j > 0
                         * k == 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ + j, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ - j, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ + j, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ - j, m_nCenterK, c_element);
                     }
                  }
                  else {
                     /*
                      * i > 0
                      * j == 0
                      */
                     if(k > 0) {
                        /*
                         * i > 0
                         * j == 0
                         * k > 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ, m_nCenterK - k, c_element);
                     }
                     else {
                        /*
                         * i > 0
                         * j == 0
                         * k == 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ, m_nCenterK, c_element);
                     }
                  }
               }
               else {
                  /*
                   * i == 0
                   */
                  if(j > 0) {
                     /*
                      * i == 0
                      * j > 0
                      */
                     if(k > 0) {
                        /*
                         * i == 0
                         * j > 0
                         * k > 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ + j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ + j, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ - j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ - j, m_nCenterK - k, c_element);
                     }
                     else {
                        /*
                         * i == 0
                         * j > 0
                         * k == 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ + j, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ - j, m_nCenterK, c_element);
                     }
                  }
                  else {                     
                     /*
                      * i == 0
                      * j == 0
                      */
                     if(k > 0) {
                        /*
                         * i == 0
                         * j == 0
                         * k > 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ, m_nCenterK - k, c_element);
                     }
                     else {
                        /*
                         * i == 0
                         * j == 0
                         * k == 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ, m_nCenterK, c_element);
                     }
                  }
               }
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   class CSpaceOperationAddCWiFiEquippedEntity : public CSpaceOperationAddEntity {
   public:
      void ApplyTo(CSpace& c_space, CWiFiEquippedEntity& c_entity) {
         /* Add entity to space - this ensures that the WiFi entity
          * gets an id before being added to the WiFi medium */
         c_space.AddEntity(c_entity);
         /* Enable the WiFi entity, if it's enabled - this ensures that
          * the entity gets added to the WiFi if it's enabled */
         c_entity.SetEnabled(c_entity.IsEnabled());
      }
   };

   class CSpaceOperationRemoveCWiFiEquippedEntity : public CSpaceOperationRemoveEntity {
   public:
      void ApplyTo(CSpace& c_space, CWiFiEquippedEntity& c_entity) {
         /* Disable the entity - this ensures that the entity is
          * removed from the WiFi medium */
         c_entity.Disable();
         /* Remove the WiFi entity from space */
         c_space.RemoveEntity(c_entity);
      }
   };

   REGISTER_SPACE_OPERATION(CSpaceOperationAddEntity, CSpaceOperationAddCWiFiEquippedEntity, CWiFiEquippedEntity);
   REGISTER_SPACE_OPERATION(CSpaceOperationRemoveEntity, CSpaceOperationRemoveCWiFiEquippedEntity, CWiFiEquippedEntity);

   /****************************************/
   /****************************************/

   CWiFiEquippedEntityGridCellUpdater::CWiFiEquippedEntityGridCellUpdater(CGrid<CWiFiEquippedEntity>& c_grid) :
      m_cGrid(c_grid) {}
   
   bool CWiFiEquippedEntityGridCellUpdater::operator()(SInt32 n_i,
                                                      SInt32 n_j,
                                                      SInt32 n_k,
                                                      CGrid<CWiFiEquippedEntity>::SCell& s_cell) {
      /* Update cell */
      m_cGrid.UpdateCell(n_i, n_j, n_k, *m_pcEntity);
      /* Continue with other cells */
      return true;
   }
   
   void CWiFiEquippedEntityGridCellUpdater::SetEntity(CWiFiEquippedEntity& c_entity) {
      m_pcEntity = &c_entity;
   }

   CWiFiEquippedEntityGridEntityUpdater::CWiFiEquippedEntityGridEntityUpdater(CGrid<CWiFiEquippedEntity>& c_grid) :
      m_cGrid(c_grid),
      m_cCellUpdater(c_grid) {}

   bool CWiFiEquippedEntityGridEntityUpdater::operator()(CWiFiEquippedEntity& c_entity) {
      try {
         m_cCellUpdater.SetEntity(c_entity);
         m_cGrid.ForCellsInBoxRange(c_entity.GetPosition(),
                                    CVector3(c_entity.GetRange(),
                                             c_entity.GetRange(),
                                             c_entity.GetRange()),
                                    m_cCellUpdater);
         /* Continue with the other entities */
         return true;
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While updating the WiFi entity grid for WiFi entity \"" << c_entity.GetContext() << c_entity.GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

}
