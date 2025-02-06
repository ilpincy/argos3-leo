#include "wifi_medium.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   /****************************************/
   /****************************************/

   CWiFiMedium::CWiFiMedium() :
      m_bCheckOcclusions(true) {
   }

   /****************************************/
   /****************************************/

   CWiFiMedium::~CWiFiMedium() {
   }

   /****************************************/
   /****************************************/

   void CWiFiMedium::Init(TConfigurationNode& t_tree) {
      try {
         CMedium::Init(t_tree);
         /* Check occlusions? */
         GetNodeAttributeOrDefault(t_tree, "check_occlusions", m_bCheckOcclusions, m_bCheckOcclusions);
         /* Get the positional index method */
         std::string strPosIndexMethod("grid");
         GetNodeAttributeOrDefault(t_tree, "index", strPosIndexMethod, strPosIndexMethod);
         /* Get the arena center and size */
         CVector3 cArenaCenter;
         CVector3 cArenaSize;
         TConfigurationNode& tArena = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "arena");
         GetNodeAttribute(tArena, "size", cArenaSize);
         GetNodeAttributeOrDefault(tArena, "center", cArenaCenter, cArenaCenter);
         /* Create the positional index for embodied entities */
         if(strPosIndexMethod == "grid") {
            size_t punGridSize[3];
            if(!NodeAttributeExists(t_tree, "grid_size")) {
               punGridSize[0] = static_cast<size_t>(cArenaSize.GetX());
               punGridSize[1] = static_cast<size_t>(cArenaSize.GetY());
               punGridSize[2] = static_cast<size_t>(cArenaSize.GetZ());
            }
            else {
               std::string strPosGridSize;
               GetNodeAttribute(t_tree, "grid_size", strPosGridSize);
               ParseValues<size_t>(strPosGridSize, 3, punGridSize, ',');
            }
            CGrid<CWiFiEquippedEntity>* pcGrid = new CGrid<CWiFiEquippedEntity>(
               cArenaCenter - cArenaSize * 0.5f, cArenaCenter + cArenaSize * 0.5f,
               punGridSize[0], punGridSize[1], punGridSize[2]);
            m_pcWiFiEquippedEntityGridUpdateOperation = new CWiFiEquippedEntityGridEntityUpdater(*pcGrid);
            pcGrid->SetUpdateEntityOperation(m_pcWiFiEquippedEntityGridUpdateOperation);
            m_pcWiFiEquippedEntityIndex = pcGrid;
         }
         else {
            THROW_ARGOSEXCEPTION("Unknown method \"" << strPosIndexMethod << "\" for the positional index.");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error in initialization of the range-and-bearing medium", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CWiFiMedium::PostSpaceInit() {
      Update();
   }

   /****************************************/
   /****************************************/

   void CWiFiMedium::Reset() {
      /* Reset positional index of WiFi entities */
      m_pcWiFiEquippedEntityIndex->Reset();
      /* Delete routing table */
      for(TRoutingTable::iterator it = m_tRoutingTable.begin();
          it != m_tRoutingTable.end();
          ++it) {
         it->second.clear();
      }
   }

   /****************************************/
   /****************************************/

   void CWiFiMedium::Destroy() {
      delete m_pcWiFiEquippedEntityIndex;
      if(m_pcWiFiEquippedEntityGridUpdateOperation != nullptr) {
         delete m_pcWiFiEquippedEntityGridUpdateOperation;
      }
   }

   /****************************************/
   /****************************************/

   static size_t HashWiFiPair(const std::pair<CWiFiEquippedEntity*, CWiFiEquippedEntity*>& c_pair) {
      return
         reinterpret_cast<size_t>(c_pair.first) ^
         reinterpret_cast<size_t>(c_pair.second);
   }

   void CWiFiMedium::Update() {
      /* Update positional index of WiFi entities */
      m_pcWiFiEquippedEntityIndex->Update();
      /* Delete routing table */
      for(TRoutingTable::iterator it = m_tRoutingTable.begin();
          it != m_tRoutingTable.end();
          ++it) {
         it->second.clear();
      }
      /* This map contains the pairs that have already been checked */
      unordered_map<ssize_t, std::pair<CWiFiEquippedEntity*, CWiFiEquippedEntity*> > mapPairsAlreadyChecked;
      /* Iterator for the above structure */
      unordered_map<ssize_t, std::pair<CWiFiEquippedEntity*, CWiFiEquippedEntity*> >::iterator itPair;
      /* Used as test key */
      std::pair<CWiFiEquippedEntity*, CWiFiEquippedEntity*> cTestKey;
      /* Used as hash for the test key */
      UInt64 unTestHash;
      /* The ray to use for occlusion checking */
      CRay3 cOcclusionCheckRay;
      /* Buffer for the communicating entities */
      CSet<CWiFiEquippedEntity*,SEntityComparator> cOtherWiFis;
      /* Buffer to store the intersection data */
      SEmbodiedEntityIntersectionItem sIntersectionItem;
      /* The distance between two WiFis in line of sight */
      Real fDistance;
      /* Go through the WiFi entities */
      for(TRoutingTable::iterator it = m_tRoutingTable.begin();
          it != m_tRoutingTable.end();
          ++it) {
         /* Get a reference to the current WiFi entity */
         CWiFiEquippedEntity& cWiFi = *reinterpret_cast<CWiFiEquippedEntity*>(GetSpace().GetEntityVector()[it->first]);
         /* Initialize the occlusion check ray start to the position of the robot */
         cOcclusionCheckRay.SetStart(cWiFi.GetPosition());
         /* For each WiFi entity, get the list of WiFi entities in range */
         cOtherWiFis.clear();
         m_pcWiFiEquippedEntityIndex->GetEntitiesAt(cOtherWiFis, cWiFi.GetPosition());
         /* Go through the WiFi entities in range */
         for(CSet<CWiFiEquippedEntity*>::iterator it2 = cOtherWiFis.begin();
             it2 != cOtherWiFis.end();
             ++it2) {
            /* Get a reference to the WiFi entity */
            CWiFiEquippedEntity& cOtherWiFi = **it2;
            /* First, make sure the entities are not the same */
            if(&cWiFi != &cOtherWiFi) {
               /* Proceed if the pair has not been checked already */
               if(&cWiFi < &cOtherWiFi) {
                  cTestKey.first = &cWiFi;
                  cTestKey.second = &cOtherWiFi;
               }
               else {
                  cTestKey.first = &cOtherWiFi;
                  cTestKey.second = &cWiFi;
               }
               unTestHash = HashWiFiPair(cTestKey);
               itPair = mapPairsAlreadyChecked.find(unTestHash);
               if(itPair == mapPairsAlreadyChecked.end() ||   /* Pair does not exist */
                  itPair->second.first != cTestKey.first ||   /* Pair exists, but first WiFi involved is different */
                  itPair->second.second != cTestKey.second) { /* Pair exists, but second WiFi involved is different */
                  /* Mark this pair as already checked */
                  mapPairsAlreadyChecked[unTestHash] = cTestKey;
                  /* Proceed if the message size is compatible */
                  if(1) {
                     /* Proceed if the two entities are not obstructed by another object */
                     cOcclusionCheckRay.SetEnd(cOtherWiFi.GetPosition());
                     if((!m_bCheckOcclusions) ||
                        (!GetClosestEmbodiedEntityIntersectedByRay(sIntersectionItem,
                                                                   cOcclusionCheckRay,
                                                                   cWiFi.GetEntityBody())) ||
                        (&cOtherWiFi.GetEntityBody() == sIntersectionItem.IntersectedEntity)) {
                        /* If we get here, the two WiFi entities are in direct line of sight */
                        /* cWiFi can receive cOtherWiFi's message if it is in range, and viceversa */
                        /* Calculate square distance */
                        fDistance = cOcclusionCheckRay.GetLength();
                        if(fDistance < cOtherWiFi.GetRange()) {
                           /* cWiFi receives cOtherWiFi's message */
                           m_tRoutingTable[cWiFi.GetIndex()].insert(&cOtherWiFi);
                        }
                        if(fDistance < cWiFi.GetRange()) {
                           /* cOtherWiFi receives cWiFi's message */
                           m_tRoutingTable[cOtherWiFi.GetIndex()].insert(&cWiFi);
                        }
                     } // occlusion found?
                  } // is msg size the same?
               } // is check necessary?
            } // are entities the same?
         } // for entities in range
      } // for routing table
   }

   /****************************************/
   /****************************************/

   void CWiFiMedium::AddEntity(CWiFiEquippedEntity& c_entity) {
      m_tRoutingTable.insert(
         std::make_pair<ssize_t, CSet<CWiFiEquippedEntity*,SEntityComparator> >(
            c_entity.GetIndex(), CSet<CWiFiEquippedEntity*,SEntityComparator>()));
      m_pcWiFiEquippedEntityIndex->AddEntity(c_entity);
      m_pcWiFiEquippedEntityIndex->Update();
   }

   /****************************************/
   /****************************************/

   void CWiFiMedium::RemoveEntity(CWiFiEquippedEntity& c_entity) {
      m_pcWiFiEquippedEntityIndex->RemoveEntity(c_entity);
      m_pcWiFiEquippedEntityIndex->Update();
      TRoutingTable::iterator it = m_tRoutingTable.find(c_entity.GetIndex());
      if(it != m_tRoutingTable.end())
         m_tRoutingTable.erase(it);
   }

   /****************************************/
   /****************************************/

   const CSet<CWiFiEquippedEntity*,SEntityComparator>& CWiFiMedium::GetWiFisCommunicatingWith(CWiFiEquippedEntity& c_entity) const {
      TRoutingTable::const_iterator it = m_tRoutingTable.find(c_entity.GetIndex());
      if(it != m_tRoutingTable.end()) {
         return it->second;
      }
      else {
         THROW_ARGOSEXCEPTION("WiFi entity \"" << c_entity.GetContext() << c_entity.GetId() << "\" is not managed by the WiFi medium \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CWiFiMedium,
                   "wifi",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "It simulates the communication across range-and-bearing-equipped robots.",
                   "This medium is required to simulate communication across range-and-bearing-\n"
                   "equipped robots. You need to add it to the <media> section every time you add\n"
                   "a range-and-bearing-equipped entity whose controller has a range-and-bearing\n"
                   "device activated.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "<wifi id=\"wifi\" />\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "By default, the WiFi medium requires two robots to be in direct line-of-sight in\n"
                   "order to be able to exchange messages. You can toggle this behavior on or off\n"
                   "through the 'check_occlusions' attribute:\n\n"
                   "<wifi id=\"wifi\" check_occlusions=\"false\" />\n\n",
                   "Under development"
      );

   /****************************************/
   /****************************************/

}
