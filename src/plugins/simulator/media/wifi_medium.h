#ifndef WIFI_MEDIUM_H
#define WIFI_MEDIUM_H

namespace argos {
   class CWiFiMedium;
   class CWiFiEquippedEntity;
}

#include <argos3/core/simulator/medium/medium.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_sensor.h>
#include <argos3/plugins/simulator/entities/wifi_equipped_entity.h>

namespace argos {

   class CWiFiMedium : public CMedium {

   public:

      /**
       * Class constructor.
       */
      CWiFiMedium();

      /**
       * Class destructor.
       */
      virtual ~CWiFiMedium();

      virtual void Init(TConfigurationNode& t_tree);
      virtual void PostSpaceInit();
      virtual void Reset();
      virtual void Destroy();
      virtual void Update();

      /**
       * Adds the specified entity to the list of managed entities.
       * @param c_entity The entity to add.
       */
      void AddEntity(CWiFiEquippedEntity& c_entity);
      
      /**
       * Removes the specified entity from the list of managed entities.
       * @param c_entity The entity to remove.
       */
      void RemoveEntity(CWiFiEquippedEntity& c_entity);

      /**
       * Returns an immutable vector of WiFi entities that can communicated with the given entity.
       * @param c_entity The wanted entity.
       * @return An immutable vector of WiFi entities that can communicated with the given entity.       
       * @throws CARGoSException If the passed entity is not managed by this medium.
       */
      const CSet<CWiFiEquippedEntity*,SEntityComparator>& GetWiFisCommunicatingWith(CWiFiEquippedEntity& c_entity) const;

   private:

      /** Defines the routing table */
      typedef unordered_map<ssize_t, CSet<CWiFiEquippedEntity*,SEntityComparator> > TRoutingTable;

      /** The routing table, that associates each WiFi with the WiFis that can send a message to it */
      TRoutingTable m_tRoutingTable;

      /** A positional index for the WiFi entities */
      CPositionalIndex<CWiFiEquippedEntity>* m_pcWiFiEquippedEntityIndex;

      /** The update operation for the grid positional index */
      CWiFiEquippedEntityGridEntityUpdater* m_pcWiFiEquippedEntityGridUpdateOperation;

      /* Whether occlusions should be considered or not */
      bool m_bCheckOcclusions;

   };

}

#endif
