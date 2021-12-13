/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system.h>
 *
 * Provides the impolementation of the Iridia Tracking System physics engine.
 * The Iridia Tracking System physics engine is an engine that sets the position of the robots
 * according to the values provided in the Arena State Struct.
 *
 * The Iridia Tracking System is meant to work as a client of a remote Tracking System Server
 * that updates the data structure expoloited by the physics engine.
 *
 * The Iridia Tracking System physics engine can be used in ARGoS by including the tag
 * <iridia_tracking_system> in the <physics_engines> section of the ARGoS XML configuration file.
 *
 * Mandatory arguments of the <iriida_tracking_system> tag are:
 *      id
 *      its_host
 *      its_port
 *      vss_host
 *      vss_port
 *
 * Optional arguments are:
 *      translate_y
 *      translate_x
 * to set the center of the arena inside the big arena room.
 *
 *
 *
 * @author Mattia Salvaro
 */

#ifndef IRIDIA_TRACKING_SYSTEM_H
#define IRIDIA_TRACKING_SYSTEM_H

namespace argos {
   class CArgosITSClientThread;
}

namespace argos {
   class CIridiaTrackingSystem;
   class CIridiaTrackingSystemModel;
   class CEmbodiedEntity;
   class CITSModelCheckIntersectionOperation;
   class CITSModelGridUpdater;
}

#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/argos_its_client/arena_state_struct.h>
#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_server.h>

#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include <pthread.h>

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>


namespace argos {

    class CIridiaTrackingSystem : public CPhysicsEngine
    {

    public:

        /**
         * @brief TArenaState map<Robot Tag, State of the robot in the real world>
         */
        typedef CArenaStateStruct::TArenaState TArenaState;

        /**
         * @brief TTableRobotId <Argos ID, <Robot Tag, Robot ID> >
         */
        typedef std::map<std::string, std::pair<UInt32, UInt32> > TTableRobotId;

        /**
         * @brief TRobotState pair<Robot Tag, State of the robot in the real world>
         */
        typedef CArenaStateStruct::TRobotState TRobotState;

    public:

        CIridiaTrackingSystem();

        //~CIridiaTrackingSystem();

        /*************************************/
        /* Inherited methods *****************/
        /*************************************/

        virtual void Init(TConfigurationNode& t_tree);
        virtual void Reset();
        virtual void Destroy();

        virtual void PostSpaceInit();

        virtual void Update();

        virtual bool IsPointContained(const CVector3& c_point);

        virtual UInt32 GetNumPhysicsEngineEntities();

        virtual size_t GetNumPhysicsModels(){
        	return m_tPhysicsModels.size();
        }

        virtual bool AddEntity(CEntity& c_entity);

        virtual bool RemoveEntity(CEntity& c_entity);

        void AddPhysicsModel(const std::string& str_id,
                             CIridiaTrackingSystemModel& c_model);

        void RemovePhysicsModel(const std::string& str_id);


        inline virtual bool IsEntityTransferNeeded() const {
            return false;
        }

        virtual void TransferEntities() {}

        virtual CEmbodiedEntity* CheckIntersectionWithRay(Real& f_t_on_ray,
                                                          const CRay3& c_ray) const;

        virtual void CheckIntersectionWithRay(TEmbodiedEntityIntersectionData& t_data,
                                              const CRay3& c_ray) const;

        /*************************************/
        /*************************************/

        /**
         * @brief PositionAndOrientationPhysicsToSpace Sets position and orientation of the robot at once
         * @param c_new_pos The new given position
         * @param c_new_orient The new given orientation
         * @param str_id The Argos ID of the robot
         */
        void PositionAndOrientationPhysicsToSpace(CVector3& c_new_pos, CQuaternion& c_new_orient, std::string str_id);

        /**
         * @brief IsExperimentFinished Check whether the experiment is finished
         * @return True if the experiment is finished, false otherwise
         */
        bool IsExperimentFinished();

        /**
         * @brief TerminateExperiment Triggers the termination of the experiment
         */
        void TerminateExperiment();

    private:

        /**
         * @brief InitArenaState Initializes the Arena State and the Table Robot ID
         */
        void InitArenaState();

        /**
         * @brief CreateOdomSubscribers Subscribes to the odometry topic of all robots.
         */
        void CreateOdomSubscribers();

        /**
         * @brief OdomCallback The callback function for the odometry topics.
         */
        void OdomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);

    private:

        /**
         * @brief m_tPhysicsModels <Argos ID, Pointer to the instance of the physics model>
         */
        std::map<std::string, CIridiaTrackingSystemModel*> m_tPhysicsModels;

        /**
         * @brief m_cSimulator A reference to the CSimulator instance
         */
        CSimulator& m_cSimulator;

        /**
         * @brief m_cSpace A reference to the CSpace instance
         */
        CSpace& m_cSpace;

        /**
         * @brief m_vecUsedRobotTagList List of all the tags used in the experiment
         */
        std::vector<UInt32> m_vecUsedRobotTagList;

        /**
         * @brief m_unVirtualSensorServerPort Port on which the VSS is running on the local machine
         */
        UInt32 m_unVirtualSensorServerPort;

        /**
         * @brief m_cVirtualSensorServer Reference to the singleton Virtual Sensor Server
         */
        CVirtualSensorServer & m_cVirtualSensorServer;

        /**
         * @brief m_cArenaStateStruct Reference to the singleton Arena State Struct
         */
        CArenaStateStruct & m_cArenaStateStruct;

        /**
         * @brief m_cArenaCenter3D Physics Arena Center
         */
        CVector3 m_cArenaCenter3D;

        /**
         * @brief m_tTableRobotId Table that binds Argos ID with Tag and Robot ID
         */
        TTableRobotId * m_tTableRobotId;

        /**
         * @brief m_bRealExperiment Flag that tells whether the experiment is real or simulated
         */
        bool m_bRealExperiment;

        /** A positional index for the Models entities */
		CPositionalIndex<CIridiaTrackingSystemModel>* m_pcITSModelIndex;

		/** The update operation for the grid positional index */
		CITSModelGridUpdater* m_pcITSModelGridUpdateOperation;

		/** Operation to check rays */
		CITSModelCheckIntersectionOperation* m_pcOperation;

        ros::NodeHandle* rosNode;

        std::string m_strTopic;

        ros::Subscriber timeSubscriber;

    };

    /****************************************/
    /****************************************/

    template <typename ACTION>
    class CIridiaTrackingSystemOperation : public CEntityOperation<ACTION, CIridiaTrackingSystem, SOperationOutcome> {
    public:
       virtual ~CIridiaTrackingSystemOperation() {}
    };

    class CIridiaTrackingSystemOperationAddEntity : public CIridiaTrackingSystemOperation<CIridiaTrackingSystemOperationAddEntity> {
    public:
       virtual ~CIridiaTrackingSystemOperationAddEntity() {}
    };

    class CIridiaTrackingSystemOperationRemoveEntity : public CIridiaTrackingSystemOperation<CIridiaTrackingSystemOperationRemoveEntity> {
    public:
       virtual ~CIridiaTrackingSystemOperationRemoveEntity() {}
    };


#define REGISTER_IRIDIA_TRACKING_SYSTEM_OPERATION(ACTION, OPERATION, ENTITY)        \
   REGISTER_ENTITY_OPERATION(ACTION, CIridiaTrackingSystem, OPERATION, SOperationOutcome, ENTITY);

#define REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATION_ADD_ENTITY(SPACE_ENTITY, ITS_MODEL) \
   class CIridiaTrackingSystemOperationAdd ## SPACE_ENTITY : public CIridiaTrackingSystemOperationAddEntity { \
   public:                                                              \
   CIridiaTrackingSystemOperationAdd ## SPACE_ENTITY() {}                         \
   virtual ~CIridiaTrackingSystemOperationAdd ## SPACE_ENTITY() {}                \
   SOperationOutcome ApplyTo(CIridiaTrackingSystem& c_engine,                            \
                SPACE_ENTITY& c_entity) {       \
      ITS_MODEL* pcPhysModel = new ITS_MODEL(c_engine,              \
                                             c_entity);             \
      c_engine.AddPhysicsModel(c_entity.GetId(),                        \
                               *pcPhysModel);                           \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         AddPhysicsModel(c_engine.GetId(), *pcPhysModel);               \
         return SOperationOutcome(true);                                \
   }                                                                    \
   };                                                                   \
   REGISTER_IRIDIA_TRACKING_SYSTEM_OPERATION(CIridiaTrackingSystemOperationAddEntity,         \
                                 CIridiaTrackingSystemOperationAdd ## SPACE_ENTITY, \
                                 SPACE_ENTITY);

#define REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATION_REMOVE_ENTITY(SPACE_ENTITY) \
   class CIridiaTrackingSystemOperationRemove ## SPACE_ENTITY : public CIridiaTrackingSystemOperationRemoveEntity { \
   public:                                                              \
   CIridiaTrackingSystemOperationRemove ## SPACE_ENTITY() {}                      \
   virtual ~CIridiaTrackingSystemOperationRemove ## SPACE_ENTITY() {}             \
   SOperationOutcome ApplyTo(CIridiaTrackingSystem& c_engine,                            \
                SPACE_ENTITY& c_entity) {                               \
      c_engine.RemovePhysicsModel(c_entity.GetId());                    \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         RemovePhysicsModel(c_engine.GetId());                          \
         return SOperationOutcome(true);                                \
   }                                                                    \
   };                                                                   \
   REGISTER_IRIDIA_TRACKING_SYSTEM_OPERATION(CIridiaTrackingSystemOperationRemoveEntity,      \
                                 CIridiaTrackingSystemOperationRemove ## SPACE_ENTITY, \
                                 SPACE_ENTITY);

#define REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATIONS_ON_ENTITY(SPACE_ENTITY, ITS_ENTITY) \
   REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATION_ADD_ENTITY(SPACE_ENTITY, ITS_ENTITY) \
   REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATION_REMOVE_ENTITY(SPACE_ENTITY)

   /****************************************/
   /****************************************/
}



#endif
