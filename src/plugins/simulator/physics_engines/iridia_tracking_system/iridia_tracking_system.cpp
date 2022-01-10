/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system.cpp>
 *
 * @author
 */

#include "iridia_tracking_system.h"
#include "iridia_tracking_system_model.h"

namespace argos {
    class CITSModelCheckIntersectionOperation : public CPositionalIndex<CIridiaTrackingSystemModel>::COperation {

    public:

        CITSModelCheckIntersectionOperation() : m_vecIntersections(), fTOnRay(0), cRay(CRay3()) {
        }

        virtual ~CITSModelCheckIntersectionOperation() {
            m_vecIntersections.clear();
        }

        virtual bool operator()(CIridiaTrackingSystemModel &c_model) {
            // Process this LED only if it's lit
            LOG << "PROCESSING MODEL: " << c_model.GetEmbodiedEntity().GetParent().GetTypeDescription() << std::endl;
            if (c_model.CheckIntersectionWithRay(fTOnRay, cRay)) {
                m_vecIntersections.push_back(
                        std::pair<Real, CEmbodiedEntity *>(fTOnRay, &(c_model.GetEmbodiedEntity())));
                if (c_model.GetEmbodiedEntity().GetParent().GetTypeDescription() == "box" ||
                    c_model.GetEmbodiedEntity().GetParent().GetTypeDescription() == "cylinder")
                    return false;
            }
            return true;
        }

        CEmbodiedEntity *GetFirstCollision() {
            if (m_vecIntersections.size())
                return m_vecIntersections[0].second;
            return NULL;
        }

        CEmbodiedEntity *GetFirstNonRobotCollision() {
            for (UInt32 i = 0; i < m_vecIntersections.size(); ++i) {
                if (m_vecIntersections[i].second->GetParent().GetTypeDescription() == "box" ||
                    m_vecIntersections[i].second->GetParent().GetTypeDescription() == "cylinder")
                    return m_vecIntersections[i].second;
            }
            return NULL;
        }

        void Setup(Real &f_t_on_ray, const CRay3 &c_ray) {
            fTOnRay = f_t_on_ray;
            cRay = c_ray;
            m_vecIntersections.clear();
        }

    private:
        std::vector <std::pair<Real, CEmbodiedEntity *>> m_vecIntersections;
        Real fTOnRay;
        CRay3 cRay;
    };



    /****************************************/
    /****************************************/

    CIridiaTrackingSystem::CIridiaTrackingSystem() :
            m_cSimulator(CSimulator::GetInstance()),
            m_cSpace(m_cSimulator.GetSpace()),
            m_cVirtualSensorServer(CVirtualSensorServer::GetInstance()),
            m_cArenaStateStruct(CArenaStateStruct::GetInstance()),
            m_cArenaCenter3D(),
            m_bRealExperiment(true) {
        m_tTableRobotId = new std::map <std::string, std::pair<UInt32, UInt32>>();
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::Init(TConfigurationNode &t_tree) {
        CPhysicsEngine::Init(t_tree);

        // Parse XML file and collect information
        try {
            GetNodeAttributeOrDefault(t_tree, "real_experiment", m_bRealExperiment, m_bRealExperiment);
        }
        catch (CARGoSException &ex) {
            THROW_ARGOSEXCEPTION_NESTED("Failed to initialize its physics engines. Parse error in real_experiment.",
                                        ex);
        }
        try {
            GetNodeAttributeOrDefault<UInt32>(t_tree, "vss_port", m_unVirtualSensorServerPort, 4050);
        }
        catch (CARGoSException &ex) {
            THROW_ARGOSEXCEPTION_NESTED("Failed to initialize its physics engines. Parse error in vss_port.", ex);
        }


        // Set the Virtual Sensor Server Port in the Virtual Sensor Server
        m_cVirtualSensorServer.SetVirtualSensorServerPort(m_unVirtualSensorServerPort);

        Real fArenaCenterX;
        Real fArenaCenterY;
        try {
            GetNodeAttributeOrDefault<Real>(t_tree, "translate_x", fArenaCenterX, 0.0f);
            GetNodeAttributeOrDefault<Real>(t_tree, "translate_y", fArenaCenterY, 0.0f);
        }
        catch (CARGoSException &ex) {
            THROW_ARGOSEXCEPTION_NESTED(
                    "Failed to initialize its physics engines. Parse error in translate_x, translate_y.", ex);
        }

        m_cArenaCenter3D.SetX(fArenaCenterX);
        m_cArenaCenter3D.SetY(fArenaCenterY);

        // retrieve the topic name for the odometry topic
        try {
            GetNodeAttributeOrDefault<std::string>(t_tree, "topic", m_strTopic, "odom");
        }
        catch (CARGoSException &ex) {
            THROW_ARGOSEXCEPTION_NESTED(
                    "Failed to initialize its physics engines. Parse error in expected parameter \"topic\".", ex);
        }

        m_cStatusThread = new CTychoStatusThread(this);

        // Init ROS
        if (!ros::isInitialized()) {
            //init ROS
            char **argv = NULL;
            int argc = 0;
            ros::init(argc, argv, "automode");
            rosNode = new ros::NodeHandle();
        }
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::Reset() {
        m_cStatusThread->Reset();

        // Update the physics engine
        Update();
        // Set the step counter to 0
        m_cArenaStateStruct.ResetTimestepCounter();
        // Ready for a new run of experiment
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::Destroy() {
        // Disconnect from ROS?
        // Cancel status thread?
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::PostSpaceInit() {
        // Set the initial arena state as describe in the XML configuration file
        InitArenaState();

        // Subscribe to the topics of the robots
        CreateOdomSubscribers();

        // If the VSS is enabled, then launch it
        if (m_bRealExperiment) {
            DEBUG("VSS enabled\n");
            m_cVirtualSensorServer.Launch();
        }

        DEBUG("Arena State is initialized.\n");

        // For each entry in m_tPhysicsModels update its state
        Update();

        // Set the step counter to 0
        m_cArenaStateStruct.ResetTimestepCounter();
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::Update() {
        // Refresh ROS
        ros::spinOnce();

        // If this is the first step, start the Tracking System
        if (m_cSpace.GetSimulationClock() == 1) {
            m_cStatusThread->Start();
            if (m_bRealExperiment) {
                m_cVirtualSensorServer.ExperimentStarted();
                m_cVirtualSensorServer.SendArgosSignal(1);
            }
        }

        // For each physics model, call the UpdateEntityStatus
        for (CIridiaTrackingSystemModel::TMap::iterator it = m_tPhysicsModels.begin();
             it != m_tPhysicsModels.end(); ++it) {
            // Get readings from virtual sensors
            it->second->UpdateEntityStatus();

            m_cVirtualSensorServer.SwapBuffers((*m_tTableRobotId->find(it->first)).second.second);
        }

        // ... ask HERE the Virtual Sensor Server to send the data of all robots in a loop.
        if (m_bRealExperiment) {
            m_cVirtualSensorServer.SendAllVirtualSensorData();
        }

    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::PositionAndOrientationPhysicsToSpace(CVector3 &c_new_pos, CQuaternion &c_new_orient,
                                                                     std::string str_id) {
        // Look for Id's conversion in ITS ids (tag)
        // Then look for that ITS id's position in the m_ptArenaState map
        TTableRobotId::iterator itHashRobotId;
        // If the robot is not detected by the Tracking System do not change its position
        // Otherwise...
        if ((itHashRobotId = m_tTableRobotId->find(str_id)) != m_tTableRobotId->end()) {
            CArenaStateStruct::SRealWorldCoordinates sRealWorldCoordinates;
            try {
                UInt32 robotID = itHashRobotId->second.first;
                // Get the updated state from the Arena State Struct
                m_cArenaStateStruct.GetRobotState(sRealWorldCoordinates, robotID);
            }
            catch (CARGoSException ex) {
                LOGERR << "[FATAL] Error when reading robot state for robot " << str_id << std::endl;
                DEBUG("%s\n", ex.what());
                throw;
            }

            // Set new position and orientation of the robot
            c_new_pos = sRealWorldCoordinates.cPosition.operator-(m_cArenaCenter3D);
            c_new_orient = sRealWorldCoordinates.cOrientation;
        }
    }

    /****************************************/
    /****************************************/

    bool CIridiaTrackingSystem::IsExperimentFinished() {
        return m_cSimulator.IsExperimentFinished();
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::TerminateExperiment() {
        if (m_bRealExperiment) {
            m_cVirtualSensorServer.SendArgosSignal(0);
        }
        m_cSimulator.Terminate();
    }

    /****************************************/
    /****************************************/

    bool CIridiaTrackingSystem::IsPointContained(const CVector3 &c_point) {
        return true;
    }

    /****************************************/
    /****************************************/

    UInt32 CIridiaTrackingSystem::GetNumPhysicsEngineEntities() {
        return m_tPhysicsModels.size();
    }

    /****************************************/
    /****************************************/

    bool CIridiaTrackingSystem::AddEntity(CEntity &c_entity) {
        SOperationOutcome cOutcome = CallEntityOperation<CIridiaTrackingSystemOperationAddEntity, CIridiaTrackingSystem, SOperationOutcome>(
                *this, c_entity);
        return cOutcome.Value;
    }

    /****************************************/
    /****************************************/

    bool CIridiaTrackingSystem::RemoveEntity(CEntity &c_entity) {
        SOperationOutcome cOutcome = CallEntityOperation<CIridiaTrackingSystemOperationRemoveEntity, CIridiaTrackingSystem, SOperationOutcome>(
                *this, c_entity);
        return cOutcome.Value;
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::AddPhysicsModel(const std::string &str_id,
                                                CIridiaTrackingSystemModel &c_model) {
        m_tPhysicsModels[str_id] = &c_model;
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::RemovePhysicsModel(const std::string &str_id) {
        CIridiaTrackingSystemModel::TMap::iterator it = m_tPhysicsModels.find(str_id);
        if (it != m_tPhysicsModels.end()) {
            delete it->second;
            m_tPhysicsModels.erase(it);
        } else {
            THROW_ARGOSEXCEPTION(
                    "Model id \"" << str_id << "\" not found in Iridia tracking system \"" << GetId() << "\"");
        }
    }

    /****************************************/
    /****************************************/

    CEmbodiedEntity *CIridiaTrackingSystem::CheckIntersectionWithRay(Real &f_t_on_ray,
                                                                     const CRay3 &c_ray) const {
        f_t_on_ray = 1.0f;
        CEmbodiedEntity *cFirstCollision = NULL;
        SBoundingBox sBoundingBox;
        sBoundingBox.MinCorner.SetX(Min<Real>(c_ray.GetStart().GetX(), c_ray.GetEnd().GetX()));
        sBoundingBox.MinCorner.SetY(Min<Real>(c_ray.GetStart().GetY(), c_ray.GetEnd().GetY()));
        sBoundingBox.MinCorner.SetZ(Min<Real>(c_ray.GetStart().GetZ(), c_ray.GetEnd().GetZ()));
        sBoundingBox.MaxCorner.SetX(Max<Real>(c_ray.GetStart().GetX(), c_ray.GetEnd().GetX()));
        sBoundingBox.MaxCorner.SetY(Max<Real>(c_ray.GetStart().GetY(), c_ray.GetEnd().GetY()));
        sBoundingBox.MaxCorner.SetZ(Max<Real>(c_ray.GetStart().GetZ(), c_ray.GetEnd().GetZ()));
        std::map<std::string, CIridiaTrackingSystemModel *>::const_iterator itPhysicsModels;
        for (itPhysicsModels = m_tPhysicsModels.begin(); itPhysicsModels != m_tPhysicsModels.end(); itPhysicsModels++) {
            // if the ray intersect bounding box of the model, potential intersection
            if (sBoundingBox.Intersects((*itPhysicsModels).second->GetBoundingBox())) {
                if ((*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "box"
                    || (*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "cylinder") {
                    Real fTOnRay;
                    CEmbodiedEntity *cCollision = (*itPhysicsModels).second->CheckIntersectionWithRay(fTOnRay, c_ray);
                    //Delegate to the model to look for an intersection
                    if (cCollision) {
                        if (fTOnRay < f_t_on_ray) {
                            f_t_on_ray = fTOnRay;
                            cFirstCollision = cCollision;
                        }
                    }
                }
            }
        }
        return cFirstCollision;
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::CheckIntersectionWithRay(TEmbodiedEntityIntersectionData &t_data, const CRay3 &c_ray) const {
        SBoundingBox sBoundingBox;
        sBoundingBox.MinCorner.SetX(Min<Real>(c_ray.GetStart().GetX(), c_ray.GetEnd().GetX()));
        sBoundingBox.MinCorner.SetY(Min<Real>(c_ray.GetStart().GetY(), c_ray.GetEnd().GetY()));
        sBoundingBox.MinCorner.SetZ(Min<Real>(c_ray.GetStart().GetZ(), c_ray.GetEnd().GetZ()));
        sBoundingBox.MaxCorner.SetX(Max<Real>(c_ray.GetStart().GetX(), c_ray.GetEnd().GetX()));
        sBoundingBox.MaxCorner.SetY(Max<Real>(c_ray.GetStart().GetY(), c_ray.GetEnd().GetY()));
        sBoundingBox.MaxCorner.SetZ(Max<Real>(c_ray.GetStart().GetZ(), c_ray.GetEnd().GetZ()));
        std::map<std::string, CIridiaTrackingSystemModel *>::const_iterator itPhysicsModels;
        for (itPhysicsModels = m_tPhysicsModels.begin(); itPhysicsModels != m_tPhysicsModels.end(); itPhysicsModels++) {
            if (sBoundingBox.Intersects((*itPhysicsModels).second->GetBoundingBox())) {
                if ((*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "box"
                    || (*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "cylinder") {
                    Real fTOnRay;
                    CEmbodiedEntity *cCollision = (*itPhysicsModels).second->CheckIntersectionWithRay(fTOnRay, c_ray);
                    //Delegate to the model to look for an intersection
                    if (cCollision) {
                        t_data.push_back(SEmbodiedEntityIntersectionItem(cCollision, fTOnRay));
                    }
                }
            }
        }
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::InitArenaState() {
        // A list of the state of all the robots declared in the XML configuration file
        std::vector <TRobotState> vecXMLDeclaredInitialArenaState;

        // For each physics model
        for (CIridiaTrackingSystemModel::TMap::iterator it = m_tPhysicsModels.begin();
             it != m_tPhysicsModels.end(); ++it) {
            // Does the model is a footbot or an epuck?
            if ((it->second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "epuck")
                || (it->second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "foot-bot")) {
                // Get its Argos ID
                // The Argos ID is the ID of the robot as defined in the XML file.
                // The name must contain an arbitrary string - underscore - robot tag (ITS ID) - underscore - robot ID
                std::string strArgosId = it->first;
                // Tokenize it
                std::vector <std::string> vecTokens;
                std::stringstream cStringStream(strArgosId);
                std::string strToken;
                while (std::getline(cStringStream, strToken, '_')) {
                    vecTokens.push_back(strToken);
                }

                if (vecTokens.size() < 2) {
                    LOGERR << "[ERROR] Expected two ID tokens in robot identifier, but found only " << vecTokens.size() << std::endl;
                    LOGERR << "[ERROR] The full robot identifier was " << strArgosId << std::endl;
                    LOGERR << "[ERROR] Make sure that all robot identifiers follow the form [robot type]_[ITS ID]_[robot ID]" << std::endl;
                }
                // Convert ITS ID in int form
                UInt32 unITSId = FromString<UInt32>(vecTokens[vecTokens.size() - 2]);
                CArenaStateStruct::TRobotState tRobotState(std::make_pair(unITSId, CArenaStateStruct::SRealWorldCoordinates(
                                                                          it->second->GetEmbodiedEntity().GetOriginAnchor().Position +
                                                                          m_cArenaCenter3D,
                                                                          it->second->GetEmbodiedEntity().GetOriginAnchor().Orientation,
                                                                          (UInt32) 0)));
                // Convert robot ID in int form
                UInt32 unRobotId = FromString<UInt32>(vecTokens[vecTokens.size() - 1]);

                // Create a pair <ITS ID, Robot ID>
                std::pair <UInt32, UInt32> pairITSIdRobotId = std::make_pair(unITSId, unRobotId);
                // Fill the Robot ID Table
                m_tTableRobotId->insert(std::make_pair(strArgosId, pairITSIdRobotId));
                // Fill another vector with robot tag only
                m_vecUsedRobotTagList.push_back(tRobotState.first);
                // Fill a vector with robot state
                vecXMLDeclaredInitialArenaState.push_back(tRobotState);
            }

        }
        // Set the Initial Arena State as it is defined in the XML configuration file
        m_cArenaStateStruct.SetInitalArenaState(vecXMLDeclaredInitialArenaState);
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::CreateOdomSubscribers() {
        std::stringstream topic;

        for(std::vector<UInt32>::iterator it = std::begin(m_vecUsedRobotTagList); it != std::end(m_vecUsedRobotTagList); ++it) {
            //init color
            topic.str("");
            topic << "/epuck_" << *it << "/" << m_strTopic;
            LOG << topic.str() << std::endl;
            odomSubscribers[*it] = rosNode->subscribe(topic.str(), 1000, &CIridiaTrackingSystem::OdomCallback, this);
        }
        // m_vecUsedRobotTagList
        for (int j = 0; j < 40; j++) {

        }
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::OdomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event) {
        const nav_msgs::OdometryConstPtr& msg = event.getMessage();
        // get the robot ID
        std::string topic = event.getConnectionHeader().at("topic");
        topic = topic.substr(1, topic.length()); // remove initial /
        topic = topic.substr(0, topic.find("/"));
        UInt32 robotID = std::stoi(topic.substr(6, topic.length())); // remove "epuck_"
        // get the position and orientation
        CArenaStateStruct::SRealWorldCoordinates coordinates = *(new CArenaStateStruct::SRealWorldCoordinates());
        coordinates.cPosition[0] = msg->pose.pose.position.x;
        coordinates.cPosition[1] = msg->pose.pose.position.y;
        coordinates.cPosition[2] = msg->pose.pose.position.z;
        coordinates.cOrientation.SetX(msg->pose.pose.orientation.x);
        coordinates.cOrientation.SetY(msg->pose.pose.orientation.y);
        coordinates.cOrientation.SetZ(msg->pose.pose.orientation.z);
        coordinates.cOrientation.SetW(msg->pose.pose.orientation.w);
        // set the new information
        TRobotState robotState = *(new TRobotState(robotID, coordinates));
        m_cArenaStateStruct.SetRobotState(robotState);
    }

    /****************************************/
    /****************************************/

    REGISTER_PHYSICS_ENGINE(CIridiaTrackingSystem,
                            "iridia_tracking_system",
                            "Mattia Salvaro [mattia.salvaro@gmail.com]",
                            "1.0",
                            "Real world position display.",
                            "This physics engine is a display of positions of robots in real world\n"
                            "\n"
                            "REQUIRED XML CONFIGURATION\n\n"
                            "  <physics_engines>\n"
                            "    ...\n"
                            "    <iridia_tracking_system id=\"its\" />\n"
                            "    ...\n"
                            "  </physics_engines>\n\n"
                            "The 'id' attribute is necessary and must be unique among the physics engines.\n"
                            "It is used in the subsequent section <arena_physics> to assign entities to\n"
                            "physics engines. If two engines share the same id, initialization aborts.\n\n"
                            "OPTIONAL XML CONFIGURATION\n\n"
                            "The plane of the physics engine can be translated on the Z axis, to simulate\n"
                            "for example hovering objects, such as flying robots. To translate the plane\n"
                            "2m up the Z axis, use the 'elevation' attribute as follows:\n\n"
                            "  <physics_engines>\n"
                            "    ...\n"
                            "    <iridia_tracking_system id=\"its\"\n"
                            "                elevation=\"2.0\" />\n"
                            "    ...\n"
                            "  </physics_engines>\n\n"
                            "When not specified, the elevation is zero, which means that the plane\n"
                            "corresponds to the XY plane.\n",
                            "Under development"
       );

    /****************************************/
    /****************************************/
}
