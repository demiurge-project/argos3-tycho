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

      CITSModelCheckIntersectionOperation():m_vecIntersections(),fTOnRay(0),cRay(CRay3()){
      }

      virtual ~CITSModelCheckIntersectionOperation() {
    	  m_vecIntersections.clear();
      }

      virtual bool operator()(CIridiaTrackingSystemModel& c_model) {
         // Process this LED only if it's lit
    	  LOG << "PROCESSING MODEL: " << c_model.GetEmbodiedEntity().GetParent().GetTypeDescription() << std::endl;
         if (c_model.CheckIntersectionWithRay(fTOnRay,cRay)){
        	 m_vecIntersections.push_back(std::pair<Real,CEmbodiedEntity*>(fTOnRay,&(c_model.GetEmbodiedEntity())));
        	 if (c_model.GetEmbodiedEntity().GetParent().GetTypeDescription() == "box" || c_model.GetEmbodiedEntity().GetParent().GetTypeDescription() == "cylinder")
        		 return false;
         }
         return true;
      }

      CEmbodiedEntity* GetFirstCollision(){
		  if (m_vecIntersections.size())
			  return m_vecIntersections[0].second;
		  return NULL;
      }

      CEmbodiedEntity* GetFirstNonRobotCollision(){
    	  for (UInt32 i=0; i<m_vecIntersections.size(); ++i){
    		  if (m_vecIntersections[i].second->GetParent().GetTypeDescription() == "box" || m_vecIntersections[i].second->GetParent().GetTypeDescription() == "cylinder")
    			  return m_vecIntersections[i].second;
    	  }
          return NULL;
      }

      void Setup(Real& f_t_on_ray,  const CRay3& c_ray) {
    	  fTOnRay = f_t_on_ray;
    	  cRay=c_ray;
          m_vecIntersections.clear();
      }

   private:
      std::vector<std::pair<Real,CEmbodiedEntity*> > m_vecIntersections;
      Real fTOnRay;
      CRay3 cRay;
   };



    /****************************************/
    /****************************************/

    CIridiaTrackingSystem::CIridiaTrackingSystem() :
    	m_cThread(),
        m_cSimulator(CSimulator::GetInstance()),
        m_cSpace(m_cSimulator.GetSpace()),
        m_unITSServerPort(4040),
        m_cVirtualSensorServer(CVirtualSensorServer::GetInstance()),
        m_cArenaStateStruct(CArenaStateStruct::GetInstance()),
        m_cArenaCenter3D(),
        m_bRealExperiment(true),
        m_strResultsFile("")
    {
        m_tTableRobotId = new std::map<std::string, std::pair<UInt32, UInt32> > ();
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::Init(TConfigurationNode& t_tree) {
        CPhysicsEngine::Init(t_tree);

        // Parse XML file and collect information
        try {
            GetNodeAttributeOrDefault<std::string>(t_tree, "its_host", m_strITSServerAddress, "169.254.0.200");
            GetNodeAttributeOrDefault<UInt32>(t_tree, "its_port", m_unITSServerPort, m_unITSServerPort);
        }
        catch(CARGoSException& ex) {
           THROW_ARGOSEXCEPTION_NESTED("Failed to initialize its physics engines. Parse error in its_host, its_port.", ex);
        }
        try{
            GetNodeAttributeOrDefault(t_tree, "real_experiment", m_bRealExperiment, m_bRealExperiment);
        }
        catch(CARGoSException& ex) {
           THROW_ARGOSEXCEPTION_NESTED("Failed to initialize its physics engines. Parse error in real_experiment.", ex);
        }
        try {
            GetNodeAttributeOrDefault<UInt32>(t_tree, "vss_port", m_unVirtualSensorServerPort, 4050);
        }
        catch(CARGoSException& ex) {
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
        catch(CARGoSException& ex) {
           THROW_ARGOSEXCEPTION_NESTED("Failed to initialize its physics engines. Parse error in translate_x, translate_y.", ex);
        }

        m_cArenaCenter3D.SetX(fArenaCenterX);
        m_cArenaCenter3D.SetY(fArenaCenterY);

        // Check whether there is a results file as input
        try {
            GetNodeAttributeOrDefault<std::string>(t_tree, "results_file", m_strResultsFile, m_strResultsFile);
            if (IsUsingResultsFile()) {
                m_cClient->OpenResultsFile(m_strResultsFile);
                m_bRealExperiment = false;
            }
        }
        catch(CARGoSException& ex) {
           THROW_ARGOSEXCEPTION_NESTED("Failed to initialize its physics engines. Parse error in results_file.", ex);
        }

        // Creates the instance of the Argos ITS Client Thread
        m_cClient = new CArgosITSClientThread(this, m_strITSServerAddress, m_unITSServerPort);

        // Spawn the thread if the experiment is set real in the configuration file
        if (m_bRealExperiment) {
            pthread_create(&m_cThread, NULL, (void * (*) (void *)) &argos::CIridiaTrackingSystem::start, (void *)m_cClient);
        }

        //init ROS
        if(!ros::isInitialized())
        {
            ros::Publisher proximityPublishers[40][8];
            ros::Publisher groundPublishers[40][3];
            sensor_msgs::Range proximityMsgs[40][8];
            ros::Publisher lightPublishers[40][8];
            ros::Publisher colorPublisher[40];
            sensor_msgs::Range groundMsgs[40][3];
            sensor_msgs::Range lightMsgs[40][8];

            //init ROS
            char** argv = NULL;
            int argc = 0;
            ros::init(argc, argv, "automode");

            ros::NodeHandle rosNode;
            timeSubscriber = rosNode.subscribe("/epuck0/time", 1000, timeCallback);

            std::stringstream ss;
            for (int j = 0; j < 40; j++)
            {
                //init proximity and light
                for (int i = 0; i < 8; i++)
                {
                    ss.str("");
                    ss << "epuck" << j << "/proximity";
                    proximityPublishers[j][i] = rosNode.advertise<sensor_msgs::Range>(ss.str(), 10);
                    proximityMsgs[j][i].radiation_type = sensor_msgs::Range::INFRARED;
                    ss.str("");
                    ss << "epuck" << j << "/base_prox" << i;
                    proximityMsgs[j][i].header.frame_id = ss.str();
                    proximityMsgs[j][i].field_of_view = 0.26;
                    proximityMsgs[j][i].min_range = 0;
                    proximityMsgs[j][i].max_range = 1;

                    ss.str("");
                    ss << "epuck" << j << "/light";
                    lightPublishers[j][i] = rosNode.advertise<sensor_msgs::Range>(ss.str(), 10);
                    lightMsgs[j][i].radiation_type = sensor_msgs::Range::INFRARED;
                    ss.str("");
                    ss << "epuck" << j << "/base_light" << i;
                    lightMsgs[j][i].header.frame_id = ss.str();
                    lightMsgs[j][i].min_range = 0;
                    lightMsgs[j][i].max_range = 1;
                }

                //init ground
                for (int i = 0; i < 3; ++i)
                {
                    ss.str("");
                    ss << "epuck" << j << "/ground";
                    groundPublishers[j][i] = rosNode.advertise<sensor_msgs::Range>(ss.str(), 10);
                    groundMsgs[j][i].radiation_type = sensor_msgs::Range::INFRARED;
                    ss.str("");
                    ss << "epuck" << j << "/base_ground" << i;
                    groundMsgs[j][i].header.frame_id = ss.str();
                    groundMsgs[j][i].min_range = 0;
                    groundMsgs[j][i].max_range = 1;
                }

                //init color

                ss.str("");
                ss << "epuck" << j << "/color";
                colorPublisher[j] = rosNode.advertise<std_msgs::ColorRGBA>(ss.str(), 10);

                //init color handler
                //colorSubscriber[j]  = rosNode.subscribe("color", 10, &ReferenceModel3Dot0::handlerLED, this);

                //init behavior handler
                //ss.str("");
                //ss << "epuck" << j << "/behavior";
                //behaviorSubscriber[j]  = rosNode.subscribe(ss.str(), 10, &ReferenceModel3Dot0::handlerBehavior, this);
            }
        }
    }

    void CIridiaTrackingSystem::timeCallback(const std_msgs::Time::ConstPtr& msg) {
        LOG << msg->data.sec << std::endl;
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::Reset()
    {
        /*
        if (IsUsingResultsFile()) {
            m_cClient->OpenResultsFile(m_strResultsFile);
            m_bRealExperiment = false;
        }

        // Notify the clients to move towards the target
        //m_cVirtualSensorServer.ReplaceRobots();
        // ReplaceRobots contains SendAllVirtualSensorData in a loop
        // wait until all robots are back in place

        // Untrigger the Client Thread
        m_cClient->TriggerTrakingSystem();
        // Wait for the Arena State Init
        while(!m_cArenaStateStruct.IsArenaStateInit()) {}
        // Update the physics engine
        Update();
        // Set the step counter to 0
        m_cArenaStateStruct.ResetTimestepCounter();
        // Ready for a new run of experiment

        //Reset the index
        //m_pcITSModelIndex->Reset();
         */
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::Destroy() {
        // Disconnect to tracking system
        m_cClient->DisconnectFromITSServer();
        // Cancel threads
        // The thread will terminate itself
        /*delete m_pcITSModelIndex;
        if(m_pcITSModelGridUpdateOperation != NULL) {
            delete m_pcITSModelGridUpdateOperation;
        }*/
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::PostSpaceInit()
    {

        //DEBUG("Number of physics models = %d\n", GetNumPhysicsEngineEntities());

        // Set the initial arena state as describe in the XML configuration file
        InitArenaState();

        /*

        // Notify the clients to move towards the target
        //m_cVirtualSensorServer.ReplaceRobots();
        // ReplaceRobots contains SendAllVirtualSensorData in a loop
        // wait until all robots are back in place

        // If the experiment is a Real Experiment
        if (m_bRealExperiment) {
            // Wait for the Merged Arena State to be ready
            while (!m_cArenaStateStruct.IsArenaStateInit()) {}
        }


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
        // Init first values of the index
        //m_pcITSModelIndex->Update();

         */
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::Update() {
        ros::spinOnce();
	// Get data from the tracking system
    // For-loop through the models to call

        //DEBUG_FUNCTION_ENTER;

        /*
        // If a results file is used, get the next line and forget all the rest
        if (IsUsingResultsFile()) {
            if (m_cClient->GetNextArenaStateFromResultsFile()) {
                // For each physics model, call the UpdateEntityStatus
                for(CIridiaTrackingSystemModel::TMap::iterator it = m_tPhysicsModels.begin();
                    it != m_tPhysicsModels.end(); ++it)
                {
                   //DEBUG("Updating robot id: %s\n", it->first.c_str());
                   it->second->UpdateEntityStatus();
                   m_cVirtualSensorServer.SwapBuffers((*m_tTableRobotId->find(it->first)).second.second);
                }
            }
            else {
                TerminateExperiment();
                //DEBUG_FUNCTION_EXIT;
                return;
            }

            m_cClient->WriteFilteredArenaState();


        }
        else {
            // If this is the first step, start the Tracking System
            if (m_cSpace.GetSimulationClock() == 1) {
                m_cClient->TriggerTrakingSystem();
                if (m_bRealExperiment) {
                    m_cVirtualSensorServer.ExperimentStarted();
                    m_cVirtualSensorServer.SendArgosSignal(1);
                }
            }

            // For each physics model, call the UpdateEntityStatus
            for(CIridiaTrackingSystemModel::TMap::iterator it = m_tPhysicsModels.begin();
                it != m_tPhysicsModels.end(); ++it)
            {
               //DEBUG("Updating robot id: %s\n", it->first.c_str());

               // Get readings from virtual sensors
               it->second->UpdateEntityStatus();

               // Swap the virtual sensor data double buffer
               //if (m_bRealExperiment) {
                   m_cVirtualSensorServer.SwapBuffers((*m_tTableRobotId->find(it->first)).second.second);
               //}

            }

            // ... ask HERE the Virtual Sensor Server to send the data of all robots in a loop.
            if (m_bRealExperiment) {
                m_cVirtualSensorServer.SendAllVirtualSensorData();
            }
        }

        //m_pcITSModelIndex->Update();
*/
        //DEBUG_FUNCTION_EXIT;

    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::PositionAndOrientationPhysicsToSpace(CVector3 &c_new_pos, CQuaternion &c_new_orient, std::string str_id)
    {
        // Look for Id's conversion in ITS ids (tag)
        // Then look for that ITS id's position in the m_ptArenaState map

        TTableRobotId::iterator itHashRobotId;

        // If the robot is not detected by the Tracking System do not change its position
        // Otherwise...
        if ((itHashRobotId = m_tTableRobotId->find(str_id)) != m_tTableRobotId->end()) {

            CArenaStateStruct::SRealWorldCoordinates sRealWorldCoordinates;
            try {
                // Get the updated state from the Arena State Struct
                m_cArenaStateStruct.GetRobotState(sRealWorldCoordinates, (*itHashRobotId).second.first);
            }
            catch (CARGoSException ex) {
                DEBUG("%s\n", ex.what());
            }

            // Set new position and orientation of the robot
            c_new_pos = sRealWorldCoordinates.cPosition.operator-(m_cArenaCenter3D);
            c_new_orient = sRealWorldCoordinates.cOrientation;
        }
    }

    /****************************************/
    /****************************************/

    void *CIridiaTrackingSystem::start(void* p) {
        // Notify the Client Thread to start
        CArgosITSClientThread *cClientThread = reinterpret_cast<CArgosITSClientThread*>(p);
        cClientThread->Run();
        return p;
    }

    /****************************************/
    /****************************************/

    bool CIridiaTrackingSystem::IsExperimentFinished()
    {
        bool bExperimentFinished = m_cSimulator.IsExperimentFinished();
        if (bExperimentFinished) {
            //m_cVirtualSensorServer.SendArgosSignal(SInt32(-1));
        }
        return m_cSimulator.IsExperimentFinished();
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::TerminateExperiment()
    {
        if (m_bRealExperiment) {
            m_cVirtualSensorServer.SendArgosSignal(0);
        }

        m_cSimulator.Terminate();
    }

    /****************************************/
    /****************************************/

    bool CIridiaTrackingSystem::IsPointContained(const CVector3& c_point) {
        return true;
    }

    /****************************************/
    /****************************************/

    UInt32 CIridiaTrackingSystem::GetNumPhysicsEngineEntities() {
       return m_tPhysicsModels.size();
    }

    /****************************************/
    /****************************************/

    bool CIridiaTrackingSystem::AddEntity(CEntity& c_entity) {
        SOperationOutcome cOutcome = CallEntityOperation<CIridiaTrackingSystemOperationAddEntity, CIridiaTrackingSystem, SOperationOutcome>(*this, c_entity);
        return cOutcome.Value;
        /*
        std::map<std::string, CIridiaTrackingSystemModel*>::const_iterator itPhysicsModels;
        for (itPhysicsModels = m_tPhysicsModels.begin(); itPhysicsModels != m_tPhysicsModels.end(); itPhysicsModels++) {
        	if (itPhysicsModels->second->GetEmbodiedEntity().GetRootEntity() == c_entity)
        		break;
        }
        if (itPhysicsModels != m_tPhysicsModels.end())
        	m_pcITSModelIndex->AddEntity(*(itPhysicsModels->second));
        else
        	THROW_ARGOSEXCEPTION("CIridiaTrackingSystem::AddEntity : Cannot find corresponding model")
        */
    }

    /****************************************/
    /****************************************/

    bool CIridiaTrackingSystem::RemoveEntity(CEntity& c_entity) {
      SOperationOutcome cOutcome = CallEntityOperation<CIridiaTrackingSystemOperationRemoveEntity, CIridiaTrackingSystem, SOperationOutcome>(*this, c_entity);
      return cOutcome.Value;
        /*
        std::map<std::string, CIridiaTrackingSystemModel*>::const_iterator itPhysicsModels;
		for (itPhysicsModels = m_tPhysicsModels.begin(); itPhysicsModels != m_tPhysicsModels.end(); itPhysicsModels++) {
			if (itPhysicsModels->second->GetEmbodiedEntity().GetRootEntity() == c_entity)
				break;
		}
		if (itPhysicsModels != m_tPhysicsModels.end())
			m_pcITSModelIndex->RemoveEntity(*(itPhysicsModels->second));
		else
			THROW_ARGOSEXCEPTION("CIridiaTrackingSystem::RemoveEntity : Cannot find corresponding model")
      */
    }

    /****************************************/
    /****************************************/

   void CIridiaTrackingSystem::AddPhysicsModel(const std::string& str_id,
                                            CIridiaTrackingSystemModel& c_model) {
      m_tPhysicsModels[str_id] = &c_model;
      //m_pcITSModelIndex->AddEntity(c_model);
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystem::RemovePhysicsModel(const std::string& str_id) {
      CIridiaTrackingSystemModel::TMap::iterator it = m_tPhysicsModels.find(str_id);
      if(it != m_tPhysicsModels.end()) {
         //m_pcITSModelIndex->RemoveEntity(*(it->second));
         delete it->second;
         m_tPhysicsModels.erase(it);
      }
      else {
         THROW_ARGOSEXCEPTION("Model id \"" << str_id << "\" not found in Iridia tracking system \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

    CEmbodiedEntity* CIridiaTrackingSystem::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                                 const CRay3& c_ray) const {
    	/*m_pcOperation->Setup(f_t_on_ray, c_ray);
		m_pcITSModelIndex->ForEntitiesAlongRay(
    	            c_ray,
    	            *m_pcOperation,
    	            false);
//		return m_pcITSModelIndex->GetFirstCollision();
		return m_pcOperation->GetFirstNonRobotCollision();*/

    	f_t_on_ray=1.0f;
    	CEmbodiedEntity* cFirstCollision = NULL;
    	SBoundingBox sBoundingBox;
    	sBoundingBox.MinCorner.SetX(Min<Real>(c_ray.GetStart().GetX(),c_ray.GetEnd().GetX()));
    	sBoundingBox.MinCorner.SetY(Min<Real>(c_ray.GetStart().GetY(),c_ray.GetEnd().GetY()));
    	sBoundingBox.MinCorner.SetZ(Min<Real>(c_ray.GetStart().GetZ(),c_ray.GetEnd().GetZ()));
    	sBoundingBox.MaxCorner.SetX(Max<Real>(c_ray.GetStart().GetX(),c_ray.GetEnd().GetX()));
    	sBoundingBox.MaxCorner.SetY(Max<Real>(c_ray.GetStart().GetY(),c_ray.GetEnd().GetY()));
    	sBoundingBox.MaxCorner.SetZ(Max<Real>(c_ray.GetStart().GetZ(),c_ray.GetEnd().GetZ()));
    	std::map<std::string, CIridiaTrackingSystemModel*>::const_iterator itPhysicsModels;
		for (itPhysicsModels = m_tPhysicsModels.begin(); itPhysicsModels != m_tPhysicsModels.end(); itPhysicsModels++) {
			// if the ray intersect bounding box of the model, potential intersection
			//LOG<<"BOUNDING BOX INTERSECT? "<< (sBoundingBox.Intersects((*itPhysicsModels).second->GetBoundingBox()))<<"\n";
			if(sBoundingBox.Intersects((*itPhysicsModels).second->GetBoundingBox()))
			{
				if ((*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() =="box"
						|| (*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "cylinder"){
					//LOG<<"BOUNDING BOX INTERSECT: "<< (*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() <<"\n";

					Real fTOnRay;
					CEmbodiedEntity* cCollision = (*itPhysicsModels).second->CheckIntersectionWithRay(fTOnRay, c_ray);
					//Delegate to the model to look for an intersection
					if (cCollision){
						if (fTOnRay < f_t_on_ray){
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

    void CIridiaTrackingSystem::CheckIntersectionWithRay(TEmbodiedEntityIntersectionData& t_data, const CRay3& c_ray) const {
      //CEmbodiedEntity* cFirstCollision = NULL; unused variable
      SBoundingBox sBoundingBox;
      sBoundingBox.MinCorner.SetX(Min<Real>(c_ray.GetStart().GetX(),c_ray.GetEnd().GetX()));
      sBoundingBox.MinCorner.SetY(Min<Real>(c_ray.GetStart().GetY(),c_ray.GetEnd().GetY()));
      sBoundingBox.MinCorner.SetZ(Min<Real>(c_ray.GetStart().GetZ(),c_ray.GetEnd().GetZ()));
      sBoundingBox.MaxCorner.SetX(Max<Real>(c_ray.GetStart().GetX(),c_ray.GetEnd().GetX()));
      sBoundingBox.MaxCorner.SetY(Max<Real>(c_ray.GetStart().GetY(),c_ray.GetEnd().GetY()));
      sBoundingBox.MaxCorner.SetZ(Max<Real>(c_ray.GetStart().GetZ(),c_ray.GetEnd().GetZ()));
      std::map<std::string, CIridiaTrackingSystemModel*>::const_iterator itPhysicsModels;
      for (itPhysicsModels = m_tPhysicsModels.begin(); itPhysicsModels != m_tPhysicsModels.end(); itPhysicsModels++) {
        if(sBoundingBox.Intersects((*itPhysicsModels).second->GetBoundingBox())) {
          if ((*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() =="box"
            || (*itPhysicsModels).second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "cylinder"){
              Real fTOnRay;
            CEmbodiedEntity* cCollision = (*itPhysicsModels).second->CheckIntersectionWithRay(fTOnRay, c_ray);
            //Delegate to the model to look for an intersection
            if (cCollision){
                t_data.push_back(SEmbodiedEntityIntersectionItem(cCollision, fTOnRay));
              }
          }
        }
      }
    }

    /****************************************/
    /****************************************/

    void CIridiaTrackingSystem::InitArenaState()
    {
        // A list of the state of all the robots declared in the XML configuration file
        std::vector<TRobotState> vecXMLDeclaredInitialArenaState;

        // For each physics model
        for(CIridiaTrackingSystemModel::TMap::iterator it = m_tPhysicsModels.begin();
            it != m_tPhysicsModels.end(); ++it)
        {
            // Does the model is a footbot or an epuck?
        	if ((it->second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "epuck")
        			|| (it->second->GetEmbodiedEntity().GetParent().GetTypeDescription() == "foot-bot")){
				// Get its Argos ID
				// The Argos ID is the ID of the robot as defined in the XML file.
				// The name must contain an arbitrary string - underscore - robot tag (ITS ID) - underscore - robot ID
				std::string strArgosId = it->first;
				// Tokenize it
				std::vector<std::string> vecTokens;
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
				UInt32 unITSId = FromString<UInt32>(vecTokens[vecTokens.size()-2]);
				//CArenaStateStruct::TRobotState tRobotState(std::make_pair<UInt32, CArenaStateStruct::SRealWorldCoordinates>
                CArenaStateStruct::TRobotState tRobotState(std::make_pair
						(unITSId, CArenaStateStruct::SRealWorldCoordinates(it->second->GetEmbodiedEntity().GetOriginAnchor().Position + m_cArenaCenter3D,
																		   it->second->GetEmbodiedEntity().GetOriginAnchor().Orientation,
																		   (UInt32)0)));

                // Convert robot ID in int form
				UInt32 unRobotId = FromString<UInt32>(vecTokens[vecTokens.size()-1]);

				// Create a pair <ITS ID, RObot ID>
				//std::pair<UInt32, UInt32> pairITSIdRobotId = std::make_pair<UInt32, UInt32>(unITSId, unRobotId);
                std::pair<UInt32, UInt32> pairITSIdRobotId = std::make_pair(unITSId, unRobotId);
				//DEBUG("push back id %s\n", strArgosId.c_str());
				// Fill the Robot ID Table
				//m_tTableRobotId->insert(std::make_pair<std::string, std::pair<UInt32, UInt32> >(strArgosId, pairITSIdRobotId));
                m_tTableRobotId->insert(std::make_pair(strArgosId, pairITSIdRobotId));
				// Fill another vector with robot tag only
				m_vecUsedRobotTagList.push_back(tRobotState.first);
				// Fill a vector with robot state
				vecXMLDeclaredInitialArenaState.push_back(tRobotState);
        	}

        }

        // Set the Initial Arena State as it is defined in the XML configuration file
        m_cArenaStateStruct.SetInitalArenaState(vecXMLDeclaredInitialArenaState);
        // If the experiment is simulated no need to wait for Tracking System information merge
        // Update the Arena State with the same state
        if (!m_bRealExperiment) {
            m_cArenaStateStruct.UpdateArenaState(vecXMLDeclaredInitialArenaState);
        }
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
