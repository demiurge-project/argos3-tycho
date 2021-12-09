/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/argos_its_client/arena_state_struct.cpp>
 *
 *
 * @author Mattia Salvaro
 */

#include "arena_state_struct.h"


namespace argos {


    /****************************************/
    /****************************************/

    CArenaStateStruct::CArenaStateStruct() :
        m_unTimestep(0),
        m_vecUsedTagList(),
        m_vecTrackingSystemState(),
        m_vecMergedArenaState(),
        MAX_DISTANCE(0.15)
    {
        pthread_mutex_init(&m_cMutex, NULL);

        m_tArenaState[0] = std::map<UInt32, SRealWorldCoordinates>();
        m_tArenaState[1] = std::map<UInt32, SRealWorldCoordinates>();

        m_ptNextArenaState = &m_tArenaState[0];
        m_ptCurrentArenaState = &m_tArenaState[1];
    }


    /****************************************/
    /****************************************/

    CArenaStateStruct::~CArenaStateStruct()
    {
        m_tArenaState[0].clear();
        m_tArenaState[1].clear();

        //delete m_ptCurrentArenaState;
        //delete m_ptNextArenaState;

        m_vecUsedTagList.clear();
        m_vecTrackingSystemState.clear();
        m_vecMergedArenaState.clear();

        pthread_mutex_destroy(&m_cMutex);
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::SetInitalArenaState(std::vector<TRobotState> &vec_initial_arena_state)
    {
        //DEBUG_FUNCTION_ENTER;
        // At the beginning the Merged Arena State is equal to the XML declared Arena State
        for (std::vector<TRobotState>::iterator itArenaState = vec_initial_arena_state.begin(); itArenaState != vec_initial_arena_state.end(); ++itArenaState)
        {
            m_vecUsedTagList.push_back((*itArenaState).first);
            m_vecMergedArenaState.push_back(*itArenaState);
        }
        //DEBUG_FUNCTION_EXIT;
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::UpdateArenaState(const std::vector<TRobotState> &vec_t_arena_state)
    {
        //DEBUG_FUNCTION_ENTER;

        if (m_vecUsedTagList.empty()) {
            THROW_ARGOSEXCEPTION("Used Tag List not initialized\n");
        }

        m_unTimestep ++;

        // At step 0
        if (m_ptCurrentArenaState->empty()) {
            // Merge m_vecXMLDeclaredState and vec_t_arena_state
            for (UInt8 unCounterTrackingSystemEntries = 0; unCounterTrackingSystemEntries<m_vecMergedArenaState.size(); unCounterTrackingSystemEntries++)
            {
                for (UInt8 unCounterArenaState = 0; unCounterArenaState<vec_t_arena_state.size(); unCounterArenaState++)
                {
                    if (m_vecMergedArenaState[unCounterTrackingSystemEntries].first == vec_t_arena_state[unCounterArenaState].first) {
                        m_vecMergedArenaState[unCounterTrackingSystemEntries].second = vec_t_arena_state[unCounterArenaState].second;
                    }
                }
            }
            InitArenaState();
        }
        /* A BIT MORE COMPLEX HEURISTIC */
        // At step n
        else {
            m_vecTrackingSystemState.clear();
            m_vecTrackingSystemState = vec_t_arena_state;
            /* ANY ARBITRARY HEURISTIC CAN BE PLACED HERE */
            MyHeuristic();
        }

        //DEBUG("Updated Arena State: %d\n", m_unTimestep);
        for(TArenaState::iterator itArenaState = m_ptNextArenaState->begin();
            itArenaState != m_ptNextArenaState->end(); ++itArenaState)
        {
            //DEBUG("%d\t%f\t%f\t%d\n", (*itArenaState).first, (*itArenaState).second.GetPosition().GetX(),
                   //(*itArenaState).second.GetPosition().GetY(), (*itArenaState).second.GetTimestep());
        }

        /* Critical session SWAP THE POINTERS */
        // Set the current pointer to the update structure
        pthread_mutex_lock(&m_cMutex);
        m_ptCurrentArenaState = m_ptNextArenaState;
        pthread_mutex_unlock(&m_cMutex);
        /* End critical session */
        //m_unTimestep ++;
        // Set the next pointer to the outdated structure
        m_ptNextArenaState = &m_tArenaState[m_unTimestep%2];
        // Update the outdated structure to the updated structure because the updating is incremental
        (*m_ptNextArenaState) = (*m_ptCurrentArenaState);
        //DEBUG_FUNCTION_EXIT;
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::MyHeuristic()
    {
        // Update matching entries
        UpdateMatchingEntries();
        // Recover Lost Robots
        RecoverLostRobots();
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::InitArenaState()
    {
        //DEBUG_FUNCTION_ENTER;

        // For each entry in the received string
        for(std::vector<TRobotState>::iterator itRobotEntries = m_vecMergedArenaState.begin();
            itRobotEntries != m_vecMergedArenaState.end(); ++itRobotEntries)
        {
            // If the id belongs to the Used Id List
            if (std::find(m_vecUsedTagList.begin(), m_vecUsedTagList.end(), (*itRobotEntries).first) != m_vecUsedTagList.end()) {
                // Assign the position to the id
                WriteEntry(*itRobotEntries);
            }
        }

        //DEBUG_FUNCTION_EXIT;
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::RecoverLostRobots()
    {
        //DEBUG_FUNCTION_ENTER;

        // <Distance, <Old Robot ID, New Robot ID> >
        std::multimap<Real, std::pair<UInt32, UInt32> > cRecoverMap;
        for(TArenaState::iterator itArenaState = m_ptNextArenaState->begin();
            itArenaState != m_ptNextArenaState->end(); ++itArenaState)
        {
            // If the entry is outdated (i.e. the robot is Lost)
            if ((*itArenaState).second.GetTimestep() < m_unTimestep) {
                //DEBUG("Lost Robot: %d %d %d\n", (*itArenaState).first, (*itArenaState).second.GetTimestep(), m_unTimestep);

                std::vector<TRobotState>::iterator itFoundRobot;
                //std::vector<TRobotState> vecEntriesToBeDeleted;

                // A robot entry is outdated for 3 reasons:
                // 1 - The tag is misread
                // 2 - The robot walked over MAX_DISTANCE
                // If the Lost Robot is found in the new robot entries
                // 3 - The combination of 1 and 2

                // Solutions:
                // 1 - Look in the neighborhood of each robot and assign to each old robot the closest misread robot
                //Real fMinDistance = MAX_DISTANCE;
                std::vector<TRobotState>::iterator itClosestRobot = m_vecTrackingSystemState.end();
                for ((itFoundRobot = m_vecTrackingSystemState.begin()); itFoundRobot != m_vecTrackingSystemState.end(); ++itFoundRobot) {

                    Real fDistance = Distance(CVector2((*itArenaState).second.GetPosition().GetX(), (*itArenaState).second.GetPosition().GetY()),
                                              CVector2((*itFoundRobot).second.GetPosition().GetX(), (*itFoundRobot).second.GetPosition().GetY()));
                    /* If the distance is the smallest so far assign the found robot as closest robot
                    if (fDistance < fMinDistance) {
                        fMinDistance = fDistance;
                        itClosestRobot = itFoundRobot;
                    }
                    */
                    // It the distance is within the neighbourhood, add the entry in the map
                    if (fDistance < MAX_DISTANCE) {
                        std::pair<UInt32, UInt32> cRobotCouple = std::make_pair((*itArenaState).first, (*itFoundRobot).first);
                        cRecoverMap.insert(std::make_pair(fDistance, cRobotCouple));
                    }

                }
                /* If closest robot exists, then assign it to the lost robot
                if (itClosestRobot != m_vecTrackingSystemState.end()) {
                    (*itArenaState).second = (*itClosestRobot).second;
                    // Remove entry from new robot list
                    m_vecTrackingSystemState.erase(itClosestRobot);
                    continue;
                }
                */
            }
        }
        // Sort the map by distance
        // The map is already sorted by key (distance)
        // Starting from smaller distance, assing the closer couples (old robot - new robot)
        // ...and delete all couples with old robot AND all the couples with new robot.

        // <Old Robot ID, <New Robot Id, Distance> >
        std::map<UInt32, std::pair<UInt32, Real> > cCoupledRobots;
        std::multimap<Real, std::pair<UInt32, UInt32> >::iterator itRecoverMap;
        for (itRecoverMap = cRecoverMap.begin(); itRecoverMap != cRecoverMap.end(); itRecoverMap++)
        {
            std::multimap<Real, std::pair<UInt32, UInt32> >::iterator itDistance;
            Real fDistance = (*itRecoverMap).first;
            for (itDistance = cRecoverMap.lower_bound(fDistance); itDistance != cRecoverMap.upper_bound(fDistance); itDistance++)
            {
                UInt32 unOldRobotID = (*itDistance).second.first;
                UInt32 unNewRobotID = (*itDistance).second.second;

                // If the old robot is not coupled yet
                std::map<UInt32, std::pair<UInt32, Real> >::iterator itCoupledRobot;
                itCoupledRobot = cCoupledRobots.find(unOldRobotID);
                if (itCoupledRobot == cCoupledRobots.end()) {
                    // ... and the new robot is not coupled to a closer old robot
                    for(itCoupledRobot = cCoupledRobots.begin(); itCoupledRobot != cCoupledRobots.end(); itCoupledRobot++)
                    {
                        if ((*itCoupledRobot).second.first == unNewRobotID) {
                            break;
                        }
                    }
                    // Add the couple to the coupled robot map
                    if (itCoupledRobot == cCoupledRobots.end()) {
                        std::pair<UInt32, Real> cNewIdDistance = std::make_pair((*itDistance).second.second, fDistance);
                        cCoupledRobots.insert(std::make_pair((*itDistance).second.first, cNewIdDistance));
                    }
                }
            }
        }

        // Now coupled robots map contains for each old robot the closest new robot.
        for(TArenaState::iterator itArenaState = m_ptNextArenaState->begin();
            itArenaState != m_ptNextArenaState->end(); ++itArenaState)
        {
            // If the entry is outdated (i.e. the robot is Lost)
            if ((*itArenaState).second.GetTimestep() < m_unTimestep) {
                // If the Old robot is found in the coupled robots map
                std::map<UInt32, std::pair<UInt32, Real> >::iterator itCoupledRobot;
                itCoupledRobot = cCoupledRobots.find((*itArenaState).first);
                if (itCoupledRobot != cCoupledRobots.end()) {
                    // Set to the old robot the state of the corresponding new robot
                    std::vector<TRobotState>::iterator itFoundRobot;
                    for ((itFoundRobot = m_vecTrackingSystemState.begin()); itFoundRobot != m_vecTrackingSystemState.end();)
                    {
                        if ((*itFoundRobot).first == (*itCoupledRobot).second.first) {
                            /*
                             *which foundrobot??
                             * the one with the same distance stored in couple robot map
                             */

                            Real fDistance = Distance(CVector2((*itArenaState).second.GetPosition().GetX(), (*itArenaState).second.GetPosition().GetY()),
                                                      CVector2((*itFoundRobot).second.GetPosition().GetX(), (*itFoundRobot).second.GetPosition().GetY()));

                            if (fDistance == (*itCoupledRobot).second.second) {
                                /*
                                DEBUG("Entry matches: %d %f %f %d - %d %f %f %d - D: %f\n", (*itArenaState).first,
                                      (*itArenaState).second.GetPosition().GetX(), (*itArenaState).second.GetPosition().GetY(),
                                      (*itArenaState).second.GetTimestep(),
                                      (*itFoundRobot).first, (*itFoundRobot).second.GetPosition().GetX(),
                                      (*itFoundRobot).second.GetPosition().GetY(), (*itFoundRobot).second.GetTimestep(),
                                      fDistance);
                                      */
                                (*itArenaState).second = (*itFoundRobot).second;
                                m_vecTrackingSystemState.erase(itFoundRobot);
                            }
                            else {
                                itFoundRobot++;
                            }
                        }
                        else {
                            itFoundRobot++;
                        }
                    }
                }
            }
        }
        DEBUG("Unplaced robots STEP 2: %d\n",m_vecTrackingSystemState.size());
        /*
        for (UInt8 i=0; i<m_vecTrackingSystemState.size(); i++)
        {
            DEBUG("%d\t%f\t%f\t%d\n", m_vecTrackingSystemState[i].first,
                  m_vecTrackingSystemState[i].second.GetPosition().GetX(),
                  m_vecTrackingSystemState[i].second.GetPosition().GetY(),
                  m_vecTrackingSystemState[i].second.GetTimestep());
        }
        */

        // 2 - If no one claimed it (e.g. it is still in the list), then take the robot with the same id, wherever it is
        for(TArenaState::iterator itArenaState = m_ptNextArenaState->begin();
            itArenaState != m_ptNextArenaState->end(); ++itArenaState)
        {
            // If the entry is outdated (i.e. the robot is Lost)
            if ((*itArenaState).second.GetTimestep() < m_unTimestep) {
                std::vector<TRobotState>::iterator itFoundRobot;
                for ((itFoundRobot = m_vecTrackingSystemState.begin()); itFoundRobot != m_vecTrackingSystemState.end(); ++itFoundRobot) {

                    if ((*itFoundRobot).first == (*itArenaState).first) {
                        // We can check if it is within a max distance (MAX_DISTANCE * diff between time steps)
                        Real fDistance = Distance(CVector2((*itArenaState).second.GetPosition().GetX(), (*itArenaState).second.GetPosition().GetY()),
                                                  CVector2((*itFoundRobot).second.GetPosition().GetX(), (*itFoundRobot).second.GetPosition().GetY()));
                        Real fMaxDistance;
                        if ((*itFoundRobot).second.GetTimestep() - (*itArenaState).second.GetTimestep() == 0) {
                            fMaxDistance = 10000;
                        }
                        else {
                            fMaxDistance = MAX_DISTANCE * ((*itFoundRobot).second.GetTimestep() - (*itArenaState).second.GetTimestep());
                        }
                        if (fDistance < fMaxDistance) {
                            /*
                            DEBUG("Entry matches: %d %f %f %d - %d %f %f %d\n", (*itArenaState).first,
                                  (*itArenaState).second.GetPosition().GetX(), (*itArenaState).second.GetPosition().GetY(),
                                  (*itArenaState).second.GetTimestep(),
                                  (*itFoundRobot).first, (*itFoundRobot).second.GetPosition().GetX(),
                                  (*itFoundRobot).second.GetPosition().GetY(), (*itFoundRobot).second.GetTimestep());
                                  */
                            // Update its position
                            (*itArenaState).second = (*itFoundRobot).second;
                            // Remove entry from new robot list
                            m_vecTrackingSystemState.erase(itFoundRobot);
                            // break...
                            break;
                        }
                    }
                }

                // 3 - DO NOT MOVE
            }
        }
        DEBUG("Unplaced robots STEP 3: %d\n",m_vecTrackingSystemState.size());
        /*
        for (UInt8 i=0; i<m_vecTrackingSystemState.size(); i++)
        {
            DEBUG("%d\t%f\t%f\t%d\n", m_vecTrackingSystemState[i].first,
                  m_vecTrackingSystemState[i].second.GetPosition().GetX(),
                  m_vecTrackingSystemState[i].second.GetPosition().GetY(),
                  m_vecTrackingSystemState[i].second.GetTimestep());
        }
        DEBUG("\n");
        */

        //DEBUG_FUNCTION_EXIT;
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::UpdateMatchingEntries()
    {
        //DEBUG_FUNCTION_ENTER;
        for(TArenaState::iterator itArenaState = m_ptNextArenaState->begin();
            itArenaState != m_ptNextArenaState->end(); ++itArenaState)
        {
            //DEBUG("Next Arena State size = %d\n", m_ptNextArenaState->size());
            std::vector<TRobotState>::iterator itMatchingRobot;
            // If the old entry matches one of the new data entries (e.g. same ID, within a given range)
            TRobotState tRobotState = *itArenaState;
            EntryMatches(itMatchingRobot, tRobotState, MAX_DISTANCE);
            if (itMatchingRobot != m_vecTrackingSystemState.end()) {

                /*
                DEBUG("match:\t\t%d %f %f %d - %d %f %f %d\n", (*itArenaState).first,
                       (*itArenaState).second.GetPosition().GetX(), (*itArenaState).second.GetPosition().GetY(), (*itArenaState).second.GetTimestep(),
                       (*itMatchingRobot).first, (*itMatchingRobot).second.GetPosition().GetX(), (*itMatchingRobot).second.GetPosition().GetY(),
                       (*itMatchingRobot).second.GetTimestep());
                       */

                // Update its position
                (*itArenaState).second = (*itMatchingRobot).second;


                // Remove the entry from the vector
                //DEBUG("Remove robot from new robot entries to be processed\n");
                m_vecTrackingSystemState.erase(itMatchingRobot);
                //vecEntriesToBeDeleted.push_back(*itMatchingRobot);
            }
            /*
            else {
                DEBUG("not match:\t%d %f %f %d\n", (*itArenaState).first,
                      (*itArenaState).second.GetPosition().GetX(), (*itArenaState).second.GetPosition().GetY(),
                       (*itArenaState).second.GetTimestep());
            }
            */
        }
        DEBUG("Unplaced robots STEP 1: %d\n",m_vecTrackingSystemState.size());
        /*
        for (UInt8 i=0; i<m_vecTrackingSystemState.size(); i++)
        {
            DEBUG("%d\t%f\t%f\t%d\n", m_vecTrackingSystemState[i].first,
                  m_vecTrackingSystemState[i].second.GetPosition().GetX(),
                  m_vecTrackingSystemState[i].second.GetPosition().GetY(),
                  m_vecTrackingSystemState[i].second.GetTimestep());
        }
        */
        //DEBUG_FUNCTION_EXIT;
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::EntryMatches(std::vector<TRobotState>::iterator &it_arena_state, TRobotState &t_robot_state, Real f_max_distance)
    {
        std::vector<TRobotState>::iterator itMatchingRobot = m_vecTrackingSystemState.end();
        Real fMinDistance = f_max_distance;
        // If the Robot Id exists in the new entry vector
        for (it_arena_state = m_vecTrackingSystemState.begin(); it_arena_state != m_vecTrackingSystemState.end(); ++it_arena_state) {
            if ((*it_arena_state).first == t_robot_state.first) {
                Real fDistance = Distance(CVector2(t_robot_state.second.GetPosition().GetX(), t_robot_state.second.GetPosition().GetY()),
                                          CVector2((*it_arena_state).second.GetPosition().GetX(), (*it_arena_state).second.GetPosition().GetY()));
                // If the distance is within a maximum value
                if (fDistance < fMinDistance) {
                    // Save the closest
                    fMinDistance = fDistance;
                    itMatchingRobot = it_arena_state;
                }
                else {
                    //DEBUG("Entry does not match: %d %f %f - %d %f %f\n", (*it_arena_state).first,
                    //       (*it_arena_state).second.GetPosition().GetX(), (*it_arena_state).second.GetPosition().GetY(),
                    //       t_robot_state.first, t_robot_state.second.GetPosition().GetX(), t_robot_state.second.GetPosition().GetY());
                }
            }
        }
        // Set the output param to the saved closest robot
        if (itMatchingRobot != m_vecTrackingSystemState.end()) {
            it_arena_state = itMatchingRobot;
        }

    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::WriteEntry(TRobotState &t_robot_state)
    {
        std::pair<TArenaState::iterator, bool> cPair;
        if (!(cPair = m_ptNextArenaState->insert(t_robot_state)).second) {
            m_ptNextArenaState->erase(cPair.first);
            m_ptNextArenaState->insert(t_robot_state);
        }
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::RemoveEntry(UInt32 un_robot_tag)
    {
        TArenaState::iterator itArenaState = m_ptNextArenaState->find(un_robot_tag);
        m_ptNextArenaState->erase(itArenaState);
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::ReplaceEntry(TRobotState &t_new_robot_state, UInt32 un_old_robot_tag)
    {
        // If new robot entry does not exist
        TArenaState::iterator itArenaState = m_ptNextArenaState->find(t_new_robot_state.first);
        if (itArenaState == m_ptNextArenaState->end()) {
            // Replace old entry with new entry
            RemoveEntry(un_old_robot_tag);
            WriteEntry(t_new_robot_state);
        }
        // Else update the old entry with new values
        else {
            //TRobotState tRobotState = std::make_pair<UInt32, SRealWorldCoordinates>(un_old_robot_tag, t_new_robot_state.second);
            TRobotState tRobotState = std::make_pair(un_old_robot_tag, t_new_robot_state.second);
            WriteEntry(tRobotState);
        }
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::ResetTimestepCounter()
    {
        m_unTimestep = 0;
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::ResetArenaState()
    {
        m_ptNextArenaState = &m_tArenaState[0];
        //m_ptCurrentArenaState->clear();
    }

    /****************************************/
    /****************************************/

    bool CArenaStateStruct::IsUsedTagListInit()
    {
        return !m_vecUsedTagList.empty();
    }

    /****************************************/
    /****************************************/

    bool CArenaStateStruct::IsArenaStateInit()
    {
        //DEBUG_FUNCTION_ENTER;
        //DEBUG_FUNCTION_EXIT;
        return !m_ptCurrentArenaState->empty();
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::GetRobotState(SRealWorldCoordinates & t_real_world_coordinates, UInt32 un_robot_tag)
    {
        t_real_world_coordinates = *(new SRealWorldCoordinates());
        return;


        TArenaState::iterator itArenaState;

        // Critical session
        pthread_mutex_lock(&m_cMutex);
        if(m_ptCurrentArenaState->empty()) {
            THROW_ARGOSEXCEPTION("Current Arena State is empty\n");
        }
        itArenaState = m_ptCurrentArenaState->find(un_robot_tag);
        pthread_mutex_unlock(&m_cMutex);
        // End critical session

        // Set the output parameter
        if (itArenaState != m_ptCurrentArenaState->end()) {
            t_real_world_coordinates = (*itArenaState).second;
        }
        else {
            THROW_ARGOSEXCEPTION("ERROR: robot not found in arena state\n");
        }

    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::GetStringArenaState(std::string & str_arena_state)
    {
        str_arena_state.clear();
        str_arena_state = ToString<UInt32>(m_unTimestep);
        TArenaState::iterator itArenaState;
        for (itArenaState = m_ptCurrentArenaState->begin(); itArenaState != m_ptCurrentArenaState->end(); itArenaState++)
        {
            CRadians cOrientationZ;
            CRadians cOrientationY;
            CRadians cOrientationX;
            (*itArenaState).second.GetOrientation().ToEulerAngles(cOrientationZ, cOrientationY, cOrientationX);
            CDegrees cDegrees;
            cDegrees.FromValueInRadians(cOrientationZ.GetValue());


            str_arena_state = str_arena_state + " " + ToString<UInt32>((*itArenaState).first) +
                    " " + ToString<Real>(Real(0)) + " " + ToString<Real>(Real(0)) + " " + ToString<Real>(Real(0)) +
                    " " + ToString<Real>((*itArenaState).second.GetPosition().GetX()) +
                    " " + ToString<Real>((*itArenaState).second.GetPosition().GetY()) +
                    " " + ToString<Real>(cDegrees.GetValue());
        }
        str_arena_state = str_arena_state + "\n";
    }

    /****************************************/
    /****************************************/

}
