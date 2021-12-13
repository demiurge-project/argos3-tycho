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
        m_vecUsedTagList()
    {
        m_tArenaState = std::map<UInt32, SRealWorldCoordinates>();
    }


    /****************************************/
    /****************************************/

    CArenaStateStruct::~CArenaStateStruct()
    {
        m_tArenaState.clear();
        m_vecUsedTagList.clear();
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
            SetRobotState((*itArenaState));
        }
        //DEBUG_FUNCTION_EXIT;
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::SetRobotState(TRobotState &t_robot_state)
    {
        // m_tArenaState.insert(t_robot_state);
        m_tArenaState[t_robot_state.first] = t_robot_state.second;
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
        return !m_tArenaState.empty();
    }

    /****************************************/
    /****************************************/

    void CArenaStateStruct::GetRobotState(SRealWorldCoordinates & t_real_world_coordinates, UInt32 un_robot_tag)
    {
        //t_real_world_coordinates = *(new SRealWorldCoordinates());
        //return;


        TArenaState::iterator itArenaState;

        if(m_tArenaState.empty()) {
            THROW_ARGOSEXCEPTION("Current Arena State is empty\n");
        }
        itArenaState = m_tArenaState.find(un_robot_tag);

        // Set the output parameter
        if (itArenaState != m_tArenaState.end()) {
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
        for (itArenaState = m_tArenaState.begin(); itArenaState != m_tArenaState.end(); itArenaState++)
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
