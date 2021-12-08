/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/argos_its_client/arena_state_struct.h>
 *
 * @brief This file provides the Arena State data structure as a singleton
 *
 *
 * @author Mattia Salvaro
 */


#ifndef ROBOTS_COORDINATES_STRUCT_H
#define ROBOTS_COORDINATES_STRUCT_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

#include <map>
#include <algorithm>
#include <memory>
// #ifndef __APPLE__
// #include <auto_ptr.h>
// #endif
namespace argos {

class CArenaStateStruct
{

public:

    /**
     * @brief The SRealWorldCoordinates struct contains the information
     * about the position of the robot in the real world and the associated
     * timestep
     */

    struct SRealWorldCoordinates {
        CVector3 cPosition;
        CQuaternion cOrientation;
        UInt32 unTimestep;

        SRealWorldCoordinates() :
            cPosition(),
            cOrientation(),
            unTimestep(0){}

        SRealWorldCoordinates(const CVector3 & c_position, const CQuaternion & c_orientation, UInt32 un_timestep) :
            cPosition(c_position),
            cOrientation(c_orientation),
            unTimestep(un_timestep) {}

        inline CVector3 GetPosition() {
            return cPosition;
        }

        inline CQuaternion GetOrientation() {
            return cOrientation;
        }

        inline UInt32 GetTimestep() {
            return unTimestep;
        }
    };

    /**
     * @brief TRobotState Defines the State of one robot
     * key: the robot tag
     * value: the State of the robot in the real world
     */
    typedef std::pair<UInt32, SRealWorldCoordinates> TRobotState;

    /**
     * @brief TArenaState Defines the State of all the robots in the Arena
     * key: the robot tag
     * value: the State of the robot in the real world
     */
    typedef std::map<UInt32, SRealWorldCoordinates> TArenaState;

public:

    // Pattern Singleton
    /**
     * @brief GetInstance The method to access the CArenaStateStruct singleton
     * @return A reference to the singleton CArenaStateStruct
     */
    static CArenaStateStruct& GetInstance()
    {
        static std::auto_ptr<CArenaStateStruct> pcArenaStateStructInstance(new CArenaStateStruct());
        return *(pcArenaStateStructInstance.get());
    }
    // -----------------

    /**
      * Destructor
      */
    ~CArenaStateStruct();

    /**
     * @brief SetInitalArenaState Saves the initial Arena State according to the Tracking System
     * without setting the state in the TArenaState
     * @param vec_initial_arena_state A reference to the initial vector of Robot State
     */
    void SetInitalArenaState(std::vector<TRobotState> & vec_initial_arena_state);

    /**
     * @brief UpdateArenaState Updates the Arena State to the given Arena State configuration
     * @param vec_t_arena_state A reference to a vector of updated Robot State
     */
    void UpdateArenaState(const std::vector<TRobotState> &vec_t_arena_state);

    /**
     * @brief ResetTimestepCounter Resets the value of the timestep
     */
    void ResetTimestepCounter();

    /**
     * @brief ResetArenaState Resets the Arena State to an empty state
     */
    void ResetArenaState();

    /**
     * @brief IsUsedTagListInit
     * @return True if the used tag list had been filled, false otherwise
     */
    bool IsUsedTagListInit();

    /**
     * @brief IsArenaStateInit
     * @return True if the initial Arena State had been set, false otherwise
     */
    bool IsArenaStateInit();

    /**
     * @brief GetRobotState Gets the Robot State given the Robot tag
     * @param t_real_world_coordinates Output param. The wanted Robot State
     * @param un_robot_tag The given Robot tag
     */
    void GetRobotState(SRealWorldCoordinates & t_real_world_coordinates, const UInt32 un_robot_tag);

    /**
     * @brief GetArenaState Gives the arena state in string form like the results file
     * @param str_arena_state Output parameter. The requested arena state
     */
    void GetStringArenaState(std::string & str_arena_state);

private:

    // Pattern Singleton

    /**
     * @brief CArenaStateStruct Private constructor for singleton
     */
    CArenaStateStruct();

    /**
     * @brief CArenaStateStruct Private copy constructor for singleton
     */
    CArenaStateStruct(CArenaStateStruct const&);

    /**
     * @brief operator = Overridden not implemented for singleton
     */
    void operator=(CArenaStateStruct const&);
    // ------------------


    /**
     * @brief InitArenaState Creates the initial TArenaState merging the
     * Arena State of the Tracking System with the definition of the XML file
     */
    void InitArenaState();

    /**
     * @brief WriteEntry Writes the given Robot state in the preparing buffer
     * of the TArenaState
     * @param t_robot_state A reference to the Robot State to write
     */
    void WriteEntry(TRobotState &t_robot_state);

    /**
     * @brief RemoveEntry Removes the Robot State with the given tag from the
     * preparing buffer of the TArenaState
     * @param un_robot_tag The tag of the Robot State to be removed
     */
    void RemoveEntry(UInt32 un_robot_tag);

    /**
     * @brief ReplaceEntry Replace the old Robot State of the robot with the
     * given tag with the given new Robot State
     * @param t_new_robot_state The new Robot State
     * @param un_old_robot_tag The tag of the Robot State to be replaced
     */
    void ReplaceEntry(TRobotState & t_new_robot_state, UInt32 un_old_robot_tag);

    /**
     * @brief EntryMatches Checks whether the given Robot State matches with any
     * of the Robots States pointes by the iterator
     * @param it_matching_robot A reference to the iterator
     * @param t_robot_state A reference to the target Robot State
     * @param f_max_distance The maximum distance required for the target to match another Robot Entry
     */
    void EntryMatches(std::vector<TRobotState>::iterator & it_arena_state, TRobotState & t_robot_state, Real f_max_distance);

    /**
     * @brief RecoverLostRobots An algorithm to catch missed robots
     */
    void RecoverLostRobots();

    /**
     * @brief UpdateMatchingEntries Updates all the "matching" Robot States to the latest state
     */
    void UpdateMatchingEntries();

    /**
     * @brief MyHeuristic Defines a heuristic for dealing with possible Tracking System mismatches
     */
    void MyHeuristic();

private:

    /**
     * @brief m_tArenaState Double buffer for the Arena State data structure
     */
    TArenaState m_tArenaState[2];

    /**
     * @brief m_ptCurrentArenaState Pointer to the current Arena State data structure
     */
    TArenaState *m_ptCurrentArenaState;

    /**
     * @brief m_ptNextArenaState Pointer to the Arena State under construction for next timestep
     */
    TArenaState *m_ptNextArenaState;

    /**
     * @brief m_unTimestep Current Arena State timestep
     */
    UInt32 m_unTimestep;

    /**
     * @brief m_vecUsedTagList List of used robot tag, to be declared in the XML file
     */
    std::vector<UInt32> m_vecUsedTagList;

    /**
     * @brief m_vecTrackingSystemState List of the Arena State result according to the Tracking System
     * to be merged with XML file information
     */
    std::vector<TRobotState> m_vecTrackingSystemState;

    /**
     * @brief m_vecMergedArenaState List of the Arena State that results as a merge of the XML
     * configuration file and the Tracking System results
     */
    std::vector<TRobotState> m_vecMergedArenaState;

    /**
     * @brief MAX_DISTANCE Hard coded maximum distance for two Robot State to match
     */
    const Real MAX_DISTANCE;

    /**
     * @brief m_cMutex To create thread safe critical sessions when dealing with TArenaState
     */
    pthread_mutex_t m_cMutex;
};

}

#endif
