/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/argos_its_client/arena_state_struct.h>
 *
 * @brief This file provides the ARGoS physics engine client for the Iridia Tracking System
 *
 *
 * @author Mattia Salvaro
 */


#ifndef ARGOS_ITS_CLIENT_THREAD_H
#define ARGOS_ITS_CLIENT_THREAD_H

namespace argos {
    class CIridiaTrackingSystem;
}

#include <arpa/inet.h>

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/string_utilities.h>

#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/argos_its_client/arena_state_struct.h>
#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system.h>



namespace argos {

class CArgosITSClientThread
{

private:

    // Robot State data structure
    /**
     * @brief The SRobotState struct defines the Robot State in terms of Robot tag,
     * 2D position in the real world (z is supposed to be 0) and orientation in the
     * real world
     */
    struct SRobotState {
        UInt32 un_robot_tag;
        Real f_robot_rw_pos_x;
        Real f_robot_rw_pos_y;
        Real f_robot_rw_teta;
    };

    // Arena State data structure
    /**
     * @brief The SArenaState struct defines the Arena State at a certain timestep
     */
    struct SArenaState {
        UInt32 un_timestep;
        std::vector<SRobotState> vec_s_robot_state;
    };


public:

    /**
     * @brief CArgosITSClientThread Constructor
     * @param c_iridia_tracking_system A pointer to the physics engine from which the thread is spawned
     * @param str_its_server_address Iridia Treacking System server address
     * @param un_its_server_port Iridia Tracking System server port
     */
    //CArgosITSClientThread(CIridiaTrackingSystem *c_iridia_tracking_system, std::string str_its_server_address, UInt32 un_its_server_port);
    CArgosITSClientThread(CIridiaTrackingSystem *c_iridia_tracking_system, const std::string & str_its_server_address, UInt32 un_its_server_port);

    /**
      * Destructor
      */
    ~CArgosITSClientThread();

    /**
     * @brief Run The method that is used to start the thread
     */
    void Run();

    /**
     * @brief TriggerTrakingSystem Triggers a boolean variable to control the execution of the thread
     */
    void TriggerTrakingSystem();

    /**
     * @brief DisconnectFromITSServer Close the socket with the ITS Server
     */
    void DisconnectFromITSServer();

    //void Reset();

    /**
     * @brief OpenResultsFile Opens the results file if available
     * @param str_result_file The given path to the results file
     * Returns true if the file is successfully opened, false otherwise
     */
    bool OpenResultsFile(const std::string& str_result_file);

    /**
     * @brief GetNextArenaStateFromResultsFile Retrieves the next line of the results file
     */
    bool GetNextArenaStateFromResultsFile();

    /**
     * @brief WriteFilteredArenaState Writes in a file the arena state after filtering
     */
    void WriteFilteredArenaState();




private:

    /**
     * @brief InitClient Opens socket connection with server
     */
    bool InitClient();

    /**
     * @brief SynchronizeClient Waits until Arena State is set
     */
    void SynchronizeClient();

    /**
     * @brief InitExperiment Prepares the simulator to start the experiment
     */
    void InitExperiment();

    /**
     * @brief ExecuteExperiment The body of the client. Starts the Tracking System
     * and cycles until the experiment is running
     */
    void ExecuteExperiment();

    /**
     * @brief EndExperiment Stops the Tracking System
     */
    void EndExperiment();

    /**
     * @brief SendCommand Communicates with the server
     * @param str_command The string that represent a command for the server
     */
    void SendCommand(const std::string & str_command);

    /**
     * @brief IsSimulationActive
     * @return True if the simulation is triggered on active, false otherwise
     */
    bool IsSimulationActive();

    /**
     * @brief ReadArenaState Reads the Arena State from the server
     * @param str_arena_state Output parameter: the received Arena State in string form
     */
    void ReadArenaState(std::string & str_arena_state);

    /**
     * @brief ParseArenaState Parses the Arena State from string form to SArenaState form
     * @param s_arena_state Output parameter: the wanted parsed Arena State
     * @param str_arena_state The Arena State in string form
     */
    void ParseArenaState(SArenaState & s_arena_state, const std::string & str_arena_state);

    /**
     * @brief ProcessResults Turns the Robot States to the ARGoS compatible format
     * @param vec_robot_state Output parameter: the wanted Robot States in ARGoS format
     * @param s_arena_state The collection of Robot States in Iridia Tracking System format
     */
    void ProcessResults(std::vector<CArenaStateStruct::TRobotState> & vec_robot_state, const SArenaState & s_arena_state);

    /**
     * @brief UpdateArenaDataStruct Updates the Arena State data struct
     * @param vec_t_robot_state The last Arena State available
     */
    void UpdateArenaDataStruct(const std::vector<CArenaStateStruct::TRobotState> &vec_t_robot_state);



    //void ReadHeartbeatAnswer();

private:

    /* Socket */
    SInt16 m_nSocketDescriptor;
    struct sockaddr_in m_sServer;

    /* Pointer to the physics engine */
    CIridiaTrackingSystem * m_pcIridiaTrackingSystem;
    /* Reference to the Arena State date struct singleton */
    CArenaStateStruct & m_cArenaStateStruct;

    /* Flags */
    bool m_bSimulationActive;
    bool m_bFirstRun;
    bool m_bIsExperimentFinished;

    /* List of possible commands to send to the server */
    std::map<std::string, UInt32> * m_pcCommandOptions;

    /* Address and port of the Iridia Tracking System server */
    std::string m_strITSServerAddress;
    UInt32 m_unITSServerPort;

    std::ifstream m_cResultsFile;
    std::ofstream m_cFiltredResults;
};
} // namespace

#endif
