/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/argos_its_client/argos_its_client_thread.cpp>
 *
 *
 * @author Mattia Salvaro
 */

#include "argos_its_client_thread.h"


namespace argos {

CArgosITSClientThread::CArgosITSClientThread(CIridiaTrackingSystem *c_iridia_tracking_system, const std::string &str_its_server_address, UInt32 un_its_server_port)
    : m_nSocketDescriptor(-1),
      m_pcIridiaTrackingSystem(c_iridia_tracking_system),
      m_cArenaStateStruct(CArenaStateStruct::GetInstance()),
      m_bSimulationActive(false),
      m_bFirstRun(true),
      m_bIsExperimentFinished(false),
      m_strITSServerAddress(str_its_server_address),
      m_unITSServerPort(un_its_server_port)
{
    /* Init command list */
    m_pcCommandOptions = new std::map<std::string, UInt32>();
    m_pcCommandOptions->insert(std::make_pair<std::string, UInt32>("START_EXPERIMENT", 0));
    m_pcCommandOptions->insert(std::make_pair<std::string, UInt32>("STOP_EXPERIMENT", 1));
    m_pcCommandOptions->insert(std::make_pair<std::string, UInt32>("ONE_SHOT", 2));
    //m_pcCommandOptions->insert(std::make_pair<std::string, UInt32>("HEARTBEAT", 3));
}

/****************************************/
/****************************************/

CArgosITSClientThread::~CArgosITSClientThread()
{
    delete m_pcIridiaTrackingSystem;
    delete m_pcCommandOptions;
}

/****************************************/
/****************************************/

void CArgosITSClientThread::Run()
{
	if (!InitClient()) {
        return;
    }
    SynchronizeClient();
    /* Life cycle of the client */
    while(!m_bIsExperimentFinished) {
        InitExperiment();
        ExecuteExperiment();
        EndExperiment();
    }
}

/****************************************/
/****************************************/

bool CArgosITSClientThread::InitClient()
{
    DEBUG ("Thread started\n");
    m_nSocketDescriptor = socket(AF_INET , SOCK_STREAM , 0);
    if (m_nSocketDescriptor == -1)
    {
        THROW_ARGOSEXCEPTION("Could not create socket\n");
    }
    m_sServer.sin_addr.s_addr = inet_addr(m_strITSServerAddress.c_str());
    DEBUG("Server address: %s\n",m_strITSServerAddress.c_str());

    m_sServer.sin_family = AF_INET;
    m_sServer.sin_port = htons( m_unITSServerPort );

    //Connect to remote server
    DEBUG("Trying to connect to the ITS server...\n");
    if (connect(m_nSocketDescriptor , (struct sockaddr *)&m_sServer , sizeof(m_sServer)) < 0)
    {
        DEBUG("[WARNING] Not connected to the Tracking System Server\n");
        return false;
    }
    else {
    	DEBUG("Connected\n");
    	return true;
    }
}

/****************************************/
/****************************************/

void CArgosITSClientThread::SynchronizeClient()
{
    while(!m_cArenaStateStruct.IsUsedTagListInit()) {}
}


/****************************************/
/****************************************/

void CArgosITSClientThread::InitExperiment()
{
    if (m_bIsExperimentFinished == true) {
        return;
    }

    // Take a fist snapshot
    SendCommand("ONE_SHOT");
	
    // Get the Arena State
    std::string strArenaState;
    ReadArenaState(strArenaState);

    // Parse it to SArenaState
    SArenaState sArenaState;
    ParseArenaState(sArenaState, strArenaState);

    /* In case it is not the first run, the Tracking System
     * will keep a buffer of pictures that must be flushed
     */
    while(sArenaState.un_timestep != 0) {
        SendCommand("ONE_SHOT");

        sArenaState.vec_s_robot_state.clear();

        ReadArenaState(strArenaState);
        ParseArenaState(sArenaState, strArenaState);
    }

    UInt32 unFlushBuffer = 0;
    while (!m_bFirstRun && unFlushBuffer <= 2) {
        SendCommand("ONE_SHOT");

        sArenaState.vec_s_robot_state.clear();
        unFlushBuffer++;
        ReadArenaState(strArenaState);
        ParseArenaState(sArenaState, strArenaState);
    }
    /* Buffer flushed
     */

    // send command REPLACE_ROBOTS
    // several ONE_SHOT could do
    // while all robots are not in initial position
    // Read arena state


    // Get the Arena State in ARGoS format
    std::vector<CArenaStateStruct::TRobotState> vecRobotState;
    ProcessResults(vecRobotState, sArenaState);

    // Update the Arena State data structure with the first snapshot
    UpdateArenaDataStruct(vecRobotState);

    // Set the step counter to 0
    m_cArenaStateStruct.ResetTimestepCounter();

    // Jump to next step
}

/****************************************/
/****************************************/

void CArgosITSClientThread::ExecuteExperiment()
{
    // Wait for the simulation to start
    while(!IsSimulationActive()) {
        if (m_bIsExperimentFinished == true) {
            return;
        }
    }

    // Send START command to the Tracking System
    SendCommand("START_EXPERIMENT");
    UInt32 unFlushBuffer = 0;

    // While the simulation is running..
    while(!m_pcIridiaTrackingSystem->IsExperimentFinished()) {
        // Get the Arena State in string form from the server
        std::string strArenaState;
        ReadArenaState(strArenaState);
        // Parse it into the SArenaState form
        SArenaState sArenaState;
        ParseArenaState(sArenaState, strArenaState);
        // Discard the first two frames as buffer leftovers
        if (!m_bFirstRun && unFlushBuffer < 2) {
            unFlushBuffer++;
            continue;
        }
        // Turn the arena State in ARGoS format
        std::vector<CArenaStateStruct::TRobotState> vecRobotState;
        ProcessResults(vecRobotState, sArenaState);

        // Update the Arena State data structure
        UpdateArenaDataStruct(vecRobotState);
    }
    // ... when simulation is over jump to next step
}

/****************************************/
/****************************************/

void CArgosITSClientThread::EndExperiment()
{
    m_pcIridiaTrackingSystem->TerminateExperiment();

    // Reset the Arena State data structure for next run
    m_cArenaStateStruct.ResetArenaState();
    m_bFirstRun = false;


    if (m_bIsExperimentFinished == true) {
        return;
    }

    // Send STOP command to the Tracking System
    SendCommand("STOP_EXPERIMENT");
    // Wait until a new run is launched ...
    while(m_pcIridiaTrackingSystem->IsExperimentFinished()) {
        if (m_bIsExperimentFinished == true) {
            return;
        }
    }

    // ... jump to next step
}

/****************************************/
/****************************************/

void CArgosITSClientThread::ReadArenaState(std::string &str_arena_state)
{
    // Receive the size of the Arena State as a 4 bytes unsignet int
    UInt32 unSizeArenaState;
    if( recv(m_nSocketDescriptor, &unSizeArenaState , sizeof(UInt32) , 0) < 0)
    {
        THROW_ARGOSEXCEPTION("recv failed\n");
    }
    // Perform network to host long transformation
    unSizeArenaState = ntohl(unSizeArenaState);

    // If the size of the Arena State is 0, terminate the experiment
    if (unSizeArenaState == 0) {
        m_pcIridiaTrackingSystem->TerminateExperiment();
    }

    // Prepare a char buffer for the Arena State in string form
    char strArenaState[unSizeArenaState + 1]; // + 1 for the string terminator \0
    UInt32 totalByteRead = 0;
    int n = 0;

    // Perfor cyclic readings untill all the bytes needed are read
    while (totalByteRead < unSizeArenaState) {
        if(( n = recv(m_nSocketDescriptor, strArenaState , unSizeArenaState - totalByteRead , 0)) < 0)
        {
            THROW_ARGOSEXCEPTION("recv failed\n");
        }
        totalByteRead = totalByteRead + n;
    }

    // Place the string terminator
    strArenaState[unSizeArenaState] = 0;
    str_arena_state = strArenaState;
}

/****************************************/
/****************************************/

void CArgosITSClientThread::ParseArenaState(SArenaState &s_arena_state, const std::string &str_arena_state)
{
    // Preapare parsing tools
    std::istringstream cStringStream(str_arena_state);
    std::string strToken;
    std::vector<std::string> vecTokens;
    // Fill a string vector
    while (getline(cStringStream, strToken, ' ')) {
        vecTokens.push_back(strToken);
    }

    // Remove the first token from the vector and assign its value to the SArenaState timestep
    s_arena_state.un_timestep = FromString<UInt32>(vecTokens[0]);
    vecTokens.erase(vecTokens.begin());

    // Create a temporary Robot State
    SRobotState sRobotState;
    // Each Robot State takes 7 consecutive tokens from the vector
    UInt32 unNumRobotEntries = vecTokens.size()/7;
    // Fill the temporary Robot State with 4 out of 7 tokens
    // Image position and orientation are not needed

    for (UInt32 unRobotState=0; unRobotState<unNumRobotEntries; unRobotState++) {

        sRobotState.un_robot_tag = FromString<UInt32>(vecTokens[unRobotState*7+0]);
        sRobotState.f_robot_rw_pos_x = FromString<Real>(vecTokens[unRobotState*7+4]);
        sRobotState.f_robot_rw_pos_y = FromString<Real>(vecTokens[unRobotState*7+5]);
        sRobotState.f_robot_rw_teta = FromString<Real>(vecTokens[unRobotState*7+6]);

        // Push back the filled Robot State in the output parameter Arena State
        s_arena_state.vec_s_robot_state.push_back(sRobotState);
    }
}

/****************************************/
/****************************************/

void CArgosITSClientThread::ProcessResults(std::vector<CArenaStateStruct::TRobotState>& vec_robot_state, const SArenaState &s_arena_state)
{
    // For each entry in the Arena State transform its numeric values into CVector3 and CQuaternion
    for(UInt32 i = 0; i<s_arena_state.vec_s_robot_state.size(); i++) {

        CVector3 cPosition(s_arena_state.vec_s_robot_state[i].f_robot_rw_pos_x, s_arena_state.vec_s_robot_state[i].f_robot_rw_pos_y, Real(0));
        CRadians cRadians;
        cRadians.FromValueInDegrees(s_arena_state.vec_s_robot_state[i].f_robot_rw_teta);
        CQuaternion cOrientation;
        cOrientation.FromEulerAngles(cRadians, CRadians(0), CRadians(0));

        // Create a new instance of SRealWorldCoordinates with the transformed values and push it back in the output paramenter
        CArenaStateStruct::SRealWorldCoordinates sRealWorldCoordinates(cPosition, cOrientation, s_arena_state.un_timestep);
        vec_robot_state.push_back(CArenaStateStruct::TRobotState(std::make_pair(s_arena_state.vec_s_robot_state[i].un_robot_tag, sRealWorldCoordinates)));

    }
}

/****************************************/
/****************************************/

void CArgosITSClientThread::UpdateArenaDataStruct(const std::vector<CArenaStateStruct::TRobotState> &vec_t_arena_state)
{
    m_cArenaStateStruct.UpdateArenaState(vec_t_arena_state);
}

/****************************************/
/****************************************/

void CArgosITSClientThread::SendCommand(const std::string &str_command)
{
    // Sends an integer which is an ID for the given command
    std::map<std::string, UInt32>::iterator itMap = m_pcCommandOptions->find(str_command);
    UInt32 nCommand;
    if (itMap == m_pcCommandOptions->end()) {
        THROW_ARGOSEXCEPTION("Command is not part of the command set\n");
    }
    else {
        nCommand = itMap->second;
    }
    UInt32 nNetworkCommand = htonl(nCommand);

    if( send(m_nSocketDescriptor , &nNetworkCommand , sizeof(UInt32) , 0) < 0)
    {
        THROW_ARGOSEXCEPTION("Send failed\n");
    }
}

/****************************************/
/****************************************/

void CArgosITSClientThread::DisconnectFromITSServer()
{
    int nShutRes = shutdown(m_nSocketDescriptor, 2);
    if (nShutRes == EBADF) {
        THROW_ARGOSEXCEPTION("Socket is a bad file descriptor\n");
    }
    if (nShutRes == ENOTSOCK) {
        THROW_ARGOSEXCEPTION("Socket is a not a socket\n");
    }
    if (nShutRes == ENOTCONN) {
        THROW_ARGOSEXCEPTION("Socket is a not connected\n");
    }
    if (nShutRes == -1) {
        THROW_ARGOSEXCEPTION("Socket shutdown fail\n");
    }
    if (nShutRes == 0) {
        DEBUG("Socket shutdown ok\n");
    }

    m_bIsExperimentFinished = true;
}

/****************************************/
/****************************************/

bool CArgosITSClientThread::OpenResultsFile(const std::string &str_result_file)
{
    m_cResultsFile.open(str_result_file.c_str());
    m_cFiltredResults.open("filt_results.txt");

    if (m_cResultsFile.is_open() && m_cFiltredResults.is_open()) {
        DEBUG("Results file opened\n");
        return true;
    }
    DEBUG("Results file NOT opened\n");
    return false;
}

/****************************************/
/****************************************/

bool CArgosITSClientThread::GetNextArenaStateFromResultsFile()
{
    std::string strArenaState;

    if (!m_cResultsFile.eof()) {

        std::getline(m_cResultsFile, strArenaState);

        if (strArenaState.empty()) {
            m_cResultsFile.close();
            m_cFiltredResults.close();
            return false;
        }

        // Parse it to SArenaState
        SArenaState sArenaState;
        ParseArenaState(sArenaState, strArenaState);

        // Turn the arena State in ARGoS format
        std::vector<CArenaStateStruct::TRobotState> vecRobotState;
        ProcessResults(vecRobotState, sArenaState);

        // Update the Arena State data structure
        UpdateArenaDataStruct(vecRobotState);
        return true;
    }
    m_cResultsFile.close();
    m_cFiltredResults.close();
    return false;

}

/****************************************/
/****************************************/

void CArgosITSClientThread::WriteFilteredArenaState()
{
    std::string strFilteredArenaState;
    m_cArenaStateStruct.GetStringArenaState(strFilteredArenaState);
    m_cFiltredResults << strFilteredArenaState;
}

/****************************************/
/****************************************/

void CArgosITSClientThread::TriggerTrakingSystem()
{
    m_bSimulationActive = !m_bSimulationActive;
}

/****************************************/
/****************************************/

bool CArgosITSClientThread::IsSimulationActive() {
    return m_bSimulationActive;
}

/****************************************/
/****************************************/

} //namespace
