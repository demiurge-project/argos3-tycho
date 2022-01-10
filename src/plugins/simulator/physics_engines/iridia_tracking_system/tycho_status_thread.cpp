/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/argos_its_client/argos_its_client_thread.cpp>
 *
 *
 * @author Jonas Kuckling
 */

#include "tycho_status_thread.h"


namespace argos {

    CTychoStatusThread::CTychoStatusThread(CIridiaTrackingSystem *c_iridia_tracking_system):
        m_pcIridiaTrackingSystem(c_iridia_tracking_system),
        m_bExperimentIsFinished(false)
    {
    }

    /****************************************/
    /****************************************/

    CTychoStatusThread::~CTychoStatusThread()
    {
        delete m_pcIridiaTrackingSystem;
    }

    void CTychoStatusThread::Start()
    {
        LOG << "StatusThread is started" << std::endl;
        // TODO: Start the thread here
    }

    void CTychoStatusThread::Reset() {
        // TODO: Implement
    }

    void CTychoStatusThread::Run()
    {
        // wait until the experiment is finished
        while(!m_bExperimentIsFinished) {
            m_bExperimentIsFinished = m_pcIridiaTrackingSystem->IsExperimentFinished();
        }
        OnExperimentFinished();
    }

    void CTychoStatusThread::OnExperimentFinished()
    {
        LOG << "StatusThread detected that experiment finished" << std::endl;
        m_bExperimentIsFinished = true;
    }
}
