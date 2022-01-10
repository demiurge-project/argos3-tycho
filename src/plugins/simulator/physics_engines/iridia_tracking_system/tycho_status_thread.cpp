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
        pthread_create(&m_cThread, NULL, (void * (*) (void *)) &argos::CTychoStatusThread::Run, this);
        LOG << "StatusThread is started" << std::endl;
    }

    void CTychoStatusThread::Reset() {
        m_bExperimentIsFinished = false;
        pthread_cancel(m_cThread);
        LOG << "StatusThread is reset" << std::endl;
    }

    void CTychoStatusThread::Run()
    {
        // wait until the experiment is finished
        while(!m_bExperimentIsFinished) {
            sleep(0);
            m_bExperimentIsFinished = m_pcIridiaTrackingSystem->IsExperimentFinished();
        }
        OnExperimentFinished();
    }

    void CTychoStatusThread::OnExperimentFinished()
    {
        m_bExperimentIsFinished = true;
        m_pcIridiaTrackingSystem->TerminateExperiment();
    }
}
