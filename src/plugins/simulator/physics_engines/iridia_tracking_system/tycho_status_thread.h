/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/tycho_status_thread.h>
 *
 * Provides the implementation of the status thread for the tycho physics engine.
 *
 * argos3 has three status changes that can be tracked:
 * - experiment started (CPhysicsEngine.Update() called for the first time or the first time after a reset)
 * - experiment reset (Reset() function called)
 * - experiment finished (CSimulator.IsExperimentFinished() returns true)
 * 
 * The physics engine can detect the first and second case, but not the third.
 * Therefore, the status thread runs in the background and checks if the experiment is finished.
 * If it detects a change in the status of argos3, it informs the tycho_tracking_system of this change.
 * 
 * @author Jonas Kuckling
 */

#ifndef TYCHO_STATUS_THREAD_H
#define TYCHO_STATUS_THREAD_H

namespace argos {
    class CIridiaTrackingSystem;
}

#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system.h>

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/string_utilities.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <pthread.h>

namespace argos {

    class CTychoStatusThread
    {
        public:
            /*
             * @brief CTychoStatusThread Constructor
             */
            CTychoStatusThread(CIridiaTrackingSystem* c_iridia_tracking_system);
            
            /*
             * Destructor
             */
            ~CTychoStatusThread();

            void Start();

            void Reset();

        private:

            void Run();

            void OnExperimentFinished();

            /* Pointer to the physics engine */
            CIridiaTrackingSystem * m_pcIridiaTrackingSystem;

            pthread_t m_cThread;

            /*
             * @brief m_bExperimentIsFinished indicates if the experiment is finished (TODO how?)
             */
            bool m_bExperimentIsFinished;
    };
}

#endif