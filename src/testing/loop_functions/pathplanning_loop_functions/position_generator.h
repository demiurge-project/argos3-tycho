/*
 * position_generator.h
 *
 *  Created on: 1 août 2014
 *      Author: bernard
 */

#ifndef POSITION_GENERATOR_H_
#define POSITION_GENERATOR_H_

#include <argos3/demiurge/loop-functions/CoreLoopFunctions.h>
#include <argos3/demiurge/loop-functions/RVRCoreLoopFunctions.h>
#include <argos3/core/simulator/loop_functions.h>

/* Real number generator, copied from argos3/core/simulator/space/space.cpp */
class RealNumberGenerator {
   public:
      virtual ~RealNumberGenerator() {}
      virtual CVector3 operator()(bool b_is_retry) = 0;
   };

   class ConstantGenerator : public RealNumberGenerator {
   public:
      ConstantGenerator(const CVector3& c_value) :
         m_cValue(c_value) {}

      inline virtual CVector3 operator()(bool b_is_retry) {
         return m_cValue;
      }
   private:
      CVector3 m_cValue;

   };

   class UniformGenerator : public RealNumberGenerator {
   public:
      UniformGenerator(const CVector3& c_min,
                       const CVector3& c_max) :
         m_cMin(c_min),
         m_cMax(c_max) {}
      inline virtual CVector3 operator()(bool b_is_retry) {
         Real fRandX =
            m_cMax.GetX() > m_cMin.GetX() ?
            CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetX(), m_cMax.GetX())) :
            m_cMax.GetX();
         Real fRandY =
            m_cMax.GetY() > m_cMin.GetY() ?
            CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetY(), m_cMax.GetY())) :
            m_cMax.GetY();
         Real fRandZ =
            m_cMax.GetZ() > m_cMin.GetZ() ?
            CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetZ(), m_cMax.GetZ())) :
            m_cMax.GetZ();
         return CVector3(fRandX, fRandY, fRandZ);
      }
   private:
      CVector3 m_cMin;
      CVector3 m_cMax;
   };

   class CircularUniformGenerator : public RealNumberGenerator {
      public:
         CircularUniformGenerator(const CVector3& c_center,
                          const Real& c_radius) :
            m_cCenter(c_center),
            m_fRadius(c_radius) {}
         inline virtual CVector3 operator()(bool b_is_retry) {
        	Real fRadius = CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(0, m_fRadius));
            CVector2 cPos = CVector2(fRadius, 0).Rotate(CSimulator::GetInstance().GetRNG()->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI)));
            return CVector3(m_cCenter.GetX()+cPos.GetX(), m_cCenter.GetY()+cPos.GetY(), m_cCenter.GetZ());
         }
   private:
       CVector3 m_cCenter;
       Real m_fRadius;
   };

   class GaussianGenerator : public RealNumberGenerator {
   public:
      GaussianGenerator(const CVector3& c_mean,
                        const CVector3& c_std_dev) :
         m_cMean(c_mean),
         m_cStdDev(c_std_dev) {}
      inline virtual CVector3 operator()(bool b_is_retry) {
         return CVector3(CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetX(), m_cMean.GetX()),
                         CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetY(), m_cMean.GetY()),
                         CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetZ(), m_cMean.GetZ()));
      }
   private:
      CVector3 m_cMean;
      CVector3 m_cStdDev;
   };

   class GridGenerator : public RealNumberGenerator {
   public:
      GridGenerator(const CVector3 c_center,
                    const UInt32 un_layout[],
                    const CVector3 c_distances):
         m_cCenter(c_center),
         m_cDistances(c_distances),
         m_unNumEntityPlaced(0) {
         m_unLayout[0] = un_layout[0];
         m_unLayout[1] = un_layout[1];
         m_unLayout[2] = un_layout[2];
         /* Check if layout is sane */
         if( m_unLayout[0] == 0 || m_unLayout[1] == 0 || m_unLayout[2] == 0 ) {
            THROW_ARGOSEXCEPTION("'layout' values (distribute position, method 'grid') must all be different than 0");
         }
      }

      virtual CVector3 operator()(bool b_is_retry) {
         if(b_is_retry) {
            THROW_ARGOSEXCEPTION("Impossible to place entity #" << m_unNumEntityPlaced << " in grid");
         }
         CVector3 cReturn;
         if(m_unNumEntityPlaced < m_unLayout[0] * m_unLayout[1] * m_unLayout[2]) {
            cReturn.SetX(m_cCenter.GetX() + ( m_unLayout[0] - 1 ) * m_cDistances.GetX() * 0.5 - ( m_unNumEntityPlaced  % m_unLayout[0] ) * m_cDistances.GetX());
            cReturn.SetY(m_cCenter.GetY() + ( m_unLayout[1] - 1 ) * m_cDistances.GetY() * 0.5 - ( m_unNumEntityPlaced  / m_unLayout[0] ) % m_unLayout[1] * m_cDistances.GetY());
            cReturn.SetZ(m_cCenter.GetZ() + ( m_unLayout[2] - 1 ) * m_cDistances.GetZ() * 0.5 - ( m_unNumEntityPlaced / ( m_unLayout[0] * m_unLayout[1] ) ) * m_cDistances.GetZ());
            ++m_unNumEntityPlaced;
         }
         else {
            THROW_ARGOSEXCEPTION("Distribute position, method 'grid': trying to place more entities than allowed "
                                 "by the 'layout', check your 'quantity' tag");
         }
         return cReturn;
      }

   private:
      CVector3 m_cCenter;
      UInt32 m_unLayout[3];
      CVector3 m_cDistances;
      UInt32 m_unNumEntityPlaced;
   };

   /****************************************/
   /****************************************/

   class CustomGenerator : public RealNumberGenerator {
   public:
      CustomGenerator(const std::string& str_libloopfunction,
                        const std::string& str_labelloopfunction,
                          TConfigurationNode& t_tree ) :
         m_strLib(str_libloopfunction),
         m_strLabel(str_labelloopfunction),
         m_tTree(t_tree) {
           InitLoopFunctions();
         }

       void InitLoopFunctions() {
         try {
            CDynamicLoading::LoadLibrary(m_strLib);
            m_pcLoopFunction = CFactory<CLoopFunctions>::New(m_strLabel);
            m_pcCoreLoopFunction = static_cast<CoreLoopFunctions*>(m_pcLoopFunction);
            std::cout << m_tTree << std::endl;
            m_pcCoreLoopFunction->Init(m_tTree);
         }
         catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Error initializing loop functions", ex);
         }
       }

      inline virtual CVector3 operator()(bool b_is_retry) {
         return m_pcCoreLoopFunction->GetRandomPosition();
      }

   private:
      std::string m_strLib;
      std::string m_strLabel;
      TConfigurationNode m_tTree;
      CLoopFunctions* m_pcLoopFunction;
      CoreLoopFunctions* m_pcCoreLoopFunction;
   };

   /****************************************/
   /****************************************/

   class RVRCustomGenerator : public RealNumberGenerator {
   public:
      RVRCustomGenerator(const std::string& str_libloopfunction,
                        const std::string& str_labelloopfunction,
                          TConfigurationNode& t_tree ) :
         m_strLib(str_libloopfunction),
         m_strLabel(str_labelloopfunction),
         m_tTree(t_tree) {
           InitLoopFunctions();
         }

       void InitLoopFunctions() {
         try {
            CDynamicLoading::LoadLibrary(m_strLib);
            m_pcLoopFunction = CFactory<CLoopFunctions>::New(m_strLabel);
            m_pcRVRCoreLoopFunction = static_cast<RVRCoreLoopFunctions*>(m_pcLoopFunction);
            std::cout << m_tTree << std::endl;
            m_pcRVRCoreLoopFunction->Init(m_tTree);
         }
         catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Error initializing loop functions", ex);
         }
       }

      inline virtual CVector3 operator()(bool b_is_retry) {
         return m_pcRVRCoreLoopFunction->GetRandomPosition();
      }

   private:
      std::string m_strLib;
      std::string m_strLabel;
      TConfigurationNode m_tTree;
      CLoopFunctions* m_pcLoopFunction;
      RVRCoreLoopFunctions* m_pcRVRCoreLoopFunction;
   };

   /****************************************/
   /****************************************/

   RealNumberGenerator* CreateGenerator(TConfigurationNode& t_tree) {
      std::string strMethod;
      GetNodeAttribute(t_tree, "method", strMethod);
      if(strMethod == "uniform") {
         CVector3 cMin, cMax;
         GetNodeAttribute(t_tree, "min", cMin);
         GetNodeAttribute(t_tree, "max", cMax);
         if(! (cMin <= cMax)) {
            THROW_ARGOSEXCEPTION("Uniform generator: the min is not less than or equal to max: " << cMin << " / " << cMax);
         }
         return new UniformGenerator(cMin, cMax);
      }
      if(strMethod == "circularUniform") {
      		CVector3 cCenter;
      		Real fRadius;
      		GetNodeAttribute(t_tree, "center", cCenter);
      		GetNodeAttribute(t_tree, "radius", fRadius);
      		if(! (0 <= fRadius)) {
      			THROW_ARGOSEXCEPTION("Circular Uniform generator: the radius should be positive");
      		}
      		return new CircularUniformGenerator(cCenter, fRadius);
      }
      else if(strMethod == "gaussian") {
         CVector3 cMean, cStdDev;
         GetNodeAttribute(t_tree, "mean", cMean);
         GetNodeAttribute(t_tree, "std_dev", cStdDev);
         return new GaussianGenerator(cMean, cStdDev);
      }
      else if(strMethod == "constant") {
         CVector3 cValues;
         GetNodeAttribute(t_tree, "values", cValues);
         return new ConstantGenerator(cValues);
      }
      else if(strMethod == "grid") {
        CVector3 cCenter,cDistances;
        GetNodeAttribute(t_tree, "center", cCenter);
        GetNodeAttribute(t_tree, "distances", cDistances);
        UInt32 unLayout[3];
        std::string strLayout;
        GetNodeAttribute(t_tree, "layout", strLayout);
        ParseValues<UInt32> (strLayout, 3, unLayout, ',');
        return new GridGenerator(cCenter, unLayout, cDistances);
      }
      else if (strMethod == "fromLoopFunction") {
        std::string strLibrary, strLabel, strRobot;
        GetNodeAttribute(t_tree, "library", strLibrary);
        GetNodeAttribute(t_tree, "label", strLabel);
        GetNodeAttributeOrDefault(t_tree, "robot", strRobot, std::string("epuck"));
        if (strRobot == "rvr"){
            return new RVRCustomGenerator(strLibrary, strLabel, t_tree);
        } else {
            return new CustomGenerator(strLibrary, strLabel, t_tree);
        }
      }
      else {
       THROW_ARGOSEXCEPTION("Unknown distribution method \"" << strMethod << "\"");
       return NULL;
      }
   }




#endif /* POSITION_GENERATOR_H_ */
