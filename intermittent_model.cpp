#include "intermittent_model.h"
#include "buzz/buzzvm.h"

/****************************************/
/****************************************/

/**
 * Functor to get data from the robots
 */
struct GetRobotData : public CBuzzLoopFunctions::COperation {

   /** Constructor */
   GetRobotData(){}

   /** The action happens here */
   virtual void operator()(const std::string& str_robot_id, buzzvm_t t_vm) {
   }
};

/****************************************/
/****************************************/

/**
 * Functor to put the flow in the Buzz VMs.
 */
struct PutFlow : public CBuzzLoopFunctions::COperation {

   /** Constructor */
   PutFlow(const std::vector<Real> &vec_flow) : m_vecFlow(vec_flow) {}

   /** The action happens here */
   virtual void operator()(const std::string& str_robot_id, buzzvm_t t_vm) {}

   /** Calculated flow */
   const std::vector<Real> &m_vecFlow;
};

/****************************************/
/****************************************/

void CIntermittentModel::Init(TConfigurationNode& t_tree) {
   /* Call parent Init() */
   CBuzzLoopFunctions::Init(t_tree);
   
   m_pcRNG = CRandom::CreateRNG("argos");

   auto cMedium = GetSimulator().GetMedium<CRABMedium>("rab");
   auto mapRABs = GetSpace().GetEntitiesByType("rab");
   std::vector<std::string> buffer;
   for (auto it = mapRABs.begin(); it != mapRABs.end(); ++it)
   {
      auto cRAB = *any_cast<CRABEquippedEntity *>(it->second);
      auto setNbrs = cMedium.GetRABsCommunicatingWith(cRAB);
      buffer.clear();
      for (auto jt = setNbrs.begin(); jt != setNbrs.end(); ++jt)
      {
         buffer.push_back(jt.m_psElem->Data->GetId());
      }
      m_adjacency_hash[cRAB.GetId()] = buffer;
      printf("%s", cRAB.GetId().c_str());
   }
}

/****************************************/
/****************************************/

void CIntermittentModel::Reset() {
   /* Reset the flow */
   printf("HERE1");
}

/****************************************/
/****************************************/

void CIntermittentModel::Destroy() {
   printf("HERE2");
}

/****************************************/
/****************************************/

void CIntermittentModel::PostStep() {
   auto cMedium = GetSimulator().GetMedium<CRABMedium>("rab");
   auto mapRABs = GetSpace().GetEntitiesByType("rab");
   std::vector<std::string> buffer;
   for (auto it = mapRABs.begin(); it != mapRABs.end(); ++it)
   {
      auto cRAB = *any_cast<CRABEquippedEntity *>(it->second);
      auto setNbrs = cMedium.GetRABsCommunicatingWith(cRAB);
      buffer.clear();
      for (auto jt = setNbrs.begin(); jt != setNbrs.end(); ++jt)
      {
         buffer.push_back(jt.m_psElem->Data->GetId());
      }
      m_adjacency_hash[cRAB.GetId()] = buffer;
      printf("%s", cRAB.GetId().c_str());
   }
   int all_pairs_path_lengths[GetNumRobots()][GetNumRobots()][GetNumRobots()];
}

/****************************************/
/****************************************/

bool CIntermittentModel::IsExperimentFinished() {
   /* Feel free to try out custom ending conditions */
   printf("HERE3");
   return false;
}

/****************************************/
/****************************************/

int CIntermittentModel::GetNumRobots() const {
   return m_mapBuzzVMs.size();
}

/****************************************/
/****************************************/

void CIntermittentModel::BuzzBytecodeUpdated() {
   /* Convey the flow to every robot */
   BuzzForeachVM(PutFlow(m_vecFlow));
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CIntermittentModel, "intermittent_model");
