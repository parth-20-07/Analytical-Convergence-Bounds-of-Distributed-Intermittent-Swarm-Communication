#include "intermittent_proj.h"
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
 * Functor to put the stimulus in the Buzz VMs.
 */
struct PutFlow : public CBuzzLoopFunctions::COperation {

   /** Constructor */
   PutFlow(){}
   
   /** The action happens here */
   virtual void operator()(const std::string& str_robot_id, buzzvm_t t_vm) {

   }
};

/****************************************/
/****************************************/

void CMuleThreshold::Init(TConfigurationNode& t_tree) {
   /* Call parent Init() */
   CBuzzLoopFunctions::Init(t_tree);
   
   m_pcRNG = CRandom::CreateRNG("argos");
}

/****************************************/
/****************************************/

void CMuleThreshold::Reset() {
   /* Reset the stimuli */
}

/****************************************/
/****************************************/

void CMuleThreshold::Destroy() {
   
}

/****************************************/
/****************************************/

void CMuleThreshold::PostStep() {
   
}

/****************************************/
/****************************************/

bool CMuleThreshold::IsExperimentFinished() {
   /* Feel free to try out custom ending conditions */
   return false;
}

/****************************************/
/****************************************/

int CMuleThreshold::GetNumRobots() const {
   return m_mapBuzzVMs.size();
}

/****************************************/
/****************************************/

void CMuleThreshold::BuzzBytecodeUpdated() {
   /* Convey the stimuli to every robot */
   BuzzForeachVM(PutStimuli(m_vecStimuli));
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CThresholdModel, "threshold_model");
