#ifndef INTERMITTENT_MODEL_H
#define INTERMITTENT_MODEL_H

#include <string.h>
#include <iostream>
#include "hash_combine.h"
#include <unordered_map>
#include <buzz/argos/buzz_loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/simulator/media/rab_medium.h>

#define NUMROBOTS 20
#define INF 9999

class CIntermittentModel : public CBuzzLoopFunctions
{

public:
   CIntermittentModel() {}
   virtual ~CIntermittentModel() {}

   /**
    * Executes user-defined initialization logic.
    * @param t_tree The 'loop_functions' XML configuration tree.
    */
   virtual void Init(TConfigurationNode &t_tree);

   /**
    * Executes user-defined reset logic.
    * This method should restore the state of the simulation at it was right
    * after Init() was called.
    * @see Init()
    */
   virtual void Reset();

   /**
    * Executes user-defined logic right after a control step is executed.
    */
   virtual void PostStep();

   /**
    * Returns true if the experiment is finished, false otherwise.
    *
    * This method allows the user to specify experiment-specific ending
    * conditions. If this function returns false and a time limit is set in the
    * .argos file, the experiment will reach the time limit and end there. If no
    * time limit was set, then this function is the only ending condition.
    *
    * @return true if the experiment is finished.
    */
   virtual bool IsExperimentFinished();

   /**
    * Executes user-defined destruction logic.
    * This method should undo whatever is done in Init().
    * @see Init()
    */
   virtual void Destroy();

   virtual void BuzzBytecodeUpdated();

private:
   /**
    * Solves the all-pairs shortest paths problem using the
    * Floyd Warshall algorithm
    *
    */
   void FloydWarshall();

   /**
    * resets all the vectors so they can be populated by poststep
   */
   void resetLists();

   void findNetworks();
   /**
    * Extracts the path for any two nodes after the Floyd Warshall algorithm is run
    *
    */
   std::vector<std::vector<UInt16>> GetPath(UInt16 i, UInt16 j, UInt16 it = 0);

   int GetNumRobots() const;

private:
   /** A key which stores IDs for nodes A and B */
   typedef std::tuple<std::string, std::string> ak;

   /** The flow associated to the nodes */
   std::vector<Real> m_vecFlow;

   // int NetworkIn[NUMROBOTS] = {};

   /** Cheap conversion between arbitrary node IDs and integral representation */
   std::string m_id_to_key[NUMROBOTS];

   /** The adjacency hash of the graph */
   std::unordered_map<ak, UInt16> m_adjacency_hash;

   /** Stores a list of backpointers to the current set of shortest paths for each node pair */
   std::vector<int16_t> m_next[NUMROBOTS][NUMROBOTS];

   /** The output file name */
   std::string m_strOutFile;

   /** The output file stream */
   std::ofstream m_cOutFile;

   /** Random number generator */
   CRandom::CRNG *m_pcRNG;
};

#endif
