#include "intermittent_model.h"
#include "buzz/buzzvm.h"
#include <argos3/core/utility/logging/argos_log.h>
// #include <argos3/core/utility/datatypes/datatypes.h>

/****************************************/
/****************************************/

/**
 * Functor to get data from the robots
 */
struct GetRobotData : public CBuzzLoopFunctions::COperation
{

   /** Constructor */
   GetRobotData() {}

   /** The action happens here */
   virtual void operator()(const std::string &str_robot_id, buzzvm_t t_vm)
   {
   }
};

/****************************************/
/****************************************/

/**
 * Functor to put the flow in the Buzz VMs.
 */
// struct PutFlow : public CBuzzLoopFunctions::COperation
// {

//    /** Constructor */
//    PutFlow(float* vec_flow, UInt16* m_id_to_key){
//       _m_id_to_key = m_id_to_key;
//       _vecFlow = vec_flow;
//    }

//    /** The action happens here */
//    virtual void operator()(const std::string &str_robot_id, buzzvm_t t_vm) {
//       // std::string str_var = "flow";

//       printf("%s, %u \n", str_robot_id.c_str(), t_vm->getIndex());

//       BuzzTableOpen(t_vm, "flow");
      
//       for(UInt16 i = 0; i < NUMROBOTS; i++) {
         
//          // buzzvm_pushs(t_vm, buzzvm_string_register(t_vm, str_robot_id.c_str(), 0));
//          // buzzvm_pushi(t_vm, static_cast<int>(m_vecFlow[i]));
//          // buzzvm_gstore(t_vm);

//          // if(_m_id_to_key[i] == str_robot_id){
//          //    BuzzTablePut(t_vm, 0, static_cast<float>(_vecFlow[i]));
//          //    // buzzvm_pushs(t_vm, buzzvm_string_register(t_vm, str_var.c_str(), 0));
//          //    // buzzvm_pushi(t_vm, static_cast<in   t>(m_vecFlow[i]));
//          //    // buzzvm_gstore(t_vm);   
//          //    LOG << " " << _m_id_to_key[i].c_str() << " " << std::endl
//          //        << str_robot_id.c_str() << std::endl;

//          //    printf("put_in_bot\n %d", i);
//          // }
//          // BuzzTablePut(t_vm, i, static_cast<float>(m_vecFlow[i]));
//       }
      


//       /* Set the values of the table 'stimulus' in the Buzz VM */
//       // BuzzTableOpen(t_vm, "stimulus");
//       // for(int i = 0; i < m_vecFlow.size(); ++i) {
//       //    BuzzTablePut(t_vm, i, static_cast<float>(m_vecFlow[i]));
//       // }
//       BuzzTableClose(t_vm);
//       LOG << std::endl;


//    }

//    /** Calculated flow */
//    float* _vecFlow;
//    UInt16* _m_id_to_key;
// };

/****************************************/
/****************************************/

void CIntermittentModel::Init(TConfigurationNode &t_tree)
{
   /* Call parent Init() */

   printf("init\n");

   CBuzzLoopFunctions::Init(t_tree);
   // m_vecFlow.resize(NUMROBOTS);

   m_pcRNG = CRandom::CreateRNG("argos");
}

/****************************************/
/****************************************/

void CIntermittentModel::Reset()
{
   /* Reset the flow */

   printf("resetting\n");
}

/****************************************/
/****************************************/

void CIntermittentModel::Destroy()
{
}

/****************************************/
/****************************************/

void CIntermittentModel::resetLists(){

   //reset the flow associated
   // m_vecFlow = std::vector<Real>();


   //reset the adjacency hash
   m_adjacency_hash = std::unordered_map<ak, UInt16>();


   //reset all backpointers
   for(int i = 0; i < NUMROBOTS; i++){
      m_vecFlow[i] = 0;
      for(int j = 0; j < NUMROBOTS; j++){
         m_next[i][j] = std::vector<int16_t>();
      }
   }

}

/****************************************
*****************************************/

// void CIntermittentModel::findNetworks(){
//    //find 2 agents without a path between them. 
//    //assign them ID "1" and ID "2"
//    //go outwards from each agent, 
//    for(int i = 0; i < NUMROBOTS; i++){

//    }
// }

/****************************************
*****************************************/

void CIntermittentModel::PostStep()
{

   printf("starting poststep\n");

   resetLists();
   CRABMedium& cMedium = GetSimulator().GetMedium<CRABMedium>("rab");
   auto mapRABs = GetSpace().GetEntitiesByType("rab");
   UInt16 i = 0, j = 0;
   for (auto it = mapRABs.begin(); it != mapRABs.end(); ++it, ++i)
   {
      CRABEquippedEntity cRAB = *any_cast<CRABEquippedEntity *>(it->second);
      auto setNbrs = cMedium.GetRABsCommunicatingWith(cRAB);
      std::string nameOfRoot = cRAB.GetRootEntity().GetId();
      buzzvm_t vm_ref_key= BuzzGetVM(nameOfRoot);
      m_id_to_key[i] = vm_ref_key;
      m_id_to_index[i] = cRAB.GetIndex();
      

      j = 0;

      // printf("populate adjaceny hash (%u) for %u\n", (unsigned int)i, cRAB.GetIndex());
      // printf(i);
      for (auto jt = setNbrs.begin(); jt != setNbrs.end(); ++jt, ++j)
      {
         m_adjacency_hash[std::make_tuple((UInt16)cRAB.GetIndex(), (UInt16)jt.m_psElem->Data->GetIndex())] = 1;
         m_next[i][j].push_back(-1);
      }
   }


   printf("starting floyd warshall\n");
   // Collect all the shortest paths
   FloydWarshall();

   std::vector<std::vector<UInt16>> all_all_paths = {};
   for (i = 0; i < NUMROBOTS; ++i)
   {
      for (j = 0; j < NUMROBOTS; ++j)
      {
         // printf("getting all paths for %u, %u\n",m_id_to_key[i], m_id_to_key[j]);

         std::vector<std::vector<UInt16>> all_paths = GetPath(i, j);

         // for(std::vector<UInt16> path : all_paths){
         //    printf("path: ");
         //    for (UInt16 node_index : path)
         //       printf(" %u", (unsigned int)node_index);
         //    printf("\n");
         // }

         all_all_paths.insert(all_all_paths.end(), all_paths.begin(), all_paths.end());
      }
   }
   // std::fill(m_vecFlow.begin(), m_vecFlow.end(), 0);
   // // Now we can brute force through everything and find the number 
   // // of shortest paths that pass through each node
   for (auto path : all_all_paths)
      
      for (UInt16 node : path){
         // printf(" %u, %f \n",   (unsigned int)node, m_vecFlow[(unsigned int) node]);
         m_vecFlow[(unsigned int) node] = m_vecFlow[(unsigned int) node] + 1.0;
      }
// 
   printf("found flow\n");
   // std::this_thread::sleep_for( dura );
   // // LOG << " to update ";
   // // BuzzBytecodeUpdated();

   // printf("put for each\n");
   // printf('\n');
   // BuzzForeachVM(PutFlow(m_vecFlow, m_id_to_key));
   
   for (int k = 0; k < NUMROBOTS; k++)
   {
      printf("here %u \n", k);

      BuzzPut(m_id_to_key[k], "flow", m_vecFlow[k]);
   }
}

/****************************************/
/****************************************/

void CIntermittentModel::FloydWarshall()
{
   UInt16 i, j;
   int16_t k;
   for (k = 0; k < NUMROBOTS; k++)
   {
      // Pick all vertices as source one by one
      for (i = 0; i < NUMROBOTS; i++)
      {
         // Pick all vertices as destination for the
         // picked source
         for (j = 0; j < NUMROBOTS; j++)
         {
            auto ij = std::make_tuple(m_id_to_index[i], m_id_to_index[j]);
            auto ik = std::make_tuple(m_id_to_index[i], m_id_to_index[k]);
            auto kj = std::make_tuple(m_id_to_index[k], m_id_to_index[j]);
            if (m_adjacency_hash.find(ij) != m_adjacency_hash.end() &&
                m_adjacency_hash.find(ik) != m_adjacency_hash.end() &&
                m_adjacency_hash.find(kj) != m_adjacency_hash.end())
            {
               auto dist = (m_adjacency_hash[ik] + m_adjacency_hash[kj]);
               if (m_adjacency_hash[ij] > dist)
               {
                  m_adjacency_hash[ij] = dist;
                  m_next[i][j].clear();
                  m_next[i][j].push_back(k);
               }
               else if ((m_adjacency_hash[ij] == dist) && (m_adjacency_hash[ij] != INF) && (k != j) && (k != i))
               {
                  m_next[i][j].push_back(k);
               }
            }
         }
      }
   }
}

/****************************************/
/****************************************/

// Gross recursion
std::vector<std::vector<UInt16>> CIntermittentModel::GetPath(UInt16 i, UInt16 j, UInt16 it)
{
   std::vector<std::vector<UInt16>> all_paths = {};
   if (m_next[i][j].size() != 0)
   {
      for (auto k : m_next[i][j])
      {
         if (k == -1)
            all_paths.push_back({i, j});
         else
         {
            auto paths_I_K = GetPath(i, k, it + 1);
            auto paths_K_J = GetPath(k, j, it + 1);
            for (auto p_ik : paths_I_K)
            {
               if (p_ik.size() != 0)
               {
                  p_ik.pop_back();
               }
               for (auto p_kj : paths_K_J)
               {
                  auto tmp = p_ik;
                  tmp.insert(tmp.end(), p_kj.begin(), p_kj.end());
                  all_paths.push_back(tmp);
               }
            }
         }
      }
   }
   if (it == 0)
   {
      // Sort and then use std::unique to remove consecutive duplicates
      // There is probably a faster way to combine these operations but oh well
      std::sort(all_paths.begin(), all_paths.end(), [](const std::vector<UInt16> &a, const std::vector<UInt16> &b)
                { return a.size() < b.size(); });
      UInt16 min_ = INF;
      for (UInt16 idx = 0; idx < all_paths.size(); ++idx)
         if (all_paths[idx].size() <= min_)
            min_ = all_paths[idx].size();
         else
            break;
      all_paths.erase(std::remove_if(all_paths.begin(), all_paths.end(), [min_](std::vector<UInt16> path)
                                     { return path.size() > min_; }),
                      all_paths.end());
      all_paths.erase(std::unique(all_paths.begin(), all_paths.end()), all_paths.end());
   }
   return all_paths;
}

/****************************************/
/****************************************/

bool CIntermittentModel::IsExperimentFinished()
{
   /* Feel free to try out custom ending conditions */
   return false;
}

/****************************************/
/****************************************/

int CIntermittentModel::GetNumRobots() const
{
   return m_mapBuzzVMs.size();
}

/****************************************/
/****************************************/

void CIntermittentModel::BuzzBytecodeUpdated()
{
   /* Convey the flow to every robot */

   // BuzzForeachVM(PutFlow(m_vecFlow, m_id_to_key));
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CIntermittentModel, "intermittent_model");
