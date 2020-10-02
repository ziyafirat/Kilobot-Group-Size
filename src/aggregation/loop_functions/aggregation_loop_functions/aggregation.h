
//#ifndef DEBUG_LOOP_FUNCTIONS_H
//#define DEBUG_LOOP_FUNCTIONS_H

//#include <kilolib.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>
#include <argos3/core/simulator/entity/entity.h>
#include "argos3/plugins/simulator/entities/box_entity.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <utility>
#include <list>
#include <math.h>

// A forward declaration for the kilobot controller.
// Just using this makes compilation lighter at this point.
class CCI_KilobotController;

////////////////////////////////////////
// DEBUGGING INFORMATION
//
// This is where the struct debug_info_t is defined.
//#include <aggregation/behaviors/aggregation.h>
//
////////////////////////////////////////




using namespace argos;

class CAggregation : public CLoopFunctions {

public:

	class CCI_KilobotController;

	//typedef std::map<CKilobotEntity*, std::vector<CVector3> > TWaypointMap;
   /**
    * Class constructor
    */
   CAggregation();

   /**
    * Class destructor
    */
   virtual ~CAggregation();

   /**
    * Initializes the experiment.
    * It is executed once at the beginning of the experiment, i.e., when ARGoS is launched.
    * @param t_tree The parsed XML tree corresponding to the <loop_functions> section.
    */
   virtual void Init(TConfigurationNode& t_tree);

   /**
    * Resets the experiment to the state it was right after Init() was called.
    * It is executed every time you press the 'reset' button in the GUI.
    */

   virtual void PlaceBots(float m_fArenaRadius);
   //virtual void ConfigBots();

   virtual void Reset();

   /**
    * Undoes whatever Init() did.
    * It is executed once when ARGoS has finished the experiment.
    */
   virtual void Destroy();

   /**
    * Performs actions right before a simulation step is executed.
    */
   virtual void PreStep();

   /*list< pair<float,float> > findCluster(list< pair<float,float> >::iterator seed, list< pair<float,float> >& pos);
   list< pair<float,float> > findCluster2(list< pair<float,float> >::iterator seed, list< pair<float,float> >& pos);
   double std2D(list< pair<float,float> > pos);
   float connectivity(list< pair<float,float> > pos);
   int clustersInfo(list< pair<float,float> > pos, vector<int>& sizes, vector<double>& stds);*/

   /**
    * Performs actions right after a simulation step is executed.
    */
   virtual void PostStep();


private:   /**
    * The path of the output file.
    */
   std::string m_strOutFile;
   std::string m_strWordFile;

   /**
    * The stream associated to the output file.
    */
   std::ofstream m_cOutFile;
   std::ofstream m_cWordFile;

   std::vector<CColor> m_vecKilobotColors;

   /**
    * This vector contains a list of positions of objects in the construction area
    */
   std::vector<CVector3> m_vecConstructionObjectsInArea;

   /**
    * Minimum and maximum Y coordinate for the objects in the construction area
    */
   Real m_fMinObjectY, m_fMaxObjectY;

   unsigned int minDist;
   unsigned int timeStopCond;

   Real beacon_blue_count;
   Real beacon_red_count;
   float a;
   float b;
   float commRange;

   double m;

   unsigned char link;

   //std::vector< std::pair<CCI_KilobotController*, commit*> > m_tKBs;

   std::vector<CKilobotEntity*> bots;
   bool* stayArray;

   //std::vector<CCI_KilobotController*>  m_tKBs;
   //std::vector< std::pair<CCI_KilobotController*, debug_info_t*> > m_tKBs;
   //std::vector<CCI_KilobotController*>  m_tKBs;

};
