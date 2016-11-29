/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class for simulation of robotic exploration of unknown
 *  terrain. On each update, the robot is provided with traversability 
 *  data of the area in its trapezoidal field of view. The ground truth
 *  to be given to the robot are loaded from a .txt. file given by 
 *  the filename in config. 
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Oct 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#ifndef TRAVERSABILITY_EXPLORER_TASK_TASK_HPP
#define TRAVERSABILITY_EXPLORER_TASK_TASK_HPP

#include "traversability_explorer/TaskBase.hpp"

namespace envire {
    class Environment;
    class FrameNode;
    class TraversabilityGrid;
}

namespace traversability_explorer {

//typedef envire::TraversabilityGrid::ArrayType TravData;
    /**
     * Used to create a shared-ptr from a reference.
     */
    struct NullDeleter
    {
        void operator()(void const *) const {}
    };


   class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        envire::Environment* mpEnv;
        envire::FrameNode* mpFrameNode;
        envire::TraversabilityGrid* mpTravGrid;
        // boost::shared_ptr<envire::TraversabilityGrid::ArrayType> mpTravData;
        base::samples::RigidBodyState mCurrentPose;
        // Traversability data as loaded from the file
        std::vector< std::vector<int> > groundTruth;
        // Grid size
        int Nrow, Ncol;
        // Description of trapezoidal field of view
        float a,b,l;
        // max cost used for sbpl library cost mapping
        static const unsigned char SBPL_MAX_COST = 20;
        // SBPL COST TO CLASS ID MAPPING
        int sbplCostToClassID[SBPL_MAX_COST+1];

    public:
        Task(std::string const& name = "traversability_explorer::Task");

        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	   ~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    private:        
        void loadTraversabilityMap(std::string filename);
        void updateTraversabilityMap(); 
        void getLineCoeff(Eigen::Vector2f& v1, Eigen::Vector2f& v2, Eigen::Vector3f& lineCoeff);
        bool areIntersecting(Eigen::Vector2f& v1, Eigen::Vector2f& v2,
                             Eigen::Vector3f& lineCoeff, 
                             Eigen::Vector2f& vertex1, Eigen::Vector2f& vertex2);
    };
}

#endif

