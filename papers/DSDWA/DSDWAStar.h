
//
//  DSDWAStar.h
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 12/26/22.
//  Copyright © 2022 MovingAI. All rights reserved.
//

#ifndef DSDWAStar_h
#define DSDWAStar_h

#include "TemplateAStar.h" // to get state definitions
#include "MNPuzzle.h" // to get the STP state

enum tExpansionPriority {
	kWA=0,
	kpwXD=1,
	kpwXU=2,
	kXDP=3,
	kXUP=4,
	OBDP=5,
	CADP=65,
	ECBP=95,
	kDSMAP11=55,
	kMAP=6,
	kHalfEdgeDrop=95,
	kDSMAP8=38,
	kDSMAP7=18,
	kDSMAP=28,
	kGreedy=10,
	kFullEdgeDrop=85,
	kDSMAP6=51,
	kPathSuboptDouble=70,
	kXDP90=7,
 	kDSDPolicyCount=12,
};

int lastExpansions=0, angleCounter=0;
double tolerance=0.0001; //It is ALSO embedded in TemplateAStar.h, AStarCompareWithF.
double globalMaxG=0, globalMaxH=-1, piviotG=0, queuepiviotG=0, piviotH=0, globalAngle=-1,prevBuckerAngle=0,prevBuckerAngle2=0,prevBuckerAngle3=0, usedGcost=0;

template <class state, class action, class environment, class openList = AStarOpenClosed<state, AStarCompareWithF<state>, AStarOpenClosedDataWithF<state>> >
class DSDWAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	DSDWAStar() {
		ResetNodeCount(); env = 0; stopAfterGoal = true; weight=1; reopenNodes = false; theHeuristic = 0;
		theConstraint = 0;
	}
	virtual ~DSDWAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state&, const state&, std::vector<action> & );
	
	openList openClosedList;
	state goal, start;

	state globalMaxState, piviot;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath, bool dummy);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath, bool dummy);
	void AddAdditionalStartState(state& newState);
	void AddAdditionalStartState(state& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{
		thePath.clear();
		uint64_t theID;
		if (openClosedList.Lookup(env->GetStateHash(node), theID) != kNotFound)
			ExtractPathToStartFromID(theID, thePath);
	}
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
    void ResetNodeCount() { 
		nodesExpanded = nodesTouched = 0; 
		uniqueNodesExpanded = 0; lastRegionExpanded=0; minH=MAXFLOAT; 
		maxH=FLT_MIN; easyProblem=true; prevH=0; guider=1; firstLast=0; 
		secondLast=0; thirdLast=0; angleCounter=0; globalMaxG=0; piviotG=0; 
		queuepiviotG=0; piviotH=0; globalMaxH=-1; globalAngle=-1;prevBuckerAngle=0;
		prevBuckerAngle2=0;prevBuckerAngle3=0;usedGcost=0;
	}
	int GetMemoryUsage();
	bool GetClosedListGCost(const state &val, double &gCost) const;
	bool GetOpenListGCost(const state &val, double &gCost) const;
	bool GetHCost(const state &val, double &hCost) const;
	bool GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &);
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedDataWithF<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedDataWithF<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	dataLocation GetStateLocation(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key); }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }

	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	void SetConstraint(Constraint<state> *c) { theConstraint = c; }

	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetNecessaryExpansions() const;
	void LogFinalStats(StatCollection *) {}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
		
	void OpenGLDraw() const;
	void Draw(Graphics::Display &disp) const;
	void DrawPriorityGraph(Graphics::Display &disp) const;
	
	double Phi(double h, double g)
	{
		if (policy == kXDP)
		{
				float nextF = (g+(2*weight-1)*h+sqrt((g-h)*(g-h)+4*weight*g*h))/(2*weight);
				
				// tmp line to try to visualize A*
				// float nextF = g+h;

				return nextF; 
				
		}
		if (policy == kXUP)
		{
				float nextF = (g+h+sqrt((g+h)*(g+h)+4*weight*(weight-1)*h*h))/(2*weight);
				return nextF;
				
		}
		else return GetPriority(h, g);
	}
	void SetWeight(double w)
	{
		weight = w;
	}
	double GetWeight() { return weight; }
	float GetPriority(float h, float g)
	{
		if (data.size() == 0)
		{
			if (fequal(g, 0, tolerance))
				return h;
			printf("WARNING: Invalid priority lookups %s line %d\n", __FILE__, __LINE__);
			return INFINITY;
		}
		float slope = g/h;
		if (fgreater(slope, data.back().slope, tolerance))
		{
			printf("WARNING: Invalid priority lookups %s line %d\n", __FILE__, __LINE__);
			return INFINITY;
		}
		// dumb/slow but correct
		// TODO: create a helper binary search function
		for (int x = 0; x < data.size(); x++)
			if (flesseq(slope, data[x].slope, tolerance))
				return data[x].K*(g + data[x].weight * h);
		return INFINITY;
	}
	float GetPriority(float h, float g, float region)
	{
		if (data.size() == 0)
		{
			if (fequal(g, 0, tolerance))
				return h;
			printf("WARNING: Invalid priority lookups %s line %d\n", __FILE__, __LINE__);
			return INFINITY;
		}
		float maxRegion = floor(atan(data.back().slope) * 180 / PI)+1;
		if (fgreater(region, maxRegion, tolerance))
		{
			printf("WARNING: Invalid priority lookups %s line %d\n", __FILE__, __LINE__);
			return INFINITY;
		}
		// dumb/slow but correct
		// TODO: create a helper binary search function
		for (int x = 0; x < data.size(); x++)
			{
				float theRegion = floor(atan(data[x].slope) * 180 / PI)+1;
				if (flesseq(region, theRegion, tolerance))
					return data[x].K*(g + data[x].weight * h);
			}
		return INFINITY;

		// for (int x = 0; x < data.size(); x++)
		// 	{
		// 		if (flesseq(region, data[x].slope, tolerance))
		// 			return data[x].K*(g + data[x].weight * h);
		// 	}
		// return INFINITY;
	}
	
	/**
	* Returns the current weight for this state--slope.
	* Returns the last weight if it is a new slope (not in data)
	* @author mohammadreza hami
	* @date 15/23/Dec
	* @param h and @param g The h and g values of the state
	**/ 
	float GetCurrentWeight(float h, float g)
	{
		if(data.size()>0)
		{
			float slope = g/h;
			for (int x = 0; x < data.size(); x++)
				if (flesseq(slope, data[x].slope, tolerance))
					{
						// std::cout<<"really here??";
						return data[x].weight;
					}
			// std::cout<<"why here??";
			return data.back().weight;
		}
		else
		{
			// std::cout<<"got here??";
			return weight;
		}
	}

	/**
	* This is the discretized version of the previous function.
	* Returns the current weight for this state --slope.
	* Returns the last weight if it is a new slope (not in data)
	* @author mohammadreza hami
	* @date 15/23/Dec
	* @param h and @param g The h and g values of the state
	* @param region discretized version of the slope in degree.
	**/ 
	float GetCurrentWeight(float h, float g, float region)
	{
		if(data.size())
		{
			for (int x = 0; x < data.size(); x++)
			{
				float theRegion = floor(atan(data[x].slope) * 180 / PI)+1;
				if (flesseq(region, theRegion, tolerance))
					return data[x].weight;
			}
		return data.back().weight;
		}
		else
		{
			return weight;
		}
		
	}

	// directly target next suboptimality
	void SetNextWeight(float h, float g, float targetWeight) // const Graphics::point &loc
	{
		if (fequal(h, 0, tolerance))
		{
			float w;
			point3d last;
			if (data.size() > 0)
				 last = data.back().crossPoint;
			else
				last = {1, 0};
			w = (weight-last.y)/last.x;
			// connect to y axis at bound*(1)
			float K = 1/(last.y+w*last.x);
			data.push_back({INFINITY, w, K, {static_cast<float>(weight), 0.0f}});
		}
		if (fgreater(h, 0, tolerance) && fgreater(g, 0, tolerance))
		{
			float slope = g/h;
			if (data.size() == 0 || fless(data.back().slope, slope))
			{
				//std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
				float minWeight, maxWeight;
				if (data.size() > 0)
					GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, slope);
				else
					GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);

				float K;
				float nextWeight = std::min(maxWeight, std::max(minWeight, targetWeight));
				point3d last;
				if (data.size() == 0)
				{
					last = point3d(1, 0);
				}
				else {
					last = data.back().crossPoint;
				}
				K = 1/(last.y+nextWeight*last.x);

				// our cross point of next slope
				point3d crossPoint1;
				crossPoint1.x = 1.0f/(K*(slope+nextWeight));
				crossPoint1.y = crossPoint1.x*slope;

				// std::cout<<"Stored weight2: "<<nextWeight<<"\n";
				
				data.push_back({slope, nextWeight, K, crossPoint1});
			}
		}
	}

	void SetNextPriority(float h, float g, float target) // const Graphics::point &loc
	{
		if (fequal(h, 0, tolerance))
		{
			float w;
			point3d last;
			if (data.size() > 0)
				 last = data.back().crossPoint;
			else
				last = {1, 0};
			w = (weight-last.y)/last.x;
			// connect to y axis at bound*(1)
			float K = 1/(last.y+w*last.x);
			data.push_back({INFINITY, w, K, {static_cast<float>(weight), 0.0f}});
		}
		if (fgreater(h, 0, tolerance) && fgreater(g, 0, tolerance))
		{
			float slope = g/h;
			if (data.size() == 0 || data.back().slope < slope)
			{
				//std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
				float minWeight, maxWeight;
				if (data.size() > 0)
					GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, slope);
				else
					GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);

				float K;
				float nextWeight;
				point3d last;
				if (data.size() == 0)
				{
					last = point3d(1, 0);
				}
				else {
					last = data.back().crossPoint;
				}

				
				switch (policy)
				{
					case kWA: // Weighted A*
					{
						K = 1/weight;
						nextWeight = weight;
						break;
					}
					case kpwXU: // PWXUP
					{
						nextWeight = maxWeight;
						K = 1/(last.y+nextWeight*last.x);
						break;
					}
					case kpwXD: // PWXDP
					{
						nextWeight = minWeight;
						K = 1/(last.y+nextWeight*last.x);
						break;
					}
					default:
					// case kGreedy:
					{
						// K (g + [w] * h) = 1 at previous point
						// returns nextWeight and K
						nextWeight = ChooseWeightForTargetPriority({h, g}, target, minWeight, maxWeight, last, K);
					}
				}
				

				// our cross point of next slope
				point3d crossPoint1;
				crossPoint1.x = 1.0f/(K*(slope+nextWeight));
				crossPoint1.y = crossPoint1.x*slope;

				// std::cout<<"Stored weight: "<<nextWeight<<"\n";
				
				data.push_back({slope, nextWeight, K, crossPoint1});
				
	//			std::cout << "Cross Priorities: ";
	//			for (const auto &i : data)
	//			{
	//				std::cout << i.crossPoint << ": ";
	//				std::cout << GetPriority(i.crossPoint.x, i.crossPoint.y) << " ";
	//			}
	//			std::cout << "\n";
			}
		}
	}
	
	/**
	 * Given the slope of the next bounding line, give the possbile range of weights that can be used in the priority function
	 *
	 * \param minWeight (returned) The minimum weight that can be used without going under the lower limit
	 * \param maxWeight (returned) The maximum weight that can be used without going over the upper limit
	 **/
	void GetNextWeightRange(float &minWeight, float &maxWeight, float nextSlope)
	{
		if (data.size() > 0)
			GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, nextSlope);
		else
			GetNextWeightRange(minWeight, maxWeight, {1, 0}, nextSlope);
	}

	/**
	 * Given the slope of the next bounding line, give the possbile range of weights that can be used in the priority function
	 *
	 * \param minWeight (returned) The minimum weight that can be used without going under the lower limit
	 * \param maxWeight (returned) The maximum weight that can be used without going over the upper limit
	 * \param currPoint The point on the previous bounding line with priorirty 1.0
	 * \param nextSlope The slope of the next bounding line
	 **/
	void GetNextWeightRange(float &minWeight, float &maxWeight, point3d currPoint, float nextSlope)
	{
		// defaults
		minWeight = 1;
		maxWeight = 2*weight-1;

		// 0. next slope line is y = slope*x
		// 1. cannot go over the [y = -x + w] line
		// slope*x = -x + w; slope*x + x = w; x = w/(slope+1)
		// y/slope = w-y; y = slope * w - slope *y; y*(1+slope) = slope*w; y = slope*w/(1+slope)
		point3d upperPoint(weight/(nextSlope+1),
						   nextSlope*weight/(1+nextSlope));
	//	std::cout << "Upper point: " << upperPoint << "\n";
		// 2. cannot go under the [y = -(2w-1)x + w] line
		// slope*x = -(2w-1)x + w; x*(slope + (2w-1)) = w
		// y/slope = (w-y)/(2w-1)
		// y (2w-1) = slope * w - slope*y
		// y (slope + 2w-1) = slope*w
		point3d lowerPoint(weight/(nextSlope+2*weight-1),
						   nextSlope*weight / (nextSlope + 2*weight-1));
		// get (negative) slopes to upper and lower points
		minWeight = std::max(minWeight, (lowerPoint.y-currPoint.y)/(currPoint.x-lowerPoint.x));
		// std::cout<<" inside NextRangefunc minWeight:"<<(lowerPoint.y-currPoint.y)/(currPoint.x-lowerPoint.x)<<"\n";
		if (upperPoint.x < currPoint.x)
			maxWeight = std::min(maxWeight, (upperPoint.y-currPoint.y)/(currPoint.x-upperPoint.x));
	//	printf("Weight needs to be [%f, %f]\n", minWeight, maxWeight);
	}
	
	/*
	 * Given location, and a range of priorities, try to give the point the exact desired priority
	 * Returned priority will always be in the minWeight/maxWeight range
	 */
	float ChooseWeightForTargetPriority(point3d loc, float priority, float minWeight, float maxWeight, point3d last, float &K)
	{
		float weight;
		// New: Last point is the priority 1 crossing point.
		// 1. Find the point on the previous line with priority {priority}
		point3d projectedPoint = last*priority;
		// 2. FYI: loc is the point on the new line that we want to have the desired priority
		// 3. Find the slope between the last point and our new point.
		//    Which is delta y / delta x
		if (flesseq(loc.y, projectedPoint.y, tolerance))
		{
			//printf("Ill defined case (new y < old y); defaulting to min\n");

			// Explainer: State can be immediately expanded, just use min
			K = 1/(last.y+minWeight*last.x);
			return minWeight;
		}
		if (fgreatereq(loc.x, projectedPoint.x, tolerance))
		{
//			printf("Ill defined case (new x > old x); defaulting to max\n");
			
			K = 1/(last.y+maxWeight*last.x);
			return maxWeight;
		}
		
		// Then try to extend that point to the new point giving it the desired priority
		weight = (loc.y-projectedPoint.y)/(projectedPoint.x-loc.x);
		// and bound by min/max weight
		weight = std::max(std::min(weight, maxWeight), minWeight);
		K = 1/(last.y+weight*last.x);
		return weight;
	}
	tExpansionPriority policy = kFullEdgeDrop;//kHalfEdgeDrop;
private:
	point3d HOGToLocal(point3d p) const
	{
		return point3d((p.x+1)*weight/2.0f , (p.y-1)*weight/-2.0);
	}

	point3d LocalToHOG(point3d p) const
	{
		return point3d(2.0f*(p.x/weight-0.5), -2.0*(p.y/weight-0.5));
	}

	uint64_t nodesTouched, nodesExpanded;

	uint64_t lastRegionExpanded, firstLast, secondLast, thirdLast; //added for NodeBased policy.
	float minH, maxH, prevH, guider; //added for TheOne policy.
	float dataAverage, dataLength;
	bool easyProblem;

	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<double> heuristicCosts;
	std::vector<dataLocation> neighborLoc;
	environment *env;
	bool stopAfterGoal;
	
	double goalFCost;
	double weight;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	environment *radEnv;
	Heuristic<state> *theHeuristic;
	Constraint<state> *theConstraint;

	struct DSDdata {
		float slope;
		float weight;
		float K;
		point3d crossPoint; // cached for simplicity
	};
	std::vector<DSDdata> data;
};

/**
 * Return the name of the algorithm.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */
template <class state, class action, class environment, class openList>
const char *DSDWAStar<state,action,environment,openList>::GetName()
{
	static char name[32];
	sprintf(name, "DSDWAStar[]");
	return name;
}

/**
 * Perform an A* search between two states.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @param thePath A vector of states which will contain an optimal path
 * between from and to when the function returns, if one exists.
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{
		if (10000000 == nodesExpanded){
			//Terminate the search after 10 million node expansions.
			printf("%" PRId64 " nodes expanded, %" PRId64 " generated. ", nodesExpanded, nodesTouched);
			std::cout<<"Policy "<<policy<<" => Terminated.\n";
			break;
		}
	}
}

template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
{
	std::vector<state> thePath;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	path.resize(0);
	// float maxRegion = 0;
	while (!DoSingleSearchStep(thePath)){}
	for (size_t x = 0; x < thePath.size()-1; x++)
	{
		path.push_back(_env->GetAction(thePath[x], thePath[x+1]));
	}
}

/**
 * Initialize the A* search
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @return TRUE if initialization was successful, FALSE otherwise
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state,action,environment,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	openClosedList.Reset(env->GetMaxHash());
	ResetNodeCount();
	start = from;
	goal = to;
	if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}
	data.resize(0);

	double h = theHeuristic->HCost(start, goal);
	openClosedList.AddOpenNode(start, env->GetStateHash(start), Phi(h, 0), 0, h);

	env->SetqueuePiviotState(start);
	env->SetPiviotState();
	globalMaxH = h;
	
	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::AddAdditionalStartState(state& newState)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), Phi(h, 0), 0, h);
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::AddAdditionalStartState(state& newState, double cost)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), Phi(h, cost), cost, h);
}

/**
 * Expand a single node.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param thePath will contain an optimal path from start to goal if the
 * function returns TRUE
 * @return TRUE if there is no path or if we have found the goal, FALSE
 * otherwise
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state,action,environment,openList>::DoSingleSearchStep(std::vector<state> &thePath)
{
	if (openClosedList.OpenSize() == 0)
	{
		thePath.resize(0); // no path found!
		//closedList.clear();
		return true;
	}
	uint64_t nodeid = openClosedList.Close();
//	if (openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h > lastF)
//	{ lastF = openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h;
//		//printf("Updated limit to %f\n", lastF);
//	}
	if (!openClosedList.Lookup(nodeid).reopened)
		uniqueNodesExpanded++;
	nodesExpanded++;

	if ((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal)))
	{
		ExtractPathToStartFromID(nodeid, thePath);
		// Path is backwards - reverse
		reverse(thePath.begin(), thePath.end());
		goalFCost = openClosedList.Lookup(nodeid).f;// + openClosedList.Lookup(nodeid).h;

		// std::cout<<"The Average is "<<dataAverage/dataLength<<"\n";
		return true;
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	heuristicCosts.resize(0);
	
	//std::cout << "Expanding: " << env->GetStateHash(openClosedList.Lookup(nodeid).data) << " with f:";
	//std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
	
	env->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
	float maxSlope = 0;
	float maxSlopeG=-1, maxSlopeH=-1;
	int which = -1;
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
		heuristicCosts.push_back(theHeuristic->HCost(neighbors[x], goal));

		// open list can only be reduced in cost, thus not needing to extend the priority function
		if (neighborLoc[x] == kOpenList)
			continue;
		float slope = (openClosedList.Lookup(nodeid).g+edgeCosts[x])/heuristicCosts[x];
		if (fgreater(slope, maxSlope, tolerance))
		{
			maxSlope = slope;
			maxSlopeG = openClosedList.Lookup(nodeid).g+edgeCosts[x];
			maxSlopeH = heuristicCosts[x];
			which = x;
		}
	}
	if (fgreater(maxSlope, 0, tolerance))
	{
		// TODO: handle edge cases
		// if (openClosedList.OpenSize() != 0 || openClosedList.OpenSize() == 0)
		if (openClosedList.OpenSize() != 0)
		{
			if (policy == kGreedy)
			{
				SetNextPriority(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f);
			}
			else if (policy == kHalfEdgeDrop) {
				SetNextPriority(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f-edgeCosts[which]*(weight-1)/(2.0*weight));
			}
			else if (policy == kFullEdgeDrop) {
				SetNextPriority(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f-edgeCosts[which]*(weight-1)/weight);
			}
			else if (policy == kPathSuboptDouble)
			{
				// Estimated cost to here: theHeuristic->HCost(start, neighbors[which]);
				// Actual cost to here: maxSlopeG
				float soFar = maxSlopeG/theHeuristic->HCost(start, neighbors[which]);
				SetNextWeight(maxSlopeH, maxSlopeG, (soFar+weight)-1); // const Graphics::point &loc
			} 
			else if (policy == kXDP90) 
			{
				float minWeight, maxWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlopeG/maxSlopeH);
				float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;
				assert(angle>=0 && angle<=90);
                SetNextWeight(maxSlopeH, maxSlopeG, minWeight+(maxWeight-minWeight)*(angle/90));
			}
			else if (policy == kMAP)
			{
				//CHECK THE SLOPE OF THE EXPANDED NODE TO SEE WHICH SECTION IT WAS FROM.
				//ADD ONE TO THE NUMBER OF THAT SECTION.
				float nodeSlope = openClosedList.Lookup(nodeid).g/openClosedList.Lookup(nodeid).h;
				float weightGuider;
				if(data.size()>=3)
				{
					if(nodeSlope <= data[data.size()-3].slope)
						thirdLast += 1;
					else if(nodeSlope <= data[data.size()-2].slope)
						secondLast += 1;
					else if(nodeSlope <= data[data.size()-1].slope)
						firstLast += 1;
				}

				float minWeight, maxWeight, midWeight, lowMidWeight, highMidWeight, lowHighMidWeight, highLowMidWeight, highHighMidWeight, lowLowMidWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);
				midWeight = (maxWeight + minWeight)/2;
				lowMidWeight = (midWeight + minWeight)/2;
				highMidWeight = (maxWeight + midWeight)/2;
				lowHighMidWeight = (lowMidWeight + midWeight)/2;
				highLowMidWeight = (highMidWeight + midWeight)/2;
				highHighMidWeight = (maxWeight + highMidWeight)/2;
				lowLowMidWeight = (lowMidWeight + minWeight)/2;


				float WMA = (3*firstLast + 2*secondLast + 1*thirdLast)/6;
				float rangeTop = (thirdLast + secondLast + firstLast)/2;
				float rangeButtom = (thirdLast + secondLast + firstLast)/6;
				
				////LARGER WEIGHTS IF (PROGRESS MADE = NODES MOSTLY EXPANDED IN THE MOST RECENT SECTION)
				//// 0<=weightGuider<=1
				// if(rangeTop - rangeButtom !=0)
				// 	weightGuider = (WMA - rangeButtom)/(rangeTop - rangeButtom);
				// else
				// 	weightGuider = 0;
				////SMALLER WEIGHTS IF (PROGRESS MADE = NODES MOSTLY EXPANDED IN THE MOST RECENT SECTION)
				//// 0<=weightGuider<=1
				if(rangeTop - rangeButtom !=0)
					weightGuider = 1-(WMA - rangeButtom)/(rangeTop - rangeButtom);
				else
					weightGuider = 0;

				////WEIGHTS IN THE RANGE OF minWeight to midWeight
				// float TheNextWeight = minWeight + (midWeight-minWeight)*weightGuider;
				////WEIGHTS IN THE RANGE OF lowMidWeight to highMidWeight
				// float TheNextWeight = lowMidWeight + (highMidWeight-lowMidWeight)*weightGuider;
				////WEIGHTS IN THE RANGE OF minWeight to maxWeight
				float TheNextWeight = minWeight + (maxWeight-minWeight)*weightGuider;

				SetNextWeight(maxSlopeH, maxSlopeG, TheNextWeight);

				if (fgreater(maxSlope, data.back().slope, tolerance))
				{
					// maxRegion = lastRegion;
					thirdLast = secondLast;
					secondLast = firstLast;
					firstLast = 0;
				}
			}
			else if (policy == kXDP)
			{
				float minWeight, maxWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);
				float nextF = (maxSlopeG+(2*weight-1)*maxSlopeH+sqrt((maxSlopeG-maxSlopeH)*(maxSlopeG-maxSlopeH)+4*weight*maxSlopeG*maxSlopeH))/(2*weight);
				SetNextPriority(maxSlopeH, maxSlopeG, nextF);

			}
			else if (policy == kXUP)
			{
				float minWeight, maxWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);
				float nextF = (maxSlopeG+maxSlopeH+sqrt((maxSlopeG+maxSlopeH)*(maxSlopeG+maxSlopeH)+4*weight*(weight-1)*maxSlopeH*maxSlopeH))/(2*weight);
				SetNextPriority(maxSlopeH, maxSlopeG, nextF);

			}
			else if (policy == kDSMAP)
			{
				float minWeight, maxWeight, midWeight, lowMidWeight, highMidWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);
				midWeight = (maxWeight + minWeight)/2;
				lowMidWeight = (midWeight + minWeight)/2;
				highMidWeight = (maxWeight + midWeight)/2;
				// double buckerScore = env->GetBuckerScore(openClosedList.Lookup(nodeid).data);
				double buckerScore = env->GetBuckerScore(neighbors[which]);
				// std::cout<<buckerScore<<std::endl<<"======"<<std::endl;
				float TheNextWeight = lowMidWeight + (highMidWeight-lowMidWeight)*buckerScore;
				
				int lastSize = data.size();
				SetNextWeight(maxSlopeH, maxSlopeG, TheNextWeight);
				// if(data.size() > lastSize)
				// {
				// 	std::cout<<nodesExpanded-lastExpansions<<" epansions in prev regions, "<<std::endl;;
				// 	lastExpansions = nodesExpanded;
				// 	std::cout<<"Generating ray #"<<data.size()<<std::endl;
				// 	env->PrintState(neighbors[which]);
				// 	std::cout<<"its slope= "<<maxSlope<<std::endl;
				// }
			}
			else if (policy == kDSMAP7)
			{
				float minWeight, maxWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);

				// double buckerScore = env->GetBuckerScore(openClosedList.Lookup(nodeid).data);
				double buckerScore = env->GetBuckerScore(neighbors[which]);

				if(buckerScore==0){
					float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;
                	SetNextWeight(maxSlopeH, maxSlopeG, minWeight+(maxWeight-minWeight)*(angle-prevBuckerAngle)/(90-prevBuckerAngle));
					}
				else{
					prevBuckerAngle = max(atan2f(maxSlopeG,maxSlopeH)/PID180, prevBuckerAngle);
					SetNextWeight(maxSlopeH, maxSlopeG, edgeCosts[which]);
				}
			}
			else if (policy == kDSMAP8)
			{
				// float minWeight, maxWeight;
				// GetNextWeightRange(minWeight, maxWeight, maxSlope);
				// int lastSize = data.size();
				// SetNextWeight(maxSlopeH, maxSlopeG, edgeCosts[which]);
				// std::cout<<"The Edge cost=weight is "<<edgeCosts[which]<<std::endl;

				float minWeight, maxWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);
				// int lastSize = data.size();
				double Nedge = env->NormalizeTileCost(openClosedList.Lookup(nodeid).data, neighbors[which], maxWeight, minWeight);
				// double Hb = theHeuristic->HCost(start, goal) - maxSlopeH;
				// double Hedge = theHeuristic->HCost(openClosedList.Lookup(nodeid).data, goal) - theHeuristic->HCost(neighbors[which], goal);
				// double NHedge = Nedge/edgeCosts[which]*Hedge;
				// float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;
                SetNextWeight(maxSlopeH, maxSlopeG, Nedge);

				// if(data.size() > lastSize)
				// {
				// 	std::cout<<nodesExpanded-lastExpansions<<" epansions in prev regions, "<<std::endl;;
				// 	lastExpansions = nodesExpanded;
				// 	std::cout<<"Generating ray #"<<data.size()<<std::endl;
				// 	env->PrintState(neighbors[which]);
				// 	std::cout<<"its weight="<<edgeCosts[which]<<" and maxWeight="<<maxWeight<<std::endl;
				// 	std::cout<<"------------"<<std::endl;
				// }	
			}
			else if (policy == kDSMAP11)
			{
				float minWeight, maxWeight, midWeight, lowMidWeight, highMidWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);
				midWeight = (maxWeight + minWeight)/2;
				lowMidWeight = (midWeight + minWeight)/2;
				highMidWeight = (maxWeight + midWeight)/2;
				float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;
				int lastSize = data.size();
				float deltaGlobalH, deltaGlobalG;
				float angleJump = 2.5;

				//Set the piviot state
				if(angle>=angleCounter*angleJump){

					env->SetPiviotState();
					env->SetqueuePiviotState(neighbors[which]);
					
					piviotG = queuepiviotG;
					queuepiviotG = maxSlopeG;
					
					// if not using the piviot state to [which] state heuristic
					// piviotH = theHeuristic->HCost(start, neighbors[which]);
					// or
					// piviotH = theHeuristic->HCost(neighbors[which], goal);

					// env->SetPiviotState(neighbors[which]);

					angleCounter += 1;
				}

				if(fgreater(maxSlope, data.back().slope)){
					globalMaxG = maxSlopeG;

					// if not using the piviot state to [which] state heuristic
					// globalMaxH = theHeuristic->HCost(start, neighbors[which]);
					// or
					// globalMaxH = theHeuristic->HCost(neighbors[which], goal); 

					globalMaxH = theHeuristic->HCost(env->GetPiviotState(), neighbors[which]);

					deltaGlobalH = globalMaxH;
					deltaGlobalG = globalMaxG - piviotG;
					// env->PrintState(env->GetPiviotState());
					// env->PrintState(neighbors[which]);
					// std::cout<<"deltaGlobalH: "<<deltaGlobalH<<"\n"<<"deltaGlobalG: "<<deltaGlobalG<<"\n======\n";
				
				
					// if not using the piviot state to which state heuristic
					// float deltaGlobalH = globalMaxH - piviotH;

					// To generate a number in [1,2] so min=2 and max=1
					float normalizeWeight = 2-(GetWeight()-1.5)/(10-1.5);
					// or
					// float normalizeWeight = 3-(GetWeight()-1.5)/(10-1.5)*2;

					// [0.7,1] : [0,1]*0.3->[0,0.3]+0.6->[0.6,0.9]
					float normalizeLimit = (GetWeight()-1.5)/(10-1.5)*(0.3)+0.6;

					
					if(fequal(deltaGlobalG, 0) || fgreater(deltaGlobalH/deltaGlobalG, 1)){

						if(fgreater(prevBuckerAngle3, prevBuckerAngle2)){
							//1
							SetNextWeight(maxSlopeH, maxSlopeG, maxWeight);
							prevBuckerAngle3 = max(atan2f(maxSlopeG,maxSlopeH)/PID180, prevBuckerAngle3);

							//2
							// float prevAngle = max(prevBuckerAngle, prevBuckerAngle2);
							// SetNextWeight(maxSlopeH, maxSlopeG, maxWeight-(maxWeight-minWeight)*(pow(((angle-prevAngle)/(90-prevAngle)), normalizeWeight)));
							// prevBuckerAngle3 = max(atan2f(maxSlopeG,maxSlopeH)/PID180, prevBuckerAngle3);
						}
						else if(fgreater(prevBuckerAngle2, prevBuckerAngle3)){
							//3:
							float prevAngle = max(prevBuckerAngle, prevBuckerAngle3);
							SetNextWeight(maxSlopeH, maxSlopeG, minWeight+(maxWeight-minWeight)*(pow(((angle-prevAngle)/(90-prevAngle)), normalizeWeight)));
							prevBuckerAngle2 = max(atan2f(maxSlopeG,maxSlopeH)/PID180, prevBuckerAngle2);
						}
						else{
							prevBuckerAngle = max(atan2f(maxSlopeG,maxSlopeH)/PID180, prevBuckerAngle);
							SetNextWeight(maxSlopeH, maxSlopeG, minWeight);
						}
					}
					// else if(flesseq(deltaGlobalH/deltaGlobalG, normalizeLimit)){
					else if(flesseq(deltaGlobalH/deltaGlobalG, 0.9)){
						//1
						SetNextWeight(maxSlopeH, maxSlopeG, maxWeight);
						prevBuckerAngle3 = max(atan2f(maxSlopeG,maxSlopeH)/PID180, prevBuckerAngle3);

						//2
						// float prevAngle = max(prevBuckerAngle, prevBuckerAngle2);
						// SetNextWeight(maxSlopeH, maxSlopeG, maxWeight-(maxWeight-minWeight)*(pow(((angle-prevAngle)/(90-prevAngle)), normalizeWeight)));
						// prevBuckerAngle3 = max(atan2f(maxSlopeG,maxSlopeH)/PID180, prevBuckerAngle3);
					}
					else{ //easy problems: lower weights
						
						//3
						float prevAngle = max(prevBuckerAngle, prevBuckerAngle3);
						SetNextWeight(maxSlopeH, maxSlopeG, minWeight+(maxWeight-minWeight)*(pow(((angle-prevAngle)/(90-prevAngle)), normalizeWeight)));
						prevBuckerAngle2 = max(atan2f(maxSlopeG,maxSlopeH)/PID180, prevBuckerAngle2);
					}
				}
				else{
					// dummy set weight, as it does not go into data.
					SetNextWeight(maxSlopeH, maxSlopeG, minWeight);
				}
			}
			else if (policy == CADP)
			{
				float minWeight, maxWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);
				float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;

				if(fgreater(maxSlope, data.back().slope)){
					globalMaxH = theHeuristic->HCost(openClosedList.Lookup(nodeid).data, neighbors[which]);
					globalMaxG = edgeCosts[which];
					if(fless(globalMaxH/globalMaxG, 1)){
						SetNextWeight(maxSlopeH, maxSlopeG, edgeCosts[which]);
					}
					else{
						float prevAngle = max(prevBuckerAngle, prevBuckerAngle3);
						SetNextWeight(maxSlopeH, maxSlopeG, minWeight+(maxWeight-minWeight)*(pow(((angle-prevAngle)/(90-prevAngle)), 3)));
					}
				}
			}
			else if (policy == OBDP)
			{
				float minWeight, maxWeight;
				GetNextWeightRange(minWeight, maxWeight, maxSlope);
				float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;
				assert(angle>=0 && angle<=90);
				if(fgreater(maxSlope, data.back().slope) || data.size() == 0){
					if(fgreater(globalMaxH, openClosedList.Lookup(nodeid).h)){

						float nextF = (maxSlopeG+maxSlopeH+sqrt((maxSlopeG+maxSlopeH)*(maxSlopeG+maxSlopeH)+4*weight*(weight-1)*maxSlopeH*maxSlopeH))/(2*weight);
						SetNextPriority(maxSlopeH, maxSlopeG, nextF);
					}
					else{ //easy problems: lower weights
						//1, 3 best
						float prevAngle = max(prevBuckerAngle, prevBuckerAngle3);
						SetNextWeight(maxSlopeH, maxSlopeG, minWeight+(maxWeight-minWeight)*(pow(((angle-prevAngle)/(90-prevAngle)), 3)));
					}
					globalMaxH = openClosedList.Lookup(nodeid).h;
				}
			}
			else {// To Run BaseLines using DSDWA* for graphic
				// -> CL code, uses template Astar
				// last argument will be ignored
				SetNextPriority(maxSlopeH, maxSlopeG, 0.01);
			}
		}
		else {
			// Expansion of the start state. open is empty.

			// Unsure if this is the right setting - can use lowest possible weight
			// or perhaps use same weight as parent?
			float minWeight, maxWeight;
			GetNextWeightRange(minWeight, maxWeight, maxSlope);

			if (policy == OBDP || policy == CADP)
				SetNextWeight(maxSlopeH, maxSlopeG, minWeight);
			else 
				SetNextPriority(maxSlopeH, maxSlopeG, 0.01);
		}
	}
	
	// iterate again updating costs and writing out to memory
	for (size_t x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		//std::cout << "looking at child with hash : " << env->GetStateHash(neighbors[x]) << "and g-cost"<<openClosedList.Lookup(nodeid).g+edgeCosts[x]<<std::endl;
		if (theConstraint &&
			theConstraint->ShouldNotGenerate(start, openClosedList.Lookup(nodeid).data, neighbors[x],
											 openClosedList.Lookup(nodeid).g+edgeCosts[x], goal))
			continue;

		switch (neighborLoc[x])
		{
			case kClosedList:
				// TODO: Can update parent pointers when shorter paths are found to improve solution quality
				if (reopenNodes)
				{
					if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g, tolerance))
					{
						auto &i = openClosedList.Lookup(neighborID[x]);
						i.parentID = nodeid;
						i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
						i.f = Phi(i.h, i.g);
						openClosedList.Reopen(neighborID[x]);
						// This line isn't normally needed, but in some state spaces we might have
						// equality but different meta information, so we need to make sure that the
						// meta information is also copied, since this is the most generic A* implementation
						i.data = neighbors[x];
					}
				}
				break;
			case kOpenList:
				if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g, tolerance))
				{
					auto &i = openClosedList.Lookup(neighborID[x]);
					i.parentID = nodeid;
					i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
					i.f = Phi(i.h, i.g);
					// This line isn't normally needed, but in some state spaces we might have
					// equality but different meta information, so we need to make sure that the
					// meta information is also copied, since this is the most generic A* implementation
					i.data = neighbors[x];
					openClosedList.KeyChanged(neighborID[x]);
//					std::cout << " Reducing cost to " << openClosedList.Lookup(nodeid).g+edgeCosts[x] << "\n";
					// TODO: unify the KeyChanged calls.
				}
				else {
//					std::cout << " no cheaper \n";
				}
				break;
			case kNotFound:
				{
					double h = heuristicCosts[x];
					openClosedList.AddOpenNode(neighbors[x],
											   env->GetStateHash(neighbors[x]),
											   Phi(h, openClosedList.Lookup(nodeid).g+edgeCosts[x]),
											   openClosedList.Lookup(nodeid).g+edgeCosts[x],
											   h,
											   nodeid);
				}
		}
	}

	return false;
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue).
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The first state in the open list.
 */
template <class state, class action, class environment, class openList>
state DSDWAStar<state, action,environment,openList>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
	//assert(false);
	//return openQueue.top().currNode;
}

/**
 * Get the path from a goal state to the start state
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment,class openList>
void DSDWAStar<state, action,environment,openList>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

template <class state, class action,class environment,class openList>
const state &DSDWAStar<state, action,environment,openList>::GetParent(const state &s)
{
	uint64_t theID;
	openClosedList.Lookup(env->GetStateHash(s), theID);
	theID = openClosedList.Lookup(theID).parentID;
	return openClosedList.Lookup(theID).data;
}

template <class state, class action, class environment, class openList>
uint64_t DSDWAStar<state, action,environment,openList>::GetNecessaryExpansions() const
{
	uint64_t n = 0;
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (fless(data.g + data.h, goalFCost, tolerance))
			n++;
	}
	return n;
}


/**
 * A function that prints the number of states in the closed list and open
 * queue.
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by DSDWAStar
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment, class openList>
int DSDWAStar<state, action,environment,openList>::GetMemoryUsage()
{
	return openClosedList.size();
}

/**
 * Get state from the closed list
 * @author Nathan Sturtevant
 * @date 10/09/07
 *
 * @param val The state to lookup in the closed list
 * @gCost The g-cost of the node in the closed list
 * @return success Whether we found the value or not
 * the states
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetClosedListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kClosedList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetOpenListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kOpenList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetHCost(const state &val, double &hCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc != kNotFound)
	{
		hCost = openClosedList.Lookat(theID).h;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &result)
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(s), theID);
	if (loc == kClosedList)
	{
		result = openClosedList.Lookat(theID);
		return true;
	}
	return false;

}


/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 * Deprecated
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::OpenGLDraw() const
{} // deprecated

/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 7/12/16
 *
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::Draw(Graphics::Display &disp) const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kClosedList)
		{
			//			if (top != -1)
			//			{
			//				env->SetColor((data.g+data.h-minf)/(maxf-minf), 0.0, 0.0, transparency);
			//			}
			//			else {
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			//			}
			env->Draw(disp, data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(disp, goal);
}


template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::DrawPriorityGraph(Graphics::Display &display) const
{
	point3d origin(-1, 1);
	float priority = 1;
	float bound = weight;
	
	// draw bounding line
	point3d bl1(priority, 0), bl2(0, priority*bound); // main suboptimality line
	point3d bl3(priority*bound, 0);//(0+2, priority*bound-2); //
	point3d bl4(priority*bound/(2*bound-1), 0);//priority*bound-(2*bound-1));
	point3d bl2a(0, priority);
	point3d bl2c(priority-priority*bound/(2*bound-1), priority*bound);
	// WA* priority line
	display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2), 1/100.0f, Colors::yellow);
	// 45° upper bound line
	display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl3), 1/100.0f, Colors::darkgray);
	// 2w-1 upper bound line
	display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2c), 1/100.0f, Colors::darkgray);

	// 45° lower bound line
	display.DrawLine(LocalToHOG(bl2a), LocalToHOG(bl1), 1/100.0f, Colors::lightgray);
	// 2w-1 lower bound line
	display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl4), 1/100.0f, Colors::lightgray);
	
	
	// Draw actual priority line across
	for (int x = 0; x < data.size(); x++)
	{
		point3d value = origin;
		
		// y = slope * x // x=1 -> y = slope; y=1 -> x = 1/slope;
		if (isinf(data[x].slope))
		{
			value.x = -1;
			value.y = -1;
		}
		else if (data[x].slope < 1)
		{
			value.x += 2;
			value.y -= 2*data[x].slope;
		}
		else {
			value.x += 2/data[x].slope;
			value.y -= 2;
		}
		display.DrawLine(origin, value, 1/200.0f, Colors::blue);
		
		if (isinf(data[x].slope))
		{
			point3d crossPoint2;
			float lastSlope = ((x==0)?(0):(data[x-1].slope));
			crossPoint2.x = priority/(data[x].K*(lastSlope+data[x].weight));
			crossPoint2.y = crossPoint2.x*lastSlope;
			display.DrawLine(LocalToHOG({0, static_cast<float>(weight)}), LocalToHOG(crossPoint2), 1/100.0f, Colors::red);
		}
		else {
			point3d crossPoint1, crossPoint2;
			crossPoint1.x = priority/(data[x].K*(data[x].slope+data[x].weight));
			crossPoint1.y = crossPoint1.x*data[x].slope;
			float lastSlope = ((x==0)?(0):(data[x-1].slope));
			crossPoint2.x = priority/(data[x].K*(lastSlope+data[x].weight));
			crossPoint2.y = crossPoint2.x*lastSlope;
			display.DrawLine(LocalToHOG(crossPoint1), LocalToHOG(crossPoint2), 1/100.0f, Colors::red);
		}
	}
	for (int x = 0; x < data.size(); x++)
		display.FillCircle(LocalToHOG(data[x].crossPoint), 0.01, Colors::darkgreen);

	display.DrawLine(origin, {1, 1}, 1./100.0f, Colors::white);
	display.DrawLine(origin, {-1, -1}, 1./100.0f, Colors::white);
}

#endif /* DSDWAStar_h */
