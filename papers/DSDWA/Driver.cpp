/*
 *  hog2
 *
 *  Created by Nathan Sturtevant on 12/13/22.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Driver.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "ScenarioLoader.h"
#include "FPUtil.h"
#include "Graphics.h"
#include "SVGUtil.h"
#include "DSDWAStar.h"
#include "MapGenerators.h"
#include "DynamicPotentialSearch.h"
#include <ctime>
#include <cmath>
#include "GridHeuristics.h"

int stepsPerFrame = 1;
float bound = 9.0;
int problemNumber = 0;
float testScale = 1.0;
void GetNextWeightRange(float &minWeight, float &maxWeight, point3d currPoint, float nextSlope);
float GetPriority(float h, float g);
float ChooseWeightForTargetPriority(point3d point, float priority, float minWeight, float maxWeight, point3d last, float &K);
DSDWAStar<xyLoc, tDirection, MapEnvironment> dsd;
TemplateAStar<xyLoc, tDirection, MapEnvironment> tas;
std::vector<xyLoc> solution;
MapEnvironment *me = 0;
xyLoc start, goal, swampedloc, swampedloc2, swampedloc3, swampedloc4;
int exper=4;
float a, b, tspp=45,ts=10, tsx=10, tsy=10, tsx2=10, tsy2=10, tsx3=10, tsy3=10, tsx4=10, tsy4=10;
double Tcosts[4], rdm, hardness[4];
bool showPlane = false;
bool searchRunning = false;
bool saveSVG = false;
bool useDH = false;
int randomIndex;
xyLoc xyLocRandomState;
MNPuzzleState<4, 4> mnpRandomState;
tExpansionPriority prevPolicy;
GridEmbedding *dh;

int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
	InstallHandlers();
	RunHOGGUI(argc, argv, 1500, 1500);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{ 
	InstallKeyboardHandler(MyDisplayHandler, "Reset lines", "Reset incremenetal lines", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Show plane", "Show gradient of plane heights", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Speed up search animation", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Slow down search animation", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Policy", "Increment policy", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Policy", "Decrement policy", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Problem", "Increment problem", kAnyModifier, '.');
	InstallKeyboardHandler(MyDisplayHandler, "Swamped Problems", "Increment swamped problem", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Bound", "Increment bound", kAnyModifier, 'w');

	InstallCommandLineHandler(MyCLHandler, "-stp", "-stp problem alg weight puzzleW", "Test STP <problem> <algorithm> <weight> <puzzleW>");
	InstallCommandLineHandler(MyCLHandler, "-stpBLs", "-stp problem alg weight puzzleW", "Test STP <problem> <algorithm> <weight> <puzzleW>");
	InstallCommandLineHandler(MyCLHandler, "-exp0", "-map <map> alg weight TerrainSize mapType", "Test grid <map> with <algorithm> <weight> <TerrainSize> <mapType>");
	InstallCommandLineHandler(MyCLHandler, "-DSMAP", "-map <map> <scenario> alg weight TerrainSize SwampHardness", "Test grid <map> on <scenario> with <algorithm> <weight> <TerrainSize> and <SwampHardness>");
	InstallCommandLineHandler(MyCLHandler, "-gridBLs", "-map <map> <scenario> alg weight TerrainSize SwampHardness", "Test grid <map> on <scenario> with <algorithm> <weight> <TerrainSize> and <SwampHardness>");
	InstallCommandLineHandler(MyCLHandler, "-pwxds", "-map <map> <scenario> alg weight TerrainSize SwampHardness", "Test grid <map> on <scenario> with <algorithm> <weight> <TerrainSize> and <SwampHardness>");
	InstallCommandLineHandler(MyCLHandler, "-stpAstar", "-stpAstar problem alg weight", "Test STP <problem> <algorithm> <weight>");
	InstallCommandLineHandler(MyCLHandler, "-DPstp", "-DPstp problem weight", "Test STP <problem> <weight>");
	InstallCommandLineHandler(MyCLHandler, "-timeDSWA", "-DSDWA* stp problem weight", "Test STP <problem> <weight>");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map <map> <scenario> alg weight", "Test grid <map> on <scenario> with <algorithm> <weight>");
	
	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler);

}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare);
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare);
		Map *m = new Map(200, 200);
		srandom(20221228);

		// BuildRandomRoomMap(m, 30);
		// MakeRandomMap(m, 10);
		MakeMaze(m, 1);
      	// MakeDesignedMap(m, 20, 3);

		// default 8-connected with ROOT_TWO edge costs
		me = new MapEnvironment(m);

        // me->SetDiagonalCost(1.41);
		me->SetDiagonalCost(1.5);
		dsd.policy = kWA;
		start = {1,1};
		goal = {198, 198};

		swampedloc = {1,1};
		swampedloc2 = {1,1};
		swampedloc3 = {1,1};
		swampedloc4 = {1,1};

		//Set the cost of each terrain type randomly.
		me->SetInputWeight(bound);
		for(int i=0; i<4; i++){
			//[0]=kSwamp, [1]=kWater,[2]=kGrass, [3]=kTrees
			string type;
			if(i==0) type="Swamp";
			if(i==1) type="Water";
			if(i==2) type="Grass";
			if(i==3) type="Trees";

			// Define the Hardness
			// 1. Random Hardness
			// rdm = random()%101;
			// hardness[i] = rdm/101+1;
			//Or 2. Hardcoded Hardness
			hardness[0]=17; hardness[1]=1.45; hardness[2]=1.25; hardness[3]=1.95;
			
			// Use Hardness to define Cost
			// 1. Hardness with respect to input w
			// Tcosts[i] = hardness[i]*(me->GetInputWeight())-(hardness[i]-1);
			// std::cout<<"Cost "<<type<<"="<<Tcosts[i]<<" ("<<hardness[i]<<"*"<<me->GetInputWeight()<<"-"<<(hardness[i]-1)<<")"<<std::endl;
			//Or 2. Hardness as the exact Cost
            Tcosts[i] = hardness[i];
			std::cout<<"Cost "<<type<<"="<<Tcosts[i]<<std::endl;
		}
		me->SetTerrainCost(Tcosts);

		dsd.SetWeight(bound);

		dsd.InitializeSearch(me, start, goal, solution);

		dh = new GridEmbedding(me, 10, kLINF);
		for (int x = 0; x < 10; x++)
			dh->AddDimension(kDifferential, kFurthest);
		if(useDH)
			dsd.SetHeuristic(dh);

		searchRunning = true;
	}
}
//#include "Plot2D.h"
struct DSDdata {
	float slope;
	float weight;
	float K;
	point3d crossPoint; // cached for simplicity
};
std::vector<DSDdata> data;
point3d zero(0, 0);
point3d origin(-1, 1);

point3d HOGToLocal(point3d p)
{
	return point3d((p.x+1)*bound/2.0f , (p.y-1)*bound/-2.0);
}

point3d LocalToHOG(point3d p)
{
	return point3d(2.0f*(p.x/bound-0.5), -2.0*(p.y/bound-0.5));
}

// 1. resize window to 1024x768
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;

	if (viewport == 0)
	{
		if (me)
			me->Draw(display);
		if (searchRunning)
		{
			for (int x = 0; x < stepsPerFrame; x++)
			{
				if (solution.size() == 0)
				{
					// if (dsd.DoSingleSearchStep(solution, true))
					if (dsd.DoSingleSearchStep(solution))
						std::cout << "Node Expansions: " << dsd.GetNodesExpanded() << "\n";
				}
			}
			dsd.Draw(display);
		}
	}
	if (viewport == 1)
	{
		if (searchRunning)
		{
			dsd.DrawPriorityGraph(display);
		}
	}
	
	if (viewport == 3) // TODO: Add mode for exploration
	{
		if (showPlane)
		{
			//		float divisor = GetPriority(bound, bound);
			float divisor = GetPriority(1, 0);
			if (isinf(divisor))
				divisor = 4;
			const float delta = 0.015;//0.025;
			for (float x = 0; x < bound; x+=delta)
			{
				for (float y = 0; y < bound; y+=delta)
				{
					display.FillSquare(LocalToHOG({x, y}), delta/2, rgbColor::hsl(fmodf(GetPriority(x, y)/divisor, 1.0f), 1.0, 0.5));
				}
			}
		}
		
		//	point3d lastCrossPoint(1/bound, 0);
		float priority = 1;//HOGToLocal({-1, -1}).y;//2.0f/bound;
		
		// draw bounding line
		point3d bl1(priority, 0), bl2(0, priority*bound); // main suboptimality line
		point3d bl3(priority*bound, 0);//(0+2, priority*bound-2); //
		point3d bl4(priority*bound/(2*bound-1), 0);//priority*bound-(2*bound-1));
		point3d bl2a(0, priority);
		point3d bl2c(priority-priority*bound/(2*bound-1), priority*bound);
		// main priority line
		display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2), 1/100.0f, Colors::yellow);
		display.DrawLine(LocalToHOG(bl1*testScale), LocalToHOG(bl2*testScale), 1/100.0f, Colors::darkyellow);
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
			if (data[x].slope < 1)
			{
				value.x += 2;
				value.y -= 2*data[x].slope;
			}
			else {
				value.x += 2/data[x].slope;
				value.y -= 2;
			}
			display.DrawLine(origin, value, 1/200.0f, Colors::blue);
			
			point3d crossPoint1, crossPoint2;
			crossPoint1.x = priority/(data[x].K*(data[x].slope+data[x].weight));
			crossPoint1.y = crossPoint1.x*data[x].slope;
			float lastSlope = ((x==0)?(0):(data[x-1].slope));
			crossPoint2.x = priority/(data[x].K*(lastSlope+data[x].weight));
			crossPoint2.y = crossPoint2.x*lastSlope;
			display.DrawLine(LocalToHOG(crossPoint1), LocalToHOG(crossPoint2), 1/100.0f, Colors::red);
		}
		for (int x = 0; x < data.size(); x++)
			display.FillCircle(LocalToHOG(data[x].crossPoint), 0.01, Colors::darkgreen);
		
		display.DrawLine(origin, {1, 1}, 1./100.0f, Colors::white);
		display.DrawLine(origin, {-1, -1}, 1./100.0f, Colors::white);
	}
}

#include "MNPuzzle.h"
#include "STPInstances.h"
int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-stp") == 0)
	{
		assert(maxNumArgs >= 5);
		
		DSDWAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> dsd_mnp;
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> tas_mnp;
		std::vector<MNPuzzleState<4, 4>> path;
		MNPuzzle<4, 4> mnp;
		MNPuzzleState<4, 4> start = STP::GetKorfInstance(atoi(argument[1]));
		MNPuzzleState<4, 4> goal;

		dsd_mnp.InitializeSearch(&mnp, start, goal, path);

		dsd_mnp.policy = (tExpansionPriority)atoi(argument[2]);
		dsd_mnp.SetWeight(atof(argument[3]));
		
		printf("Solving STP Korf instance [%d of %d] using DSD weight %f\n", atoi(argument[1])+1, 100, atof(argument[3]));

		// Order of calling these functions matters.
		mnp.SetInputWeight(atof(argument[3])); //0

		mnp.SetPuzzleWeight(atoi(argument[4])); //1
		// case 0:UnitWeight, case 1:SwampedMode, case 2:SquareRoot, 
		// case 3:Squared, case 4:UnitPlusFrac, case 5:SquarePlusOneRoot

		mnp.SetMaxMinTileCost(start); //2
		
		mnp.SetNormalizedCost(false); //3

		if(exper==10){
			//Run the search once using WA* to find the solution path, and place the swamp area on that.
			//The terrainSize is a proportion of the solution length.
			prevPolicy = dsd_mnp.policy;
			dsd_mnp.policy = kWA;
			dsd_mnp.InitializeSearch(&mnp, start, goal, path);
			dsd_mnp.GetPath(&mnp, start, goal, path);
			if(path.size()){
				randomIndex = random()%path.size();
				mnp.SetMiddleState(path[randomIndex]);
				mnp.SetTerrainSize(path.size()*atof(argument[4])/100/2);
				std::cout<<"Swamped Region Created\n";
				std::cout<<"Path size is: "<<path.size()<<"\n";
				std::cout<<"Middle State at index "<<randomIndex<<" is:\n";
				// mnp.PrintState(path[randomIndex]);
			}
			else{
				mnp.SetMiddleState(start);
				mnp.SetTerrainSize(20);
			}
			dsd_mnp.policy = prevPolicy;
		}

		dsd_mnp.InitializeSearch(&mnp, start, goal, path);

		// dsd_mnp.GetPath(&mnp, start, goal, path, true);
		dsd_mnp.GetPath(&mnp, start, goal, path);

		//Save the SVG of the isoloines plots
		if(saveSVG && (dsd_mnp.policy==3)){
			Graphics::Display d;
			dsd_mnp.DrawPriorityGraph(d);
			std::string s = "stp="+ string(argument[1])+"_alg="+string(argument[2])+"_w="+string(argument[3])+".svg";
			MakeSVG(d, s.c_str(), 750,750,0);
		}
			
		printf("STP %d ALG %d weight %1.2f Nodes %llu path %lu\n", atoi(argument[1]), atoi(argument[2]), atof(argument[3]), dsd_mnp.GetNodesExpanded(), path.size());
			
		
		exit(0);
	}
	else if (strcmp(argument[0], "-stpBLs") == 0)
	{
		assert(maxNumArgs >= 5);
		
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> tas_mnp;
		std::vector<MNPuzzleState<4, 4>> path;
		MNPuzzle<4, 4> mnp;
		MNPuzzleState<4, 4> start = STP::GetKorfInstance(atoi(argument[1]));
		MNPuzzleState<4, 4> goal;

		double proveBound = atof(argument[3]);
		tas_mnp.SetWeight(proveBound);

		if(atoi(argument[2]) == 0){ //WA*
		tas_mnp.SetPhi([=](double h,double g){return g+proveBound*h;});
		}
		else if(atoi(argument[2]) == 1){ //PWXD
		tas_mnp.SetPhi([=](double h,double g){return (h>g)?(g+h):(g/proveBound+h*(2*proveBound-1)/proveBound);});
		}
		else if(atoi(argument[2]) == 2){ //PWXU
		tas_mnp.SetPhi([=](double h,double g){return (h*(2*proveBound-1)>g)?(g/(2*proveBound-1)+h):(1/proveBound*(g+h));});
		}
		else if(atoi(argument[2]) == 3){ //XDP
		tas_mnp.SetPhi([=](double h,double g){return (g+(2*proveBound-1)*h+sqrt((g-h)*(g-h)+4*proveBound*g*h))/(2*proveBound);});
		}
		else if(atoi(argument[2]) == 4){ //XUP
		tas_mnp.SetPhi([=](double h,double g){return (g+h+sqrt((g+h)*(g+h)+4*proveBound*(proveBound-1)*h*h))/(2*proveBound);});
		}
		
		// printf("Solving STP Korf instance [%d of %d] using DSD weight %f\n", atoi(argument[1])+1, 100, atof(argument[3]));

		// Order of calling these functions matters.
		mnp.SetInputWeight(atof(argument[3])); //0

		mnp.SetPuzzleWeight(atoi(argument[4])); //1
		// case 0:UnitWeight, case 1:SwampedMode, case 2:SquareRoot, 
		// case 3:Squared, case 4:UnitPlusFrac, case 5:SquarePlusOneRoot

		mnp.SetMaxMinTileCost(start); //2
		
		mnp.SetNormalizedCost(false); //3

		tas_mnp.InitializeSearch(&mnp, start, goal, path);

		// dsd_mnp.GetPath(&mnp, start, goal, path, true);
		tas_mnp.GetPath(&mnp, start, goal, path);

		//Save the SVG of the isoloines plots
		// if(saveSVG && (atoi(argument[2])==3)){
		// 	Graphics::Display d;
		// 	tas_mnp.DrawPriorityGraph(d);
		// 	std::string s = "stp="+ string(argument[1])+"_alg="+string(argument[2])+"_w="+string(argument[3])+".svg";
		// 	MakeSVG(d, s.c_str(), 750,750,0);
		// }
			
		printf("STP %d ALG %d weight %1.2f Nodes %llu path %lu\n", atoi(argument[1]), atoi(argument[2]), atof(argument[3]), tas_mnp.GetNodesExpanded(), path.size());
			
		
		exit(0);
	}
	else if (strcmp(argument[0], "-gridBLs") == 0){
		// Runs all five baselines.
		assert(maxNumArgs >= 6);
		me = new MapEnvironment(new Map(argument[1]));

		me->SetDiagonalCost(1.5);
		swampedloc = {1,1};
		swampedloc2 = {1,1};
		swampedloc3 = {1,1};
		swampedloc4 = {1,1};

		ScenarioLoader sl(argument[2]);
		
		for (int x = 0; x < sl.GetNumExperiments(); x++)
		{
			//Reset the last problem's swamped states.
			{
			for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
				for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
					if(me->GetMap()->GetTerrainType(i, j) == kSwamp)
						me->GetMap()->SetTerrainType(i, j, kGround);
			for(int i=swampedloc2.x-int(tsx2/2); i<=swampedloc2.x+int(tsx2/2); i++)
				for(int j=swampedloc2.y-int(tsy2/2); j<=swampedloc2.y+int(tsy2/2); j++)
					if(me->GetMap()->GetTerrainType(i, j) == kWater)
						me->GetMap()->SetTerrainType(i, j, kGround);
			for(int i=swampedloc3.x-int(tsx3/2); i<=swampedloc3.x+int(tsx3/2); i++)
				for(int j=swampedloc3.y-int(tsy3/2); j<=swampedloc3.y+int(tsy3/2); j++)
					if(me->GetMap()->GetTerrainType(i, j) == kGrass)
						me->GetMap()->SetTerrainType(i, j, kGround);
			for(int i=swampedloc4.x-int(tsx4/2); i<=swampedloc4.x+int(tsx4/2); i++)
				for(int j=swampedloc4.y-int(tsy4/2); j<=swampedloc4.y+int(tsy4/2); j++)
					if(me->GetMap()->GetTerrainType(i, j) == kTrees)
						me->GetMap()->SetTerrainType(i, j, kGround);
			}

			Experiment exp = sl.GetNthExperiment(x);
			if(exp.GetDistance()<30) continue;

			//Set the start and goal states, weight and policy of the search. 
			start.x = exp.GetStartX();
			start.y = exp.GetStartY();
			goal.x = exp.GetGoalX();
			goal.y = exp.GetGoalY();

			double proveBound = atof(argument[4]);
			tas.SetWeight(proveBound);
			me->SetInputWeight(atof(argument[4]));
			

			//Set the cost of each terrain type randomly.
			for(int i=0; i<4; i++){
				//[0]=kSwamp, [1]=kWater, [2]=kGrass, [3]=kTrees

				// Define the Hardness
				// 1. Random Hardness
				// rdm = random()%101;
				// hardness[i] = rdm/101+1;
				//Or 2. Hardcoded Hardness
				// hardness[0]=1.1; hardness[1]=1.45; hardness[2]=1.25; hardness[3]=1.95;
				//Or 3. Get Hardness from Input Argument
				hardness[0]=atof(argument[6]);/*Followings are "Don't Care":*/hardness[1]=100.0; hardness[2]=100.0; hardness[3]=100.0;

				// Use Hardness to define Cost
				// 1. Hardness with respect to input w
				// Tcosts[i] = hardness[i]*(me->GetInputWeight())-(hardness[i]-1);
				//Or 2. Hardness as the exact Cost
				Tcosts[i] = hardness[i];

				string type;
				if(i==0) type="Swamp";
				if(i==1) type="Water";
				if(i==2) type="Grass";
				if(i==3) type="Trees";
				// std::cout<<"Cost "<<type<<"="<<Tcosts[i]<<" ("<<hardness[i]<<"*"<<me->GetInputWeight()<<"-"<<(hardness[i]-1)<<")"<<std::endl;
			}
			me->SetTerrainCost(Tcosts);

			//Set the size of the swamp area using the tspp argument.
			{
			tspp = atof(argument[5]);
			ts = tspp/100*(abs(goal.x-start.x) + abs(goal.y-start.y));
			tsx = max(tspp/100*abs(goal.x-start.x), 20);
			tsy = max(tspp/100*abs(goal.y-start.y), 20);
			tsx = max(tsx, tsy);
			tsy = tsx;
			}
			
			//Change one or more squares of ground states to swamp type.
			if(exper==0){
				//Set the square around the start state.
				swampedloc.x = start.x;
				swampedloc.y = start.y;
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=start.x-int(tsx/2); i<=start.x+int(tsx/2); i++)
					for(int j=start.y-int(tsy/2); j<=start.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==1){
				//Set the square around the goal state.
				swampedloc.x = goal.x;
				swampedloc.y = goal.y;
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=goal.x-int(tsx/2); i<=goal.x+int(tsx/2); i++)
					for(int j=goal.y-int(tsy/2); j<=goal.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);

			}
			else if(exper==2){
				//Set the square around a random state with a center on the [start-goal] line.
				if(goal.x == start.x) swampedloc.x = start.x;
				else swampedloc.x = random()%abs(goal.x-start.x) + min(goal.x, start.x);
				a = float(goal.y - start.y)/(goal.x-start.x);
				b = goal.y - a * goal.x;
				swampedloc.y = uint16_t(a * swampedloc.x + b);
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==3){
				//Set the square around a random state with a center on the (start+10%)-(goal-10%) line.
				if(goal.x == start.x) swampedloc.x = start.x;
				else swampedloc.x = random()%(abs(goal.x-start.x) - 2*(int(0.1*abs(goal.x-start.x))+int(tsx/2))) + min(goal.x, start.x) +int(0.1*abs(goal.x-start.x))+int(tsx/2);
				a = float(goal.y - start.y)/(goal.x-start.x);
				b = goal.y - a * goal.x;
				swampedloc.y = uint16_t(a * swampedloc.x + b);
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==4){
				//Run the search once using WA* to find the solution path, and place the swamp area on that.
				tas.SetPhi([=](double h,double g){return g+proveBound*h;}); //WA*
				tas.InitializeSearch(me, start, goal, solution);

				dh = new GridEmbedding(me, 10, kLINF);
				for (int x = 0; x < 10; x++)
					dh->AddDimension(kDifferential, kFurthest);
				if(useDH)
					dsd.SetHeuristic(dh);

				tas.GetPath(me, start, goal, solution);
				randomIndex = random()%solution.size();
				xyLocRandomState = solution[randomIndex];

				if(atoi(argument[3]) == 0){ //WA*
				tas.SetPhi([=](double h,double g){return g+proveBound*h;});
				}
				else if(atoi(argument[3]) == 1){ //PWXD
				tas.SetPhi([=](double h,double g){return (h>g)?(g+h):(g/proveBound+h*(2*proveBound-1)/proveBound);});
				}
				else if(atoi(argument[3]) == 2){ //PWXU
				tas.SetPhi([=](double h,double g){return (h*(2*proveBound-1)>g)?(g/(2*proveBound-1)+h):(1/proveBound*(g+h));});
				}
				else if(atoi(argument[3]) == 3){ //XDP
				tas.SetPhi([=](double h,double g){return (g+(2*proveBound-1)*h+sqrt((g-h)*(g-h)+4*proveBound*g*h))/(2*proveBound);});
				}
				else if(atoi(argument[3]) == 4){ //XUP
				tas.SetPhi([=](double h,double g){return (g+h+sqrt((g+h)*(g+h)+4*proveBound*(proveBound-1)*h*h))/(2*proveBound);});
				}

				//Set the square around a random state with a center on the solution path.
				swampedloc.x = xyLocRandomState.x;
				swampedloc.y = xyLocRandomState.y;

				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==5){
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				//Run the search once, to find the solution path using WA*, and place the swamp area on that.
				//It leaves a margin between the start state and the goal state of the solution.
				prevPolicy = dsd.policy;
				dsd.policy = kWA;
				dsd.InitializeSearch(me, start, goal, solution);
				dsd.GetPath(me, start, goal, solution);
				randomIndex = random()%(int(solution.size()-tsx))+tsx/2;
				xyLocRandomState = solution[randomIndex];
				dsd.policy = prevPolicy;

				//Set the square around a random state with a center on the solution path.
				swampedloc.x = xyLocRandomState.x;
				swampedloc.y = xyLocRandomState.y;
				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==6){
				//Set two squares around two random states on the start-goal line.
				//swampedloc.x = random()%abs(goal.x-start.x) + min(goal.x, start.x);
				if(goal.x == start.x) swampedloc.x = start.x;
				else swampedloc.x = random()%(abs(goal.x-start.x)/2) + min(goal.x, start.x);
				a = float(goal.y - start.y)/(goal.x-start.x);
				b = goal.y - a * goal.x;
				if(goal.x == start.x) swampedloc.x = start.x;
				else swampedloc.y = uint16_t(a * swampedloc.x + b);
				// do{
				// 	swampedloc2.x = random()%abs(goal.x-start.x) + min(goal.x, start.x);
				// } while(abs(swampedloc.x - swampedloc2.x)<ts/5);
				if(goal.x = start.x) swampedloc2.x = start.x;
				else swampedloc2.x = max(goal.x, start.x) - random()%(abs(goal.x-start.x)/2);

				a = float(goal.y - start.y)/(goal.x-start.x);
				b = goal.y - a * goal.x;
				swampedloc2.y = uint16_t(a * swampedloc2.x + b);
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=swampedloc.x-int(tsx/4); i<=swampedloc.x+int(tsx/4); i++)
					for(int j=swampedloc.y-int(tsy/4); j<=swampedloc.y+int(tsy/4); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
				for(int i=swampedloc2.x-int(tsx/4); i<=swampedloc2.x+int(tsx/4); i++)
					for(int j=swampedloc2.y-int(tsy/4); j<=swampedloc2.y+int(tsy/4); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==7){
				//Run the search once, to find the solution path using WA*, and place the swamp area on that.
				prevPolicy = dsd.policy;
				dsd.policy = kWA;
				dsd.InitializeSearch(me, start, goal, solution);
				dsd.GetPath(me, start, goal, solution);
				dsd.policy = prevPolicy;

				//Set 4 centers for Terrain Regions
				randomIndex = random()%(solution.size()/4);
				xyLocRandomState = solution[randomIndex];
				swampedloc.x = xyLocRandomState.x;
				swampedloc.y = xyLocRandomState.y;
				tsx = max(random()%int(ts/4), ts/10);
				tsy = tsx;

				randomIndex = random()%(solution.size()/4) + (solution.size()/4);
				xyLocRandomState = solution[randomIndex];
				swampedloc2.x = xyLocRandomState.x;
				swampedloc2.y = xyLocRandomState.y;
				tsx2 = max(random()%int(ts/4), ts/10);
				tsy2 = tsx2;

				randomIndex = random()%(solution.size()/4) + 2*(solution.size()/4);
				xyLocRandomState = solution[randomIndex];
				swampedloc3.x = xyLocRandomState.x;
				swampedloc3.y = xyLocRandomState.y;
				tsx3 = max(random()%int(ts/4), ts/10);
				tsy3 = tsx3;

				randomIndex = random()%(solution.size()/4) + 3*(solution.size()/4);
				xyLocRandomState = solution[randomIndex];
				swampedloc4.x = xyLocRandomState.x;
				swampedloc4.y = xyLocRandomState.y;
				tsx4 = max(random()%int(ts/4), ts/10);
				tsy4 = tsx4;

				//Set the square around a random state with a center on the solution path.
				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
				for(int i=swampedloc2.x-int(tsx2/2); i<=swampedloc2.x+int(tsx2/2); i++)
					for(int j=swampedloc2.y-int(tsy2/2); j<=swampedloc2.y+int(tsy2/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kWater);
				for(int i=swampedloc3.x-int(tsx3/2); i<=swampedloc3.x+int(tsx3/2); i++)
					for(int j=swampedloc3.y-int(tsy3/2); j<=swampedloc3.y+int(tsy3/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kGrass);
				for(int i=swampedloc4.x-int(tsx4/2); i<=swampedloc4.x+int(tsx4/2); i++)
					for(int j=swampedloc4.y-int(tsy4/2); j<=swampedloc4.y+int(tsy4/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kTrees);
			}
			
			tas.InitializeSearch(me, start, goal, solution);

			dh = new GridEmbedding(me, 10, kLINF);
			for (int x = 0; x < 10; x++)
				dh->AddDimension(kDifferential, kFurthest);
			if(useDH)
				dsd.SetHeuristic(dh);

			tas.GetPath(me, start, goal, solution);
			printf("MAP %s #%d %1.2f ALG %d weight %1.2f Nodes %llu path %f\n", argument[1], x, exp.GetDistance(), atoi(argument[3]), atof(argument[4]), tas.GetNodesExpanded(), me->GetPathLength(solution));

		}
		exit(0);
	}
	else if (strcmp(argument[0], "-DSMAP") == 0){
		// Arguments: -DSMAP mapAddress $scen $alg $weight $TerrainSize $SwampHardness

		//Thie is Prior Knowledge Map.
		//The knowledge we have prior to the search about the domain, is it's a dynamic weighted grid map.
		//Some Terrain Types exist that makes action costs different in different parts of the map.
		assert(maxNumArgs >= 6);
		me = new MapEnvironment(new Map(argument[1]));

		me->SetDiagonalCost(1.5);
		swampedloc = {1,1};
		swampedloc2 = {1,1};
		swampedloc3 = {1,1};
		swampedloc4 = {1,1};

		ScenarioLoader sl(argument[2]);
		
		for (int x = 0; x < sl.GetNumExperiments(); x++)
		{
			//Reset the last problem's swamped states.
			{
			for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
				for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
					if(me->GetMap()->GetTerrainType(i, j) == kSwamp)
						me->GetMap()->SetTerrainType(i, j, kGround);
			for(int i=swampedloc2.x-int(tsx2/2); i<=swampedloc2.x+int(tsx2/2); i++)
				for(int j=swampedloc2.y-int(tsy2/2); j<=swampedloc2.y+int(tsy2/2); j++)
					if(me->GetMap()->GetTerrainType(i, j) == kWater)
						me->GetMap()->SetTerrainType(i, j, kGround);
			for(int i=swampedloc3.x-int(tsx3/2); i<=swampedloc3.x+int(tsx3/2); i++)
				for(int j=swampedloc3.y-int(tsy3/2); j<=swampedloc3.y+int(tsy3/2); j++)
					if(me->GetMap()->GetTerrainType(i, j) == kGrass)
						me->GetMap()->SetTerrainType(i, j, kGround);
			for(int i=swampedloc4.x-int(tsx4/2); i<=swampedloc4.x+int(tsx4/2); i++)
				for(int j=swampedloc4.y-int(tsy4/2); j<=swampedloc4.y+int(tsy4/2); j++)
					if(me->GetMap()->GetTerrainType(i, j) == kTrees)
						me->GetMap()->SetTerrainType(i, j, kGround);
			}

			Experiment exp = sl.GetNthExperiment(x);
			if(exp.GetDistance()<30) continue;

			//Set the start and goal states, weight and policy of the search. 
			{
			start.x = exp.GetStartX();
			start.y = exp.GetStartY();
			goal.x = exp.GetGoalX();
			goal.y = exp.GetGoalY();
			dsd.policy = (tExpansionPriority)atoi(argument[3]);
			dsd.SetWeight(atof(argument[4]));

			me->SetInputWeight(atof(argument[4]));
			}

			//Set the cost of each terrain type randomly.
			for(int i=0; i<4; i++){
				//[0]=kSwamp, [1]=kWater, [2]=kGrass, [3]=kTrees

				// Define the Hardness
				// 1. Random Hardness
				// rdm = random()%101;
				// hardness[i] = rdm/101+1;
				//Or 2. Hardcoded Hardness
				// hardness[0]=1.1; hardness[1]=1.45; hardness[2]=1.25; hardness[3]=1.95;
				//Or 3. Get Hardness from Input Argument
				hardness[0]=atof(argument[6]);/*Followings are "Don't Care":*/hardness[1]=100.0; hardness[2]=100.0; hardness[3]=100.0;

				// Use Hardness to define Cost
				// 1. Hardness with respect to input w
				// Tcosts[i] = hardness[i]*(me->GetInputWeight())-(hardness[i]-1);
				//Or 2. Hardness as the exact Cost
				Tcosts[i] = hardness[i];

				string type;
				if(i==0) type="Swamp";
				if(i==1) type="Water";
				if(i==2) type="Grass";
				if(i==3) type="Trees";
				// std::cout<<"Cost "<<type<<"="<<Tcosts[i]<<" ("<<hardness[i]<<"*"<<me->GetInputWeight()<<"-"<<(hardness[i]-1)<<")"<<std::endl;
			}
			me->SetTerrainCost(Tcosts);

			//Set the size of the swamp area using the tspp argument.
			{
			tspp = atof(argument[5]);
			ts = tspp/100*(abs(goal.x-start.x) + abs(goal.y-start.y));
			tsx = max(tspp/100*abs(goal.x-start.x), 20);
			tsy = max(tspp/100*abs(goal.y-start.y), 20);
			tsx = max(tsx, tsy);
			tsy = tsx;
			}
			
			//Change one or more squares of ground states to swamp type.
			if(exper==0){
				//Set the square around the start state.
				swampedloc.x = start.x;
				swampedloc.y = start.y;
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=start.x-int(tsx/2); i<=start.x+int(tsx/2); i++)
					for(int j=start.y-int(tsy/2); j<=start.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==1){
				//Set the square around the goal state.
				swampedloc.x = goal.x;
				swampedloc.y = goal.y;
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=goal.x-int(tsx/2); i<=goal.x+int(tsx/2); i++)
					for(int j=goal.y-int(tsy/2); j<=goal.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);

			}
			else if(exper==2){
				//Set the square around a random state with a center on the [start-goal] line.
				if(goal.x == start.x) swampedloc.x = start.x;
				else swampedloc.x = random()%abs(goal.x-start.x) + min(goal.x, start.x);
				a = float(goal.y - start.y)/(goal.x-start.x);
				b = goal.y - a * goal.x;
				swampedloc.y = uint16_t(a * swampedloc.x + b);
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==3){
				//Set the square around a random state with a center on the (start+10%)-(goal-10%) line.
				if(goal.x == start.x) swampedloc.x = start.x;
				else swampedloc.x = random()%(abs(goal.x-start.x) - 2*(int(0.1*abs(goal.x-start.x))+int(tsx/2))) + min(goal.x, start.x) +int(0.1*abs(goal.x-start.x))+int(tsx/2);
				a = float(goal.y - start.y)/(goal.x-start.x);
				b = goal.y - a * goal.x;
				swampedloc.y = uint16_t(a * swampedloc.x + b);
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==4){
				//Run the search once using WA* to find the solution path, and place the swamp area on that.
				prevPolicy = dsd.policy;
				dsd.policy = kWA;
				dsd.InitializeSearch(me, start, goal, solution);

				dh = new GridEmbedding(me, 10, kLINF);
				for (int x = 0; x < 10; x++)
					dh->AddDimension(kDifferential, kFurthest);
				if(useDH)
					dsd.SetHeuristic(dh);

				dsd.GetPath(me, start, goal, solution);
				randomIndex = random()%solution.size();
				xyLocRandomState = solution[randomIndex];
				dsd.policy = prevPolicy;

				//Set the square around a random state with a center on the solution path.
				swampedloc.x = xyLocRandomState.x;
				swampedloc.y = xyLocRandomState.y;

				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==5){
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				//Run the search once, to find the solution path using WA*, and place the swamp area on that.
				//It leaves a margin between the start state and the goal state of the solution.
				prevPolicy = dsd.policy;
				dsd.policy = kWA;
				dsd.InitializeSearch(me, start, goal, solution);
				dsd.GetPath(me, start, goal, solution);
				randomIndex = random()%(int(solution.size()-tsx))+tsx/2;
				xyLocRandomState = solution[randomIndex];
				dsd.policy = prevPolicy;

				//Set the square around a random state with a center on the solution path.
				swampedloc.x = xyLocRandomState.x;
				swampedloc.y = xyLocRandomState.y;
				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==6){
				//Set two squares around two random states on the start-goal line.
				//swampedloc.x = random()%abs(goal.x-start.x) + min(goal.x, start.x);
				if(goal.x == start.x) swampedloc.x = start.x;
				else swampedloc.x = random()%(abs(goal.x-start.x)/2) + min(goal.x, start.x);
				a = float(goal.y - start.y)/(goal.x-start.x);
				b = goal.y - a * goal.x;
				if(goal.x == start.x) swampedloc.x = start.x;
				else swampedloc.y = uint16_t(a * swampedloc.x + b);
				// do{
				// 	swampedloc2.x = random()%abs(goal.x-start.x) + min(goal.x, start.x);
				// } while(abs(swampedloc.x - swampedloc2.x)<ts/5);
				if(goal.x = start.x) swampedloc2.x = start.x;
				else swampedloc2.x = max(goal.x, start.x) - random()%(abs(goal.x-start.x)/2);

				a = float(goal.y - start.y)/(goal.x-start.x);
				b = goal.y - a * goal.x;
				swampedloc2.y = uint16_t(a * swampedloc2.x + b);
				tsx = max(tsx, 10);
				tsy = max(tsy, 10);

				tsx = max(tsx, tsy);
				tsy = tsx;

				for(int i=swampedloc.x-int(tsx/4); i<=swampedloc.x+int(tsx/4); i++)
					for(int j=swampedloc.y-int(tsy/4); j<=swampedloc.y+int(tsy/4); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
				for(int i=swampedloc2.x-int(tsx/4); i<=swampedloc2.x+int(tsx/4); i++)
					for(int j=swampedloc2.y-int(tsy/4); j<=swampedloc2.y+int(tsy/4); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
			}
			else if(exper==7){
				//Run the search once, to find the solution path using WA*, and place the swamp area on that.
				prevPolicy = dsd.policy;
				dsd.policy = kWA;
				dsd.InitializeSearch(me, start, goal, solution);
				dsd.GetPath(me, start, goal, solution);
				dsd.policy = prevPolicy;

				//Set 4 centers for Terrain Regions
				randomIndex = random()%(solution.size()/4);
				xyLocRandomState = solution[randomIndex];
				swampedloc.x = xyLocRandomState.x;
				swampedloc.y = xyLocRandomState.y;
				tsx = max(random()%int(ts/4), ts/10);
				tsy = tsx;

				randomIndex = random()%(solution.size()/4) + (solution.size()/4);
				xyLocRandomState = solution[randomIndex];
				swampedloc2.x = xyLocRandomState.x;
				swampedloc2.y = xyLocRandomState.y;
				tsx2 = max(random()%int(ts/4), ts/10);
				tsy2 = tsx2;

				randomIndex = random()%(solution.size()/4) + 2*(solution.size()/4);
				xyLocRandomState = solution[randomIndex];
				swampedloc3.x = xyLocRandomState.x;
				swampedloc3.y = xyLocRandomState.y;
				tsx3 = max(random()%int(ts/4), ts/10);
				tsy3 = tsx3;

				randomIndex = random()%(solution.size()/4) + 3*(solution.size()/4);
				xyLocRandomState = solution[randomIndex];
				swampedloc4.x = xyLocRandomState.x;
				swampedloc4.y = xyLocRandomState.y;
				tsx4 = max(random()%int(ts/4), ts/10);
				tsy4 = tsx4;

				//Set the square around a random state with a center on the solution path.
				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kSwamp);
				for(int i=swampedloc2.x-int(tsx2/2); i<=swampedloc2.x+int(tsx2/2); i++)
					for(int j=swampedloc2.y-int(tsy2/2); j<=swampedloc2.y+int(tsy2/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kWater);
				for(int i=swampedloc3.x-int(tsx3/2); i<=swampedloc3.x+int(tsx3/2); i++)
					for(int j=swampedloc3.y-int(tsy3/2); j<=swampedloc3.y+int(tsy3/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kGrass);
				for(int i=swampedloc4.x-int(tsx4/2); i<=swampedloc4.x+int(tsx4/2); i++)
					for(int j=swampedloc4.y-int(tsy4/2); j<=swampedloc4.y+int(tsy4/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGround)
							me->GetMap()->SetTerrainType(i, j, kTrees);
			}
			
			dsd.InitializeSearch(me, start, goal, solution);

			dh = new GridEmbedding(me, 10, kLINF);
			for (int x = 0; x < 10; x++)
				dh->AddDimension(kDifferential, kFurthest);
			if(useDH)
				dsd.SetHeuristic(dh);
			
			dsd.GetPath(me, start, goal, solution);
			printf("MAP %s #%d %1.2f ALG %d weight %1.2f Nodes %llu path %f\n", argument[1], x, exp.GetDistance(), atoi(argument[3]), atof(argument[4]), dsd.GetNodesExpanded(), me->GetPathLength(solution));
		}
		exit(0);
	}
	else if (strcmp(argument[0], "-exp0") == 0)
	{
		Map *m = new Map(200, 200);
		MakeDesignedMap(m, atoi(argument[4]), atoi(argument[5]));
		// std::string filename = argument[1];
		// m->Save(filename.c_str());
		assert(maxNumArgs >= 5);
		me = new MapEnvironment(m);

		start = {1,1};
		goal = {198, 198};
		dsd.policy = (tExpansionPriority)atoi(argument[2]);
		dsd.SetWeight(atof(argument[3]));
		dsd.GetPath(me, start, goal, solution);
		printf("MAP %s #%d %1.2f ALG %d weight %1.2f Nodes %llu path %f\n", argument[1], "dummy", 0.0, atoi(argument[2]), atof(argument[3]), dsd.GetNodesExpanded(), me->GetPathLength(solution));
		exit(0);
	}
	else if (strcmp(argument[0], "-DPstp") == 0)
	{
		assert(maxNumArgs >= 3);
		
		DynamicPotentialSearch<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> dsd_mnp;
		std::vector<MNPuzzleState<4, 4>> path;
		MNPuzzle<4, 4> mnp;
		MNPuzzleState<4, 4> start = STP::GetKorfInstance(atoi(argument[1]));
		MNPuzzleState<4, 4> goal;
		// dsd_mnp.policy = (tExpansionPriority)atoi(argument[2]);
		// dsd_mnp.SetWeight(atof(argument[3]));
		dsd_mnp.SetOptimalityBound(atof(argument[2]));
		printf("Solving STP Korf instance [%d of %d] using DSD weight %f\n", atoi(argument[1])+1, 100, atof(argument[2]));
		// dsd_mnp.GetPath(&mnp, start, goal, path, true);
		dsd_mnp.GetPath(&mnp, start, goal, path);
		printf("STP %d dummy %d weight %1.2f Nodes %llu path %lu\n", atoi(argument[1]), 0, atof(argument[2]), dsd_mnp.GetNodesExpanded(), path.size());
		exit(0);
	}
	else if (strcmp(argument[0], "-timeDSWA") == 0)
	{
		assert(maxNumArgs >= 4);
		
		DSDWAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> dsd_mnp;
		std::vector<MNPuzzleState<4, 4>> path;
		MNPuzzle<4, 4> mnp;
		MNPuzzleState<4, 4> start = STP::GetKorfInstance(atoi(argument[1]));
		MNPuzzleState<4, 4> goal;
		dsd_mnp.policy = (tExpansionPriority)atoi(argument[2]);
		dsd_mnp.SetWeight(atof(argument[3]));
		printf("Solving STP Korf instance [%d of %d] using DSD weight %f\n", atoi(argument[1])+1, 100, atof(argument[3]));
		
		clock_t start_time, end_time;
    	start_time = clock();

		// dsd_mnp.GetPath(&mnp, start, goal, path, true);
		dsd_mnp.GetPath(&mnp, start, goal, path);

		end_time = clock();
		float runningTime = (float) (end_time - start_time) / CLOCKS_PER_SEC;

		printf("STP %d ALG %d weight %1.2f nodePERsec %f path %lu\n", atoi(argument[1]), atoi(argument[2]), atof(argument[3]), dsd_mnp.GetNodesExpanded()/runningTime, path.size());
		exit(0);
	}
	else if (strcmp(argument[0], "-stpAstar") == 0)
	{
		assert(maxNumArgs >= 4);
		
		TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> dsd_mnp;
		std::vector<MNPuzzleState<4, 4>> path;
		MNPuzzle<4, 4> mnp;
		MNPuzzleState<4, 4> start = STP::GetKorfInstance(atoi(argument[1]));
		MNPuzzleState<4, 4> goal;
		// dsd_mnp.policy = (tExpansionPriority)atoi(argument[2]);
		dsd_mnp.SetWeight(atof(argument[3]));
		printf("Solving STP Korf instance [%d of %d] using DSD weight %f\n", atoi(argument[1])+1, 100, atof(argument[3]));

		clock_t start_time, end_time;
    	start_time = clock();

		// dsd_mnp.GetPath(&mnp, start, goal, path, true);
		dsd_mnp.GetPath(&mnp, start, goal, path);

		end_time = clock();
		float runningTime = (float) (end_time - start_time) / CLOCKS_PER_SEC;

		printf("STP %d ALG %d weight %1.2f nodePERsec %f path %lu\n", atoi(argument[1]), atoi(argument[2]), atof(argument[3]), dsd_mnp.GetNodesExpanded()/runningTime, path.size());
		exit(0);
	}
	else if (strcmp(argument[0], "-map") == 0)
	{
		assert(maxNumArgs >= 5);
		me = new MapEnvironment(new Map(argument[1]));
		ScenarioLoader sl(argument[2]);
		// std::cout<<"number of experiments is "<<sl.GetNumExperiments()<<std::endl;
		for (int x = 0; x < sl.GetNumExperiments(); x++)
		{
			Experiment exp = sl.GetNthExperiment(x);
			start.x = exp.GetStartX();
			start.y = exp.GetStartY();
			goal.x = exp.GetGoalX();
			goal.y = exp.GetGoalY();
			dsd.policy = (tExpansionPriority)atoi(argument[3]);
			dsd.SetWeight(atof(argument[4]));
			// dsd.GetPath(me, start, goal, solution, true);
			dsd.GetPath(me, start, goal, solution);
			printf("MAP %s #%d %1.2f ALG %d weight %1.2f Nodes %llu path %f\n", argument[1], x, exp.GetDistance(), atoi(argument[3]), atof(argument[4]), dsd.GetNodesExpanded(), me->GetPathLength(solution));
		}
		exit(0);
	}
	
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 's':
			{
				//Reset the last problem's swamped states.
				for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
					for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kSwamp)
							me->GetMap()->SetTerrainType(i, j, kGround);
				for(int i=swampedloc2.x-int(tsx2/2); i<=swampedloc2.x+int(tsx2/2); i++)
					for(int j=swampedloc2.y-int(tsy2/2); j<=swampedloc2.y+int(tsy2/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kWater)
							me->GetMap()->SetTerrainType(i, j, kGround);
				for(int i=swampedloc3.x-int(tsx3/2); i<=swampedloc3.x+int(tsx3/2); i++)
					for(int j=swampedloc3.y-int(tsy3/2); j<=swampedloc3.y+int(tsy3/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kGrass)
							me->GetMap()->SetTerrainType(i, j, kGround);
				for(int i=swampedloc4.x-int(tsx4/2); i<=swampedloc4.x+int(tsx4/2); i++)
					for(int j=swampedloc4.y-int(tsy4/2); j<=swampedloc4.y+int(tsy4/2); j++)
						if(me->GetMap()->GetTerrainType(i, j) == kTrees)
							me->GetMap()->SetTerrainType(i, j, kGround);

				//Set the start and goal states of the search.
				do {
					start.x = random()%me->GetMap()->GetMapWidth();
					start.y = random()%me->GetMap()->GetMapHeight();
				} while (me->GetMap()->GetTerrainType(start.x, start.y) != kGround);
				do {
					goal.x = random()%me->GetMap()->GetMapWidth();
					goal.y = random()%me->GetMap()->GetMapHeight();
				} while (me->GetMap()->GetTerrainType(goal.x, goal.y) != kGround);

				me->SetInputWeight(bound);

				//Change one or more squares of ground states to swamp type.
				if(exper==0){
					//Set the square around the start state.
					swampedloc.x = start.x;
					swampedloc.y = start.y;
					tsx = max(tsx, 10);
					tsy = max(tsy, 10);

					tsx = max(tsx, tsy);
					tsy = tsx;

					for(int i=start.x-int(tsx/2); i<=start.x+int(tsx/2); i++)
						for(int j=start.y-int(tsy/2); j<=start.y+int(tsy/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);
				}
				else if(exper==1){
					//Set the square around the goal state.
					swampedloc.x = goal.x;
					swampedloc.y = goal.y;
					tsx = max(tsx, 10);
					tsy = max(tsy, 10);

					tsx = max(tsx, tsy);
					tsy = tsx;

					for(int i=goal.x-int(tsx/2); i<=goal.x+int(tsx/2); i++)
						for(int j=goal.y-int(tsy/2); j<=goal.y+int(tsy/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);

				}
				else if(exper==2){
					//Set the square around a random state with a center on the [start-goal] line.
					if(goal.x == start.x) swampedloc.x = start.x;
					else swampedloc.x = random()%abs(goal.x-start.x) + min(goal.x, start.x);
					a = float(goal.y - start.y)/(goal.x-start.x);
					b = goal.y - a * goal.x;
					swampedloc.y = uint16_t(a * swampedloc.x + b);
					tsx = max(tsx, 10);
					tsy = max(tsy, 10);

					tsx = max(tsx, tsy);
					tsy = tsx;

					for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
						for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);
				}
				else if(exper==3){
					//Set the square around a random state with a center on the (start+10%)-(goal-10%) line.
					if(goal.x == start.x) swampedloc.x = start.x;
					else swampedloc.x = random()%(abs(goal.x-start.x) - 2*(int(0.1*abs(goal.x-start.x))+int(tsx/2))) + min(goal.x, start.x) +int(0.1*abs(goal.x-start.x))+int(tsx/2);
					a = float(goal.y - start.y)/(goal.x-start.x);
					b = goal.y - a * goal.x;
					swampedloc.y = uint16_t(a * swampedloc.x + b);
					tsx = max(tsx, 10);
					tsy = max(tsy, 10);

					tsx = max(tsx, tsy);
					tsy = tsx;

					for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
						for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);
				}
				else if(exper==4){
					//Run the search once, to find the solution path using WA*, and place the swamp area on that.
					prevPolicy = dsd.policy;
					dsd.policy = kWA;
					dsd.InitializeSearch(me, start, goal, solution);
					dsd.GetPath(me, start, goal, solution);
					randomIndex = random()%solution.size();
					xyLocRandomState = solution[randomIndex];
					dsd.policy = prevPolicy;

					//Set the size of the swamp area using the tspp argument.
					ts = tspp/100*(abs(goal.x-start.x) + abs(goal.y-start.y));
					// ts = tspp/100*(solution.size());
					tsx = max(tspp/100*abs(goal.x-start.x), 10);
					tsy = max(tspp/100*abs(goal.y-start.y), 10);
					tsx = max(tsx, tsy);
					tsy = tsx;

					//Set the square around a random state with a center on the solution path.
					swampedloc.x = xyLocRandomState.x;
					swampedloc.y = xyLocRandomState.y;

					for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
						for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);
				}
				else if(exper==5){
					//Run the search once, to find the solution path using WA*, and place the swamp area on that.
					//It leaves a margin between the start state and the goal state of the solution.
					prevPolicy = dsd.policy;
					dsd.policy = kWA;
					dsd.InitializeSearch(me, start, goal, solution);
					dsd.GetPath(me, start, goal, solution);
					randomIndex = random()%(int(solution.size()-tsx))+tsx/2;
					xyLocRandomState = solution[randomIndex];
					dsd.policy = prevPolicy;

					//Set the square around a random state with a center on the solution path.
					swampedloc.x = xyLocRandomState.x;
					swampedloc.y = xyLocRandomState.y;
					for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
						for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);
				}
				else if(exper==6){
					//Set two squares around two random states on the start-goal line.
					//swampedloc.x = random()%abs(goal.x-start.x) + min(goal.x, start.x);
					if(goal.x == start.x) swampedloc.x = start.x;
					else swampedloc.x = random()%(abs(goal.x-start.x)/2) + min(goal.x, start.x);
					a = float(goal.y - start.y)/(goal.x-start.x);
					b = goal.y - a * goal.x;
					if(goal.x == start.x) swampedloc.x = start.x;
					else swampedloc.y = uint16_t(a * swampedloc.x + b);

					if(goal.x == start.x) swampedloc2.x = start.x;
					else swampedloc2.x = max(goal.x, start.x) - random()%(abs(goal.x-start.x)/2);

					a = float(goal.y - start.y)/(goal.x-start.x);
					b = goal.y - a * goal.x;
					swampedloc2.y = uint16_t(a * swampedloc2.x + b);

					for(int i=swampedloc.x-int(tsx/4); i<=swampedloc.x+int(tsx/4); i++)
						for(int j=swampedloc.y-int(tsy/4); j<=swampedloc.y+int(tsy/4); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);
					for(int i=swampedloc2.x-int(tsx/4); i<=swampedloc2.x+int(tsx/4); i++)
						for(int j=swampedloc2.y-int(tsy/4); j<=swampedloc2.y+int(tsy/4); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);
				}
				else if(exper==7){
					//Run the search once, to find the solution path using WA*, and place the swamp area on that.
					prevPolicy = dsd.policy;
					dsd.policy = kWA;
					dsd.InitializeSearch(me, start, goal, solution);
					dsd.GetPath(me, start, goal, solution);
					dsd.policy = prevPolicy;

					//Set 4 centers for Terrain Regions
					randomIndex = random()%(solution.size()/4);
					xyLocRandomState = solution[randomIndex];
					swampedloc.x = xyLocRandomState.x;
					swampedloc.y = xyLocRandomState.y;
					tsx = max(random()%int(ts/4), ts/10);
					tsy = tsx;

					randomIndex = random()%(solution.size()/4) + (solution.size()/4);
					xyLocRandomState = solution[randomIndex];
					swampedloc2.x = xyLocRandomState.x;
					swampedloc2.y = xyLocRandomState.y;
					tsx2 = max(random()%int(ts/4), ts/10);
					tsy2 = tsx2;

					randomIndex = random()%(solution.size()/4) + 2*(solution.size()/4);
					xyLocRandomState = solution[randomIndex];
					swampedloc3.x = xyLocRandomState.x;
					swampedloc3.y = xyLocRandomState.y;
					tsx3 = max(random()%int(ts/4), ts/10);
					tsy3 = tsx3;

					randomIndex = random()%(solution.size()/4) + 3*(solution.size()/4);
					xyLocRandomState = solution[randomIndex];
					swampedloc4.x = xyLocRandomState.x;
					swampedloc4.y = xyLocRandomState.y;
					tsx4 = max(random()%int(ts/4), ts/10);
					tsy4 = tsx4;

					//Set the square around a random state with a center on the solution path.
					for(int i=swampedloc.x-int(tsx/2); i<=swampedloc.x+int(tsx/2); i++)
						for(int j=swampedloc.y-int(tsy/2); j<=swampedloc.y+int(tsy/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kSwamp);
					for(int i=swampedloc2.x-int(tsx2/2); i<=swampedloc2.x+int(tsx2/2); i++)
						for(int j=swampedloc2.y-int(tsy2/2); j<=swampedloc2.y+int(tsy2/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kWater);
					for(int i=swampedloc3.x-int(tsx3/2); i<=swampedloc3.x+int(tsx3/2); i++)
						for(int j=swampedloc3.y-int(tsy3/2); j<=swampedloc3.y+int(tsy3/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kGrass);
					for(int i=swampedloc4.x-int(tsx4/2); i<=swampedloc4.x+int(tsx4/2); i++)
						for(int j=swampedloc4.y-int(tsy4/2); j<=swampedloc4.y+int(tsy4/2); j++)
							if(me->GetMap()->GetTerrainType(i, j) == kGround)
								me->GetMap()->SetTerrainType(i, j, kTrees);
				}

				problemNumber +=1;
				printf("==============\n");
				printf("Problem: %d\n", problemNumber);
				printf("Policy: %d\n", dsd.policy);
				printf("Bound: %f\n", bound);
				for(int i=0; i<4; i++){
					//[0]=kSwamp, [1]=kWater ,[2]=kGrass, [3]=kTrees
					string type;
					if(i==0) type="Swamp";
					if(i==1) type="Water";
					if(i==2) type="Grass";
					if(i==3) type="Trees";
					std::cout<<"Cost "<<type<<"="<<Tcosts[i]<<" ("<<hardness[i]<<"*"<<me->GetInputWeight()<<"-"<<(hardness[i]-1)<<")"<<std::endl;
				}
				data.resize(0);
				dsd.InitializeSearch(me, start, goal, solution);
				searchRunning = true;
				break;
			}
		case 'r': 
			{
				data.resize(0); 
				dsd.policy = (tExpansionPriority)((dsd.policy)%kDSDPolicyCount);
				printf("Problem: %d\n", problemNumber);
				printf("Policy: %d\n", dsd.policy);
				printf("Bound: %f\n", bound);
				dsd.InitializeSearch(me, start, goal, solution);
				searchRunning = true;
				break;
			}
		case 'w': 
			{
				data.resize(0);
				if(bound>=9) bound=1.25;
				// else bound = (bound-1)*1.1+1;
				else bound = (bound-2)*2+2;


				MyWindowHandler(windowID, kWindowDestroyed);
				MyWindowHandler(windowID, kWindowCreated);
				
				//Set the input weight to the new bound
				me->SetInputWeight(bound);

				//Set the cost of each terrain type randomly.
				for(int i=0; i<4; i++){
					//[0]=kSwamp, [1]=kWater ,[2]=kGrass, [3]=kTrees

					// 1. Random Hardness
					// rdm = random()%101;
					// hardness[i] = rdm/101+1;
					//Or 2. Hardcoded Hardness
					hardness[0]=1.5; hardness[1]=1.45; hardness[2]=1.25; hardness[3]=1.95;

					Tcosts[i] = hardness[i]*(me->GetInputWeight())-(hardness[i]-1);
					string type;
					if(i==0) type="Swamp";
					if(i==1) type="Water";
					if(i==2) type="Grass";
					if(i==3) type="Trees";
					std::cout<<"Cost "<<type<<"="<<Tcosts[i]<<" ("<<hardness[i]<<"*"<<me->GetInputWeight()<<"-"<<(hardness[i]-1)<<")"<<std::endl;
				}
				me->SetTerrainCost(Tcosts);

				problemNumber = 0;
				dsd.policy = (tExpansionPriority)((dsd.policy)%kDSDPolicyCount);
				printf("Problem: %d\n", problemNumber);
				printf("Policy: %d\n", dsd.policy);
				printf("Bound: %f\n", bound);
				dsd.InitializeSearch(me, start, goal, solution);
				searchRunning = true;
				break;
			}
		case 'p': showPlane = !showPlane; break;
		case '[': stepsPerFrame = std::max(stepsPerFrame/2, 1); break;
		case ']': stepsPerFrame = stepsPerFrame*2; break;
		case '}':
			{
				data.resize(0);
				dsd.policy = (tExpansionPriority)((dsd.policy+1)%kDSDPolicyCount);
				printf("Problem: %d\n", problemNumber);
				printf("Policy: %d\n", dsd.policy);
				printf("Bound: %f\n", bound);
				dsd.InitializeSearch(me, start, goal, solution);
				searchRunning = true;
				break;
			}
		case '{':
			{
				data.resize(0);
				dsd.policy = (tExpansionPriority)((dsd.policy-1)%kDSDPolicyCount);
				printf("Problem: %d\n", problemNumber);
				printf("Policy: %d\n", dsd.policy);
				printf("Bound: %f\n", bound);
				dsd.InitializeSearch(me, start, goal, solution);
				searchRunning = true;
				break;
			}
		case '.':
		{
			do {
				start.x = random()%me->GetMap()->GetMapWidth();
				start.y = random()%me->GetMap()->GetMapHeight();
			} while (me->GetMap()->GetTerrainType(start.x, start.y) != kGround);
			do {
				goal.x = random()%me->GetMap()->GetMapWidth();
				goal.y = random()%me->GetMap()->GetMapHeight();
			} while (me->GetMap()->GetTerrainType(goal.x, goal.y) != kGround);

			problemNumber +=1;
			printf("==============\n");
			printf("Problem: %d\n", problemNumber);
			printf("Policy: %d\n", dsd.policy);
			printf("Bound: %f\n", bound);
			data.resize(0);
			dsd.InitializeSearch(me, start, goal, solution);
			searchRunning = true;
			break;
		}
    	default:
			break;
	
	}
}

void SetNextPriority(float h, float g, float target) // const Graphics::point &loc
{
	float slope = g/h;
	if (h > 0 && g > 0)
	{
		if (data.size() == 0 || data.back().slope < slope)
		{
			//std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
			float minWeight, maxWeight;
			if (data.size() > 0)
				GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, slope);
			else
				GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);
						
			float K;
			// K (g + [w] * h) = 1 at previous point
			point3d last;
			if (data.size() == 0)
			{
				last = point3d(1, 0);
			}
			else {
				last = data.back().crossPoint;
			}
			// returns nextWeight and K
			float nextWeight = ChooseWeightForTargetPriority({h, g}, target, minWeight, maxWeight, last, K);
			
			// our cross point of next slope
			point3d crossPoint1;
			crossPoint1.x = 1.0f/(K*(slope+nextWeight));
			crossPoint1.y = crossPoint1.x*slope;
			
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

bool MyClickHandler(unsigned long windowID, int viewport, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
//	return false;
//	static point3d startLoc;
//	loc.x = (loc.x+1)/2;
//	loc.y = 1-(loc.y+1)/2;
//	loc *= 2;
	loc = HOGToLocal(loc);
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: //printf("Right button\n"); break;
			{
				std::cout << "Priority of " << loc << " is " << GetPriority(loc.x, loc.y) << "\n";
			}
				break;
			case kLeftButton: //printf("Left button\n");
				
				if (viewport == 3) // TODO: add interactive mode back in later
					SetNextPriority(loc.x, loc.y, testScale); // h and g
				break;
			case kMiddleButton: printf("Middle button\n"); break;
			case kNoButton: break;
		}
	}
	if ((button == kMiddleButton) && (mType == kMouseDown))
	{}
	if (button == kRightButton)
	{}
	return false;
}

float GetPriority(float h, float g)
{
	if (data.size() == 0)
		return INFINITY;
	float slope = g/h;
	if (fgreater(slope, data.back().slope, tolerance))
		return INFINITY;
	// range includes low but not high
//	int low = 0, high = data.size();
	// dumb/slow but correct
	for (int x = 0; x < data.size(); x++)
		if (flesseq(slope, data[x].slope, tolerance))
			return data[x].K*(g + data[x].weight * h);
//	while (true)
//	{
//		// f = K * ( g + w_i * h )
//		if (low >= high-1)
//		{
//			return data[low].K*(g + data[low].weight * h);
//		}
//		int mid = (low+high)/2;
//		if (data[mid].slope > slope)
//		{
//			high = mid+1;
//		}
//		else {
//			low = mid+1;
//		}
//	}
	return INFINITY;
//	return data[low].K*(g + data[low].weight * h);
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
	maxWeight = 2*bound-1;

	// 0. next slope line is y = slope*x
	// 1. cannot go over the [y = -x + w] line
	// slope*x = -x + w; slope*x + x = w; x = w/(slope+1)
	// y/slope = w-y; y = slope * w - slope *y; y*(1+slope) = slope*w; y = slope*w/(1+slope)
	point3d upperPoint(bound/(nextSlope+1),
					   nextSlope*bound/(1+nextSlope));
//	std::cout << "Upper point: " << upperPoint << "\n";
	// 2. cannot go under the [y = -(2w-1)x + w] line
	// slope*x = -(2w-1)x + w; x*(slope + (2w-1)) = w
	// y/slope = (w-y)/(2w-1)
	// y (2w-1) = slope * w - slope*y
	// y (slope + 2w-1) = slope*w
	point3d lowerPoint(bound/(nextSlope+2*bound-1),
					   nextSlope*bound / (nextSlope + 2*bound-1));
	// get (negative) slopes to upper and lower points
	minWeight = std::max(minWeight, (lowerPoint.y-currPoint.y)/(currPoint.x-lowerPoint.x));
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
		printf("Ill defined case (new y < old y); defaulting to min\n");
		K = 1/(last.y+minWeight*last.x);
		return minWeight;
	}
	if (fgreatereq(loc.x, projectedPoint.x, tolerance))
	{
		printf("Ill defined case (new x > old x); defaulting to max\n");
		K = 1/(last.y+maxWeight*last.x);
		return maxWeight;
	}
	
	// Then try to extend that point to the new point giving it the desired priority
	weight = (loc.y-projectedPoint.y)/(projectedPoint.x-loc.x);
	// and bound by min/max weight
	weight = std::max(std::min(weight, maxWeight), minWeight);
	K = 1/(last.y+weight*last.x);
	return weight;

/*
	// Old approach. (Keeping to verify should be same for priority=1)
    // Only difference is in checking for ill defined cases
	// Requires that loc be above and to the left of last
	if (loc.x > last.x)
	{
		K = 1/(last.y+maxWeight*last.x);
		return maxWeight;
	}
	if (loc.y < last.y)
	{
		K = 1/(last.y+minWeight*last.x);
		return minWeight;
	}
	// Old: Find the crossing point of two equations of the form:
	// f = K * (y +  w*x);
	// K = 1/(last.y+nextWeight*last.x);
	// priority = K*(loc.y + weight * loc.x);
	// priority = (loc.y + weight * loc.x)/(last.y+weight*last.x);
	// priority*last.y+priority*weight*last.x = loc.y + weight * loc.x;
	// priority*weight*last.x-weight * loc.x = loc.y - priority*last.y
	// weight*(priority*last.x-loc.x) = loc.y - priority*last.y
	weight = (loc.y - priority*last.y)/(priority*last.x-loc.x);
	weight = std::max(std::min(weight, maxWeight), minWeight);
	K = 1/(last.y+weight*last.x);
	return weight;
*/
 }
