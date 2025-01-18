#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include <vector>
#include <queue>
#include <unordered_map>
#include "../PriorityQueue.h"
//#include "../Framework/TileSystem/Tile.h"
#include "../Framework/TileSystem/TileMap.h"
#include <chrono>
#include <iostream>
#include <DirectXColors.h>
using namespace std::chrono;
using namespace std;

namespace ufl_cap4053
{
	namespace searches
	{
		class PathSearch
		{
		// CLASS DECLARATION GOES HERE
			public:
				// methods
				DLLEXPORT PathSearch(); // EX: DLLEXPORT required for public methods - see platform.h
				DLLEXPORT ~PathSearch();
				DLLEXPORT void load(TileMap* _tileMap);
				DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);
				DLLEXPORT void update(long timeslice);
				DLLEXPORT void shutdown();
				DLLEXPORT void unload();
				DLLEXPORT bool isDone() const;
				DLLEXPORT std::vector<Tile const*> const getSolution() const;
				DLLEXPORT inline bool areAdjacent(Tile* lhs, Tile* rhs);
				DLLEXPORT vector<Tile*> getAdjVec(Tile* currTile);
				DLLEXPORT inline double heuristic(Tile* currTile, Tile* goalTile);
				DLLEXPORT inline double given(Tile* currTile);				
				DLLEXPORT vector<Tile*> pathToTile(Tile* tile);
				DLLEXPORT void reset();

				// variables
				static unordered_map<Tile*, double> heurMap;
				static unordered_map<Tile*, double> givenMap;
				static TileMap* tilesMap;
				static unordered_map<Tile*, Tile*> parentMap;
				

			private:
				vector<Tile const*> finalPath;
				bool solFound;
				Tile* goal;
				Tile* start;
				PriorityQueue<Tile*> tilesQueue;
				unordered_map<Tile*, bool> tilesVisited;
				
		};
	}
}  // close namespace ufl_cap4053::searches
