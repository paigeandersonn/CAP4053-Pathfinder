#include "PathSearch.h"


bool greaterThan(ufl_cap4053::Tile* const& lhs, ufl_cap4053::Tile* const& rhs);
namespace ufl_cap4053
{
	namespace searches	{
		// ASSIGNMENT API METHOD DEFINITIONS
		unordered_map<Tile*, double> PathSearch::heurMap;
		unordered_map<Tile*, double> PathSearch::givenMap;
		unordered_map<Tile*, Tile*> PathSearch::parentMap;
		TileMap* PathSearch::tilesMap;

		// The constructor; takes no arguments.
		PathSearch::PathSearch() : start(nullptr), goal(nullptr), solFound(false), tilesQueue(greaterThan) {}

		// The destructor should perform any final cleanup required before deletion of the object.
		PathSearch::~PathSearch() {}

		// Called after the tile map is loaded. This is usually where the search graph is generated. 
		// NOTE: This method must take no longer than twice the benchmark example for any given map.
		void PathSearch::load(TileMap* _tileMap) {
			tilesMap = _tileMap;
		}

		// Called before any update of the path planner; 
		// Should prepare for search to be performed between the tiles at the coordinates indicated.
		// This method is always preceded by at least one call to load().
		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol) {
			// first tilesMap is loaded, then here start/goal states & all maps initialized; start pushed to queue
			solFound = false;
			tilesMap->setStartTile(startRow, startCol);
			tilesMap->setGoalTile(goalRow, goalCol);
			heurMap[tilesMap->getStartTile()] = heuristic(tilesMap->getStartTile(), tilesMap->getGoalTile());
			givenMap[tilesMap->getStartTile()] = 0;
			parentMap[tilesMap->getStartTile()] = nullptr;
			tilesVisited[tilesMap->getStartTile()] = true;
			start = tilesMap->getStartTile();
			goal = tilesMap->getGoalTile();
			tilesQueue.push(tilesMap->getStartTile());
		}

		// Allow path planner to execute for specified timeslice (in millieconds); if 0 then perform one iteration
		// Search performed until time expires or solution found (iterate for number of milliseconds)
		// Always preceded by at least one call to initialize()
		void PathSearch::update(long timeslice) {
			// 1. find start time, check if start and goal are equal; if so then add start to final path & pop queue to escape update
			auto startTime = high_resolution_clock::now();
			if (start == goal) {
				finalPath.push_back(const_cast<const Tile*>(start));
				tilesQueue.pop();
				solFound = true;
				return;
			}

			// 2. while queue is not empty...
			while (!tilesQueue.empty()) {
				// 3. set current tile and current time; pop tile from queue
				Tile* currTile = tilesQueue.front();
				auto currTime = high_resolution_clock::now();
				auto elapsedTime = duration_cast<milliseconds>(currTime - startTime);
				tilesQueue.pop();
				
				// 4. check all break conditions (time elapsed, currTile = goalTile)	
				if (timeslice > 0 && elapsedTime.count() > timeslice) {
					return;
				} 	 
				if (currTile == goal) {
					vector<Tile*> currPath = pathToTile(currTile);
					tilesMap->resetTileDrawing();
					solFound = true;
					for (Tile* tile : currPath) {
						tile->resetDrawing();
						finalPath.push_back(const_cast<const Tile*>(tile));
					}
					break;
				}

				// 5. get adjacent vector
				vector<Tile*> adjVec = getAdjVec(currTile);	

				// 6. for adjTiles, perform cost updates (based on slide)
				for (Tile* tile : adjVec) {
					double tempCost = givenMap[currTile] + tilesMap->getTileRadius();
					if (tilesVisited.find(tile) == tilesVisited.end()) {
						givenMap[tile] = tempCost;
						parentMap[tile] = currTile;
						heurMap[tile] = heuristic(tile, goal);
						tilesVisited[tile] = true;						
						tilesQueue.push(tile);
						tile->setFill(0x7FFF99FF);
					}
					else {
						if (givenMap[tile] > tempCost) {
							givenMap[tile] = tempCost;
							parentMap[tile] = currTile;
							heurMap[tile] = heuristic(tile, goal);
							vector<Tile*> enumVec;
							tilesQueue.enumerate(enumVec);
							if (find(enumVec.begin(), enumVec.end(), tile) == enumVec.end()) {
								tilesQueue.push(tile);
							}
							tile->setFill(0x7FFF99FF);
						}
					}
				}
				currTile->setFill(0x7F9999FF);

				// 8. only perform one iteration if timeslice is 0
				if (timeslice == 0) {
					return;
				}
			}			
		}

		// Called when search data no longer needed
		// Clean up memory allocated for this search (not same as destructor)
		// May be called before initialize() and/or update()
		void PathSearch::shutdown() {
			reset();
		}

		// Called when tile map is unloaded and cleans up any memory allocated for tile map
		// Not same as destructor - search object may be reinitialized with new map; can be called before load()
		void PathSearch::unload() {
			reset();
			tilesMap = nullptr;
		}

		// Returns true if the update function has finished because it found a solution and false if otherwise
		// Once search is completed this method continues to return true until initialize() is called
		bool PathSearch::isDone() const {
			return solFound;
		}

		// Return a vector containing the solution path as an ordered series of Tile pointers from finish to start
		// Once search has been completed this method continues to return the path until initialize() is called
		vector<Tile const*> const PathSearch::getSolution() const {
			return finalPath;
		}

		// based on manual determine if two tiles are adjacent
		bool PathSearch::areAdjacent(Tile* lhs, Tile* rhs) {
			auto lhsRow = lhs->getRow();
			auto rhsRow = rhs->getRow();
			auto lhsCol = lhs->getColumn();
			auto rhsCol = rhs->getColumn();
			
			// check if odd
			if (lhsRow % 2 != 0) {
				if (rhsRow - lhsRow != 0) {
					if (rhsCol - lhsCol == -1) {
						return false;
					}
				}
			}
			// check if even
			else {
				if (rhsRow - lhsRow != 0) {
					if (rhsCol - lhsCol == 1) {
						return false;
					}
				}
			}
			return true;


		}

		// iterate through surrounding tiles to return vector containing all adjacent tiles 
		vector<Tile*> PathSearch::getAdjVec(Tile* currTile) {
			int currRow = currTile->getRow();
			int currCol = currTile->getColumn();
			vector<Tile*> adjVec;

			for (int aRow = -1; aRow <= 1; ++aRow) {
				for (int aCol = -1; aCol <= 1; ++aCol) {
					if (aRow == 0 && aCol == 0) {
						continue;
					}
					int newRow = currRow + aRow;
					int newCol = currCol + aCol;
					Tile* currAdj = tilesMap->getTile(newRow, newCol);
					if (currAdj != nullptr && currAdj->getWeight() != 0 && areAdjacent(currTile, currAdj)) {
						adjVec.push_back(currAdj);
					}
				}
			} 
			return adjVec;
		}

		// for heuristic cost, use x and y coordinated to obtain straight line distance from tile to goal 
		double PathSearch::heuristic(Tile* currTile, Tile* goalTile) {
			double dX = (goalTile->getYCoordinate() - currTile->getYCoordinate());
			double dY = (goalTile->getXCoordinate() - currTile->getXCoordinate());
			double dist = sqrt(dX * dX + dY * dY);
			return dist;
		}

		// for given cost, iterate through path to tile, taking into account tile radius and weight
		double PathSearch::given(Tile* currTile) {
			if (currTile == tilesMap->getStartTile()) {
				return 0;
			}
			vector <Tile*> path = pathToTile(currTile);
			double cost = 0;
			for (Tile* tile : path) {
				cost = cost + (tilesMap->getTileRadius() * tile->getWeight());
			}
			return cost;
		}

		// starting at current tile add to vector then move to parent tile and repeat until start tile is reached
		vector<Tile*> PathSearch::pathToTile(Tile* tile) {
			vector<Tile*> path;
			Tile* tempTile = tile;
			while (tempTile != tilesMap->getStartTile()) {
				path.push_back(tempTile);
				tempTile = parentMap[tempTile];
			}
			path.push_back(tilesMap->getStartTile());
			return path;
		}

		// perform reset actions 
		void PathSearch::reset() {
			if (tilesMap != nullptr) {
				tilesMap->resetTileDrawing();
			}
			solFound = false;
			start = nullptr;
			goal = nullptr;
			finalPath.clear();
			while (!tilesQueue.empty()) {
				tilesQueue.pop();
			}
			tilesVisited.clear();
			parentMap.clear();
			givenMap.clear();
			heurMap.clear();
		}
	}
}  // close namespace ufl_cap4053::searches

// calculate final cost of both lhs and rhs, return bool value of lhs > rhs
bool greaterThan(ufl_cap4053::Tile* const& lhs, ufl_cap4053::Tile* const& rhs) {
	double lhsFinal = ufl_cap4053::searches::PathSearch::givenMap[lhs] + ufl_cap4053::searches::PathSearch::heurMap[lhs];
	double rhsFinal = ufl_cap4053::searches::PathSearch::givenMap[rhs] + ufl_cap4053::searches::PathSearch::heurMap[rhs];
	return lhsFinal > rhsFinal;
}
