#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "stdint.h"
#include "Astar.h"
#include <typeinfo>
using namespace std;

namespace opendlv {
namespace logic {
namespace miniature {


vector<vector<int16_t> > Astar::createMap(vector<vector<double> > outerWalls, vector<vector<double> > innerWalls) {
	vector<vector<int16_t> > map;
	int16_t wallValue = 1;
	int16_t thickWallValue = 2;
	int16_t noWallValue = 0;

	for (int16_t i=0; i<mapSizeX; i++) {
		vector<int16_t> row;
		for (int16_t j=0; j<mapSizeY; j++) {
			row.push_back(noWallValue);
		}
		map.push_back(row);
	}
	double minX = 100;
	double maxX = -100;
	double minY = 100;
	double maxY = -100;
	for (uint16_t i=0; i<outerWalls.size(); i++) {
		if(outerWalls[i][0] > maxX) {
			maxX = outerWalls[i][0];
		} 
		if (outerWalls[i][0] < minX) {
			minX = outerWalls[i][0];
		}
		if (outerWalls[i][1] > maxY) {
			maxY = outerWalls[i][1];
		} 
		if (outerWalls[i][1] < minY) {
			minY = outerWalls[i][1];
		}
	}

	double binX = (maxX-minX)/(double)mapSizeX;
	double binY = (maxY-minY)/(double)mapSizeY;

	int16_t xIndex;
	int16_t yIndex;
	for (uint16_t n=0; n<outerWalls.size(); n++) {
		double xVal = minX;
		double yVal = maxY;
		bool foundX = false;
		bool foundY = false;
		for (int16_t i=0; i<mapSizeX; i++) {
			xVal += binX;
			if (xVal > outerWalls[n][0] && !foundX) {
				xIndex = i;
				foundX = true;
			}
		}
		for (int16_t j=0; j<mapSizeY; j++) {
			yVal -= binY;
			if (yVal < outerWalls[n][1] && !foundY) {
				yIndex = j;
				foundY = true;
			}
		}
		map[xIndex][yIndex] = wallValue;
	}
	// Set border - UGLY
	for (int16_t i=0; i<mapSizeX; i++) {
		map[i][0] = 1;
		map[i][mapSizeY-1] = wallValue;
	}
	for (int16_t j=0; j<mapSizeY; j++) {
		map[0][j] = 1;
		map[mapSizeX-1][j] = wallValue;
	}
	

	uint16_t xIndex1;
	uint16_t yIndex1;

	uint16_t nSteps = 20;
	for (uint16_t n=0; n<innerWalls.size(); n+=2) {
		double diffX = (innerWalls[n][0]-innerWalls[n+1][0])/nSteps;
		double diffY = (innerWalls[n][1]-innerWalls[n+1][1])/nSteps;
		double x = innerWalls[n][0];
		double y = innerWalls[n][1];
		
		for (uint16_t k=0;k<nSteps;k++) {
			
			double xVal = minX;
			double yVal = maxY;
			bool foundX = false;
			for (int16_t i=0; i<mapSizeX; i++) {
				xVal += binX;
				if (xVal > x && !foundX) {
					xIndex1 = i;
					foundX = true;
				}
			}
			bool foundY = false;
			for (int16_t j=0; j<mapSizeY; j++) {
				yVal -= binY;
				if (yVal < y && !foundY) {
					yIndex1 = j;
					foundY = true;
				}
			}
			map[xIndex1][yIndex1] = wallValue;
			x = x-diffX;
			y = y-diffY;
		}
	}

	// Fill all walls with outer layer
	for (int16_t x=0; x<mapSizeX; x++) {
		for (int16_t y=0; y<mapSizeY; y++) {
			if (map[x][y] == wallValue) {
				for (uint16_t k=0; k<8; k++) {
					int16_t nX;
					int16_t nY;
					if (k == 0) {
						nX = x-1;
						nY = y;
					} else if (k == 1) {
						nX = x;
						nY = y-1;
					} else if (k == 2) {
						nX = x+1;
						nY = y;
					} else if (k == 3) {
						nX = x;
						nY = y+1;
					} else if (k == 4) {
						nX = x-1;
						nY = y-1;
					} else if (k == 5) {
						nX = x-1;
						nY = y+1;
					} else if (k == 6) {
						nX = x+1;
						nY = y-1;
					} else if (k == 7) {
						nX = x+1;
						nY = y+1;
					}
					if (nX >= 0 && nY >= 0 && nX < mapSizeX && nY < mapSizeY) {
						if (map[nX][nY] != wallValue) {
							map[nX][nY] = thickWallValue;
						}
					}
				}
			}
		}
	}
	for (int16_t i=0; i<mapSizeX; i++) {
		for (int16_t j=0; j<mapSizeY; j++) {
			if (map[i][j] == thickWallValue) {
				map[i][j] = wallValue;
			}
		}
	}
	return map;
}

void Astar::printMap(vector<vector<int16_t> > map, vector<vector<int16_t> > bestPath) {
	int16_t noWallValue = 0;	
	for (int16_t j=0; j<mapSizeY; j++) {
		for (int16_t i=0; i<mapSizeX; i++) {
			bool pathOutput = false;
			
			for (uint16_t k=0; k<bestPath.size(); k++) {
				if (i == bestPath[k][0] && j == bestPath[k][1]) {
					cout << "o";
					pathOutput = true;
				}
			}
			if (!pathOutput) {
				if (map[i][j] == noWallValue) {
					cout << " ";
				} else {
					cout << "#";
				}
			}
			
			cout << " ";
		}
	cout << endl;
	}
}


vector<vector<int16_t> > Astar::getPath(vector<vector<int16_t> > map) {
	vector<Node> queue;
	vector<Node> checkedNodes;
	queue.push_back(startNode);
	checkedNodes.push_back(startNode);

	int16_t*** pathTrace = new int16_t**[mapSizeX];
	for (int16_t i=0; i<mapSizeX; i++) {
		pathTrace[i] = new int16_t*[mapSizeY];
		for (int16_t j=0; j<mapSizeY; j++) {
			pathTrace[i][j] = new int16_t[2];
		}
	}
	Node** nodes = new Node*[mapSizeX];
	for (int16_t i=0; i<mapSizeX; i++) {
		nodes[i] = new Node[mapSizeY];
	}
	for(int16_t i = 0; i<mapSizeX; i++) {
		for(int16_t j = 0; j<mapSizeY; j++) {
			nodes[i][j].distanceToTargetCost = 10000; // Large number
			nodes[i][j].edgeCost = 10000; // Large number
		}
	}
	nodes[startNode.x][startNode.y].distanceToTargetCost = 0;
	nodes[startNode.x][startNode.y].edgeCost = 0;
	bool foundTarget = false;
	while (!foundTarget && queue.size() != 0) {
		Node currentNode = queue[0];
		if (currentNode.x == targetNode.x && currentNode.y == targetNode.y) {
			foundTarget = true;
			cout << "Target found!!" << endl;
		} else {
			// Update neighbours
			int16_t x = currentNode.x;
			int16_t y = currentNode.y;
			uint16_t checkedNeighbours = 0;
			while (checkedNeighbours < 8) {
				int16_t nX;
				int16_t nY;
				if (checkedNeighbours == 0) {
					nX = x-1;
					nY = y;
				} else if (checkedNeighbours == 1) {
					nX = x;
					nY = y-1;
				} else if (checkedNeighbours == 2) {
					nX = x+1;
					nY = y;
				} else if (checkedNeighbours == 3) {
					nX = x;
					nY = y+1;
				} else if (checkedNeighbours == 4) {
					nX = x-1;
					nY = y-1;
				} else if (checkedNeighbours == 5) {
					nX = x-1;
					nY = y+1;
				} else if (checkedNeighbours == 6) {
					nX = x+1;
					nY = y-1;
				} else if (checkedNeighbours == 7) {
					nX = x+1;
					nY = y+1;
				}
				if (nX > 0 && nY > 0 && nX < mapSizeX && nY < mapSizeY) { 
					if (map[nX][nY] == 0 || (nX == targetNode.x && nY == targetNode.y)) {
						int16_t diffX = x-nX;
						int16_t diffY = y-nY;
						int16_t diagDiff = 2;
						if (abs(diffX)+abs(diffY) < diagDiff) {
							if (nodes[nX][nY].edgeCost > nodes[x][y].edgeCost + 10) {
								nodes[nX][nY].edgeCost = nodes[x][y].edgeCost + 10;
								pathTrace[nX][nY][0] = (int16_t)diffX;
								pathTrace[nX][nY][1] = (int16_t)diffY;
							} 
						} else {
							if (nodes[nX][nY].edgeCost > nodes[x][y].edgeCost + 14) {
								nodes[nX][nY].edgeCost = nodes[x][y].edgeCost + 14;
								pathTrace[nX][nY][0] = (int16_t)diffX;
								pathTrace[nX][nY][1] = (int16_t)diffY;
							}
						}
						double diffXTarget = abs((double)(nX-targetNode.x));
						double diffYTarget = abs((double)(nY-targetNode.y));
						double minDiff = min(diffXTarget,diffYTarget);
						double maxDiff = max(diffXTarget,diffYTarget);
						nodes[nX][nY].distanceToTargetCost = 14*minDiff+10*(maxDiff-minDiff);
						nodes[nX][nY].x = nX;
						nodes[nX][nY].y = nY;
						
						// Checked nodes
						bool isChecked = false;
						for (uint16_t i=0; i<checkedNodes.size(); i++) {
							if (nodes[nX][nY].x == checkedNodes[i].x && nodes[nX][nY].y == checkedNodes[i].y) {
								isChecked = true;
							}
						}
						for (uint16_t i=0; i<queue.size(); i++) {
							if (nodes[nX][nY].x == queue[i].x && nodes[nX][nY].y == queue[i].y) {
								isChecked = true;
							}
						}
						if (!isChecked) {
							queue.push_back(nodes[nX][nY]);
						}
					}
				}
				checkedNeighbours++;
			}
			checkedNodes.push_back(currentNode);
			queue.erase(queue.begin());
			sort(queue.begin(), queue.end());
		}
	}
	vector<vector<int16_t> > optimalPath;
	vector<vector<double> > optimalPathCoord;
	if (foundTarget) {
		bool fullPath = false;
		Node currentNode = targetNode;
		while (!fullPath) {
			int16_t cX = currentNode.x;
			int16_t cY = currentNode.y;
			currentNode = nodes[cX+pathTrace[cX][cY][0]][cY+pathTrace[cX][cY][1]];
			vector<int16_t> nodeIndex;
			nodeIndex.push_back(currentNode.x);
			nodeIndex.push_back(currentNode.y);
			optimalPath.push_back(nodeIndex);
			if (currentNode.x == startNode.x && currentNode.y == startNode.y) {
				fullPath = true;
			}
		}
	} 
        vector<vector<int16_t>> reversedPath;
        for (int16_t i = optimalPath.size()-1; i >= 0; i--) {
          reversedPath.push_back(optimalPath[i]);
        }
	return reversedPath;
}

vector<vector<double> > Astar::indexPathToCoordinate(vector<vector<int16_t> > bestPath, vector<vector<double> > outerWalls) {
	double minX = 100;
	double maxX = -100;
	double minY = 100;
	double maxY = -100;
	for (uint16_t i=0; i<outerWalls.size(); i++) {
		if(outerWalls[i][0] > maxX) {
			maxX = outerWalls[i][0];
		} 
		if (outerWalls[i][0] < minX) {
			minX = outerWalls[i][0];
		}
		if (outerWalls[i][1] > maxY) {
			maxY = outerWalls[i][1];
		} 
		if (outerWalls[i][1] < minY) {
			minY = outerWalls[i][1];
		}
	}
	double xFactor = (maxX-minX)/mapSizeX;
	double yFactor = (maxY-minY)/mapSizeY;
	vector<vector<double> > bestPathCoordinate;
	for (uint16_t i=0; i<bestPath.size(); i++) {
		double xCoord = (double)minX + xFactor*(double)bestPath[i][0];
		double yCoord = (double)maxY - yFactor*(double)bestPath[i][1];
		vector<double> coordinate;
		coordinate.push_back(xCoord);
		coordinate.push_back(yCoord);
		bestPathCoordinate.push_back(coordinate);
	}
	return bestPathCoordinate;
}

vector<int16_t> Astar::coordToIndex(double x, double y, vector<vector<double> > outerWalls) {
	double minX = 100;
	double maxX = -100;
	double minY = 100;
	double maxY = -100;
	for (uint16_t i=0; i<outerWalls.size(); i++) {
		if(outerWalls[i][0] > maxX) {
			maxX = outerWalls[i][0];
		} 
		if (outerWalls[i][0] < minX) {
			minX = outerWalls[i][0];
		}
		if (outerWalls[i][1] > maxY) {
			maxY = outerWalls[i][1];
		} 
		if (outerWalls[i][1] < minY) {
			minY = outerWalls[i][1];
		}
	}

	double binX = (maxX-minX)/(double)mapSizeX;
	double binY = (maxY-minY)/(double)mapSizeY;

	vector<int16_t> indexes;
	double xVal = minX;
	double yVal = maxY;
	bool foundX = false;
	bool foundY = false;
	for (int16_t i=0; i<mapSizeX; i++) {
		xVal += binX;
		if (xVal > x && !foundX) {
			indexes.push_back(i);
			foundX = true;
		}
	}
	for (int16_t j=0; j<mapSizeY; j++) {
		yVal -= binY;
		if (yVal < y && !foundY) {
			indexes.push_back(j);
			foundY = true;
		}
	}
	return indexes;
	
}


/*int main() {
	Astar aStar;
	aStar.mapSizeX = 60;
	aStar.mapSizeY = 30;
	aStar.startNode.x = 3;
	aStar.startNode.y = 3;
	aStar.targetNode.x = 51;
	aStar.targetNode.y = 20;

	vector<vector<double>> outerWalls = {{50.84, -23.93}, {-9.48, -24.49}, {-9.70, 5.50}, {50.54, 5.65}};

	vector<vector<double>> innerWalls = {{40.02, 5.86},{39.76, -6.63}, {2.88,5.33}, {2.92,0.57}, {-9.71,-4.17}, {-7.45,-4.11}, {33.06,-24.10}, {33.08,-19.09}, {33.08,-19.09}, {35.50,-19.10}, {20.74,-17.83}, {14.24,-7.36}, {14.24,-7.36}, {18.59,-4.93}, {18.59,-4.93}, {26.08,-6.93}, {26.08,-6.93}, {20.74,-17.83}};
	
	vector<vector<uint16_t>> map = aStar.createMap(outerWalls, innerWalls);
	vector<Node> bestPath = aStar.getPath(map);
	
	vector<vector<uint16_t>> bestPathCoord;
	for (uint16_t i=0; i<bestPath.size(); i++) {
		vector<uint16_t> coordinate = {bestPath[i].x, bestPath[i].y};
		bestPathCoord.push_back(coordinate);
	}
	//uint16_t bestPathCoord[bestPath.size()][2];
	//for (uint16_t i = 0; i<bestPath.size(); i++) {
	//	bestPathCoord[i][0] = bestPath[i].x;
	//	bestPathCoord[i][1] = bestPath[i].y;
	//}
	
	aStar.printMap(map, bestPathCoord);
}*/

}
}
}
