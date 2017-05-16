#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "stdint.h"

namespace opendlv {
namespace logic {
namespace miniature {

//using namespace std;

class Node {
	public:
		int16_t x;
		int16_t y;
		uint16_t edgeCost;
		uint16_t distanceToTargetCost;
		void set_position(int16_t xPos, int16_t yPos) {
			x = xPos;
			y = yPos;
		}
		bool operator<(const Node& node) const {
			if ((edgeCost+distanceToTargetCost)<(node.edgeCost+node.distanceToTargetCost)) {
				return true;
			} else if((edgeCost+distanceToTargetCost)==(node.edgeCost+node.distanceToTargetCost)) {
				if (distanceToTargetCost < node.distanceToTargetCost) {
					return true;
				} 
			}
			return false;
		}
};


class Astar {
	public:	
		int16_t mapSizeX;
		int16_t mapSizeY;
		void setMapSize(int16_t sizeX, int16_t sizeY) {
			mapSizeX = sizeX;
			mapSizeY = sizeY;
		}		
		Node startNode;
		Node targetNode;
		std::vector<std::vector<int16_t> > createMap(std::vector<std::vector<double> > outerWalls, std::vector<std::vector<double> > innerWalls);
		void printMap(std::vector<std::vector<int16_t> > map, std::vector<std::vector<int16_t> > bestPath);
		std::vector<std::vector<int16_t> > getPath(std::vector<std::vector<int16_t> > map);
		std::vector<std::vector<double> > indexPathToCoordinate(std::vector<std::vector<int16_t> > bestPath, std::vector<std::vector<double> > outerWalls);
		std::vector<int16_t> coordToIndex(double x, double y, std::vector<std::vector<double> > outerWalls);
};

}
}
}
