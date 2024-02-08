// application.cpp <Starter Code>
// Javid Uddin
//
// Application for Open Street Map implementation with UIC campus map to find shortest distance between two buildings & calculate destination building (equidistant) using Dijkstra's shortest-path algorithm. Utilizes graph.h class that stores graphs as adjacency lists. 
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <stack> /*priority_queue*/
#include <set>
#include <utility> /*pair*/
#include <cstdlib>
#include <cstring>
#include <cassert>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"

using namespace std;
using namespace tinyxml2;

const double INF = numeric_limits<double>::max(); // defines 'infinite' for this program (max possible value for a double)

// Searches buildings-vector for user-inputted building name (partial/full) & abbreviations, returns BuildingInfo struct containing building properties or empty if no building was found with the query
BuildingInfo searchBuilding(const vector<BuildingInfo>& Buildings, string query) {
    for(size_t i=0; i < Buildings.size(); i++) { // Search for abbreviations first
        if(Buildings[i].Abbrev == query) { // if abbreviation of current building matches query exactly, it will be set as the matched building
            return Buildings[i];
        }
    }
    for(size_t i=0; i < Buildings.size(); i++) { // If no abbreviation matched, search for partial matches in full name
        if(Buildings[i].Fullname.find(query) != string::npos) {
            return Buildings[i];
        }
    }
    BuildingInfo empty; // returns an empty build info struct (all values defaulted to blank/0) if building was not found w/ specified query
    return empty;
}

// Searches buildings-vector for closest building to midpoint between two inputted buildings, returns BuildingInfo struct containing building properties. Also ensures that building is not part of 'unreachable' set (determined through utilization of Djikstra's algorithm)
BuildingInfo findClosestBuilding(const vector<BuildingInfo>& Buildings, const Coordinates& midpoint, set<string>& unreachableBuildings) {
    double closestDistance = INF;
    BuildingInfo closestBuilding;
    for(size_t i=0; i < Buildings.size(); i++) { // Search through entire buildings list
        if(unreachableBuildings.find(Buildings[i].Fullname) != unreachableBuildings.end()) { // skip any unreachable buildings
            continue;
        }
        double currDistance = distBetween2Points(midpoint.Lat, midpoint.Lon, Buildings[i].Coords.Lat, Buildings[i].Coords.Lon);
        if(currDistance < closestDistance) { // current building has less distance from midpoint, set to closest building
            closestDistance = currDistance;
            closestBuilding = Buildings[i];
        }
    }
    return closestBuilding; 
}

// Finds the closest node to the specified building by looping through all of the nodes and comparing distances. Returns a Coordinates struct for the closest node
Coordinates findClosestNode(const vector<FootwayInfo>& Footways, map<long long, Coordinates>& Nodes, const BuildingInfo& building) {
    // Coordinates for the building we need to find a node closest to
    double buildingLat = building.Coords.Lat;
    double buildingLon = building.Coords.Lon;
    double closestDistance = INF;
    Coordinates closestNode;
    for(size_t i=0; i < Footways.size(); i++) { // navigate through all nodes of all footways and compare distance from building
      for(size_t j=0; j < Footways[i].Nodes.size(); j++) {  
        Coordinates c1 = Nodes[Footways[i].Nodes[j]];
        double currDistance = distBetween2Points(c1.Lat, c1.Lon, buildingLat, buildingLon); // calculate all distances between nodes and compare them using "find minimum" approach, does not return until visiting all nodes
        if(currDistance < closestDistance) {
            closestDistance = currDistance;
            closestNode = Nodes[Footways[i].Nodes[j]];
        }
      }
    }
    return closestNode; // contains the coordinates & id for the node closest to building
}

class prioritize { // defines prioritizing algorithm
    public:
        bool operator() (const pair<long long, double>& p1, const pair<long long, double>& p2) const {
            return p1.second > p2.second;
        }
};

// Dijkstra's algorithm for finding the shortest paths given our starting vertex. Based on ZyBooks/instructor modified implementation of Dijkstra's algorithm. Returns distance to provided destination vertex as a double type.
double DijkstraShortestPath(
    const Coordinates& startV, // start vertex (closest to starting building)
    const Coordinates& destV, // destination vertex (center building)
    graph<long long, double>& G, // contains the graph made using graph.h
    map<long long, double>& distances, // contains each node and their distance from starting node
    map<long long, long long>& predecessors) { // contains each node and their predecessor node

    priority_queue< // priority queue signature
        pair<long long, double>,
        vector<pair<long long, double>>,
        prioritize> unvisitedQueue;
    
    vector<long long> vertices = G.getVertices(); // gets all vertices in the graph (nodes)
    set<long long> visited;
    for(long long currV : vertices) {
        distances[currV] = INF; // initial distance will be inf.
        predecessors[currV] = 0; // inital predecessor for current vertex will be null (value as 0)
        unvisitedQueue.push(make_pair(currV, INF)); // enqueue the current vertex in priority queue
    }
    distances[startV.ID] = 0; // start vertex has a distance of 0 from itself
    unvisitedQueue.push(make_pair(startV.ID, 0));
    while(!unvisitedQueue.empty()) {
        long long currV = unvisitedQueue.top().first; // visit the vertex that has the minimum distance from starting vertex
      //  double currVWeight = unvisitedQueue.top().second;
        unvisitedQueue.pop();
        if(distances[currV] == INF) { // finished finding shortest paths
            break;
        }
        else if(visited.find(currV) != visited.end()) { // skip to next iteration (vertex already visited)
            continue;
        }
        else { // else visit currV 
            // Adjacent vertices
            set<long long> neighbors = G.neighbors(currV);
            for(long long adjV : neighbors) {
                double edgeWeight;
                G.getWeight(currV, adjV, edgeWeight);
                double altPathDistance = distances[currV] + edgeWeight;
                if(altPathDistance < distances[adjV]) { // if a shorter path from starting vertex to adjacent vertex is found, update the adjacent vertex's    distance and predecessor property
                    distances[adjV] = altPathDistance;
                    predecessors[adjV] = currV;
                    unvisitedQueue.push(make_pair(adjV, altPathDistance));
                }
            }
            visited.insert(currV); // add current vertex to visited set (will not be visited again)
        }   
    }
    return distances[destV.ID]; // returns the shortest distance from starting node to destination node
}

// Creates a backwards path by storing predecessors array in a stack and returns a vector containing the forwards path from the starting node
vector<long long> getPath(map<long long, long long>& predecessors, const Coordinates& destV) {
    stack<long long> pathBackwards;
    vector<long long> pathInOrder;
    long long curr = destV.ID; // we start at the destination node and work backwards
    while(predecessors[curr] != 0) { // while not equal to null (in the case of predecessors, 0)
        pathBackwards.push(curr);
        curr = predecessors[curr];
    }
    pathBackwards.push(curr); // add the last node (starting node)
    while(!pathBackwards.empty()) {
        pathInOrder.push_back(pathBackwards.top());
        pathBackwards.pop();
    }
    return pathInOrder; // return our vector containing the path in order from the starting node->destination node
}

//
// Implement your standard application here
//
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    bool validBuildings = true;
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
    cout << endl;

    //
    // TO DO: lookup buildings, find nearest start and dest nodes, find center
    // run Dijkstra's alg from each start, output distances and paths to destination:
    //
    BuildingInfo building1 = searchBuilding(Buildings, person1Building);
    BuildingInfo building2 = searchBuilding(Buildings, person2Building);
    if(building1.Fullname.length() == 0) { // empty building struct has full name of length 0
        cout << "Person 1's building not found" << endl;
        validBuildings = false;
    }
    else if(building2.Fullname.length() == 0) {
        cout << "Person 2's building not found" << endl;
        validBuildings = false;
    }
    if(validBuildings == true) {
        // Proceed with program if both inputted buildings are valid
        cout << "Person 1's point: " << endl;
        cout << " " << building1.Fullname << endl;
        cout << " (" << building1.Coords.Lat << ", " << building1.Coords.Lon << ")" << endl;
        cout << "Person 2's point: " << endl;
        cout << " " << building2.Fullname << endl;
        cout << " (" << building2.Coords.Lat << ", " << building2.Coords.Lon << ")" << endl;

        // Find midpoint coordinates of both buildings
        Coordinates midpoint = centerBetween2Points(building1.Coords.Lat, building1.Coords.Lon, building2.Coords.Lat, building2.Coords.Lon);

        // Set containing buildings that are not reachable (checked by findClosestBuilding function)
        set<string> unreachableBuildings;
        bool pathNotFound = true; // initial boolean value will be true, until valid paths are found
        while(pathNotFound == true) {
            // Find closest building to that midpoint (will check for unreachable buildings)
            BuildingInfo closestBuilding = findClosestBuilding(Buildings, midpoint, unreachableBuildings);
            cout << "Destination Building: " << endl;
            cout << " " << closestBuilding.Fullname << endl;
            cout << " (" << closestBuilding.Coords.Lat << ", " << closestBuilding.Coords.Lon << ")" << endl << endl;

            // Find nearest nodes to each building
            Coordinates p1Node = findClosestNode(Footways, Nodes, building1);
            Coordinates p2Node = findClosestNode(Footways, Nodes, building2);
            Coordinates destNode = findClosestNode(Footways, Nodes, closestBuilding);
            cout << "Nearest P1 node: " << endl;
            cout << " " << p1Node.ID << endl;
            cout << " (" << p1Node.Lat << ", " << p1Node.Lon << ")" << endl;
            cout << "Nearest P2 node: " << endl;
            cout << " " << p2Node.ID << endl;
            cout << " (" << p2Node.Lat << ", " << p2Node.Lon << ")" << endl;
            cout << "Nearest destination node: " << endl;
            cout << " " << destNode.ID << endl;
            cout << " (" << destNode.Lat << ", " << destNode.Lon << ")" << endl << endl;

            // Calculate person 1's shortest distance to destination building using Djikstra's algorithm
            map<long long, double> distances1; // vertex, weight (distances to each node from P1 Node)
            map<long long, double> distances2; // vertex, weight (distances to each node from P2 Node)
            map<long long, long long> predecessors1; // vertex1, vertex2 (predecessors for each node for P1 Node)
            map<long long, long long> predecessors2; // vertex1, vertex2 (predecessors for each node for P2 Node)
            double shortestDistP1 = DijkstraShortestPath(p1Node, destNode, G, distances1, predecessors1);
            double shortestDistP2 = DijkstraShortestPath(p2Node, destNode, G, distances2, predecessors2);

            // Edge case 1 - no path from building 1 to building 2
            if(distances1[p2Node.ID] >= INF) {
                cout << "Sorry, destination unreachable" << endl;
                pathNotFound = false;
            }
            // Edge case 2 - no path to center building
            else if(distances1[destNode.ID] >= INF || distances2[destNode.ID] >= INF) {
                cout << "At least one person was unable to reach the destination building. Finding next closest building..." << endl;
                unreachableBuildings.insert(closestBuilding.Fullname);
                // pathNotFound boolean is true, will select another closest building
            }
            // Both paths to dest. building are valid
            else {
                // Person 1's path
                cout << "Person 1's distance to dest: " << shortestDistP1 << " miles" << endl;
                cout << "Path: ";
                vector<long long> p1Path = getPath(predecessors1, destNode); // store path in order (function converts stack to vector)
                for(size_t i=0; i < p1Path.size() - 1; i++) {
                    cout << p1Path[i] << "->";
                }
                cout << p1Path[p1Path.size()-1] << endl << endl; // output the last node without the arrow (end)

                // Person 2's path
                cout << "Person 2's distance to dest: " << shortestDistP2 << " miles" << endl;
                cout << "Path: ";
                vector<long long> p2Path = getPath(predecessors2, destNode);
                for(size_t i=0; i < p2Path.size() - 1; i++) {
                    cout << p2Path[i] << "->";
                }
                cout << p2Path[p2Path.size()-1] << endl;
                pathNotFound = false; // path was found, exit looping
            }
        }
        
    }
    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}

int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  //
  // TO DO: build the graph, output stats:
  //
  // Milestone 5: Add each node (key) to the graph as a vertex
  for(const auto& pair : Nodes) {
      G.addVertex(pair.first);
  }
  // Milestone 6: Add edges from each node based on footways
  for(size_t i=0; i < Footways.size(); i++) {
      for(size_t j=0; j < Footways[i].Nodes.size() - 1; j++) { // until second to last element
        Coordinates c1 = Nodes[Footways[i].Nodes[j]];
        Coordinates c2 = Nodes[Footways[i].Nodes[j+1]];
        double distance = distBetween2Points(c1.Lat, c1.Lon, c2.Lat, c2.Lon);
        // Add bidirectional edges (from each node)
        G.addEdge(Footways[i].Nodes[j], Footways[i].Nodes[j+1], distance);
        G.addEdge(Footways[i].Nodes[j+1], Footways[i].Nodes[j], distance);
      }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
