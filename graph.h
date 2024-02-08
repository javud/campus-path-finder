// graph.h <Starter Code>
// Javid Uddin
// 
// Basic graph class using adjacency list representation.  
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <set>
#include <unordered_set>
#include <utility>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
  // Adjacency List:
  map<VertexT, vector<pair<VertexT, WeightT>>> adjList; // WeightT represents weight of the edge
  unordered_set<VertexT> Vertices;
 public:
  //
  // constructor:
  //
  // Constructs an empty graph 

  graph() {}

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return static_cast<int>(this->Vertices.size());
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;
    //
    // loop through the adjacency list and count how many
    // edges currently exist:
    //
    for(const auto& pair : adjList) {
        count += pair.second.size();
    }
    return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph and returns true.  If the vertex already exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    if(Vertices.find(v) != Vertices.end()) { // vertex already exists in the graph, don't add it again (sets can't contain duplicates either way)
        return false;
    }
    Vertices.insert(v);
    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    if(Vertices.find(from) == Vertices.end() || Vertices.find(to) == Vertices.end()) { // if either vertex does not exist, then return false (can't add edge)
        return false;
    }
    auto it = adjList.find(from); 
    if(it != adjList.end()) { // FROM vertex already exists in the adjacency list map
        for(auto& neighbor : it->second) {
            if(neighbor.first == to) { // if we find the TO vertex (that the edge connects to), overwrite weight
                neighbor.second = weight;
                return true;
            }
        }
    }
    adjList[from].push_back(make_pair(to, weight)); // FROM vertex does not already exist in the adjacency list map, thus add it along with the TO vertex
    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    auto it = adjList.find(from);
    if(it != adjList.end()) {
        for(const auto& neighbor : it->second) {
            if(neighbor.first == to) { // if we find the second vertex (that the edge connects to), set weight variable & return true
                weight = neighbor.second;
                return true;
            }
        }
    }
    return false;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT>  S;
    auto it = adjList.find(v);
    if(it != adjList.end()) {
        for(const auto& neighbor : it->second) {
            S.insert(neighbor.first);
        }
    }
    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> temp;
    for (auto it = Vertices.begin(); it != Vertices.end(); it++) {
        temp.push_back(*it);
    }
    return temp;  // returns a copy (as vector instead of set)
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
      
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;
    output << endl;
    output << "**Vertices:" << endl;
    int i = 0;
    for (auto it = Vertices.begin(); it != Vertices.end(); it++) {
      output << " " << i << ". " << *it << endl;
      i++;
    }

    output << endl;
    output << "**Edges:" << endl;
    for (const auto& pair : adjList) {
        output << pair.first << ": ";
        for (const auto& neighbor : pair.second) {
            output << neighbor.first << "(" << neighbor.second << ") ";
        }
        output << endl;
    }
    output << "**************************************************" << endl;
  }

};
