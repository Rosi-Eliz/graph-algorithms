//
//  GraphAlgorithms.h
//  Graph
//
//  Created by Rosi-Eliz Dzhurkova on 11.07.20.
//  Copyright © 2020 Rosi-Eliz Dzhurkova. All rights reserved.
//

#ifndef GraphAlgorithms_h
#define GraphAlgorithms_h
#include "Graph.h"
#include <vector>

#define STARTING_DISTANCE 0

template<typename T>
struct PathDescriptor{
    Node<T>* node;
    int distanceFromStartingNode;
    Node<T>* previousNode;
    PathDescriptor(Node<T>* node, int distanceFromStartingNode = INFINITY, Node<T>* previousNode = nullptr) : node(node), distanceFromStartingNode(distanceFromStartingNode), previousNode(previousNode) {} ;
};

template<typename T>
class GraphAlgorithms : public Graph<T>{
public:
    GraphAlgorithms(T rootValue, bool directioned = false, bool weighted = false) : Graph<T>(rootValue, directioned, weighted) {};
    
    // MARK: - Dijkstra
    
    PathDescriptor<T>& getPathDescriptor(Node<T>* node, vector<PathDescriptor<T>>& pathDescriptorsTable);
    int shortestPathDescriptorIndex(vector<PathDescriptor<T>>& pathDescriptorsTable, vector<Node<T>*>& unvisitedNodes);
    void deleteUnvisitedNode(Node<T> *shortestPathNode, vector<Node<T> *> &unvisitedNodes);
    
    void dijkstraTableAssignment(vector<Node<T>*>& visitedNodes, vector<Node<T>*>& unvisitedNodes, vector<PathDescriptor<T>>& pathDescriptorsTable);
    void findShortestPathBetween(Node<T>* startingNode, Node<T>* toNode);
};

// MARK: - Dijkstra

template<typename T>
int GraphAlgorithms<T>::shortestPathDescriptorIndex(vector<PathDescriptor<T>>& pathDescriptorsTable, vector<Node<T>*>& unvisitedNodes)
{
    vector<PathDescriptor<T>*> result;
    
    for(PathDescriptor<T>& descriptor : pathDescriptorsTable)
    {
        result.push_back(&descriptor);
    }
    for(int i{0}; i < result.size() - 1; i++)
    {
        for(int j{i+1}; j < result.size(); j++)
       
        {
             if(result[i]->distanceFromStartingNode > result[j]->distanceFromStartingNode)
                 swap(result[i], result[j]);
        }
    }
    PathDescriptor<T>* unvisitedNodePathDescriptor = nullptr;
    for(PathDescriptor<T>* descriptor : result)
       {
           if(find(unvisitedNodes.begin(), unvisitedNodes.end(), descriptor->node) != unvisitedNodes.end())
           {
               unvisitedNodePathDescriptor = descriptor;
               break;
           }
       }
    if(unvisitedNodePathDescriptor == nullptr)
        throw runtime_error("Incorrect data source");
    
    for(int i{0}; i < pathDescriptorsTable.size(); i++)
    {
        if(&pathDescriptorsTable[i] == unvisitedNodePathDescriptor)
            return i;
    }
    
     throw runtime_error("Incorrect data source");
}

template<typename T>
PathDescriptor<T>& GraphAlgorithms<T>::getPathDescriptor(Node<T>* node, vector<PathDescriptor<T>>& pathDescriptorsTable)
{
    for(PathDescriptor<T>& p : pathDescriptorsTable)
    {
        if(p.node == node)
            return p;
    }
    throw runtime_error("Path descriptor not found!");
}

template <typename T>
void GraphAlgorithms<T>::deleteUnvisitedNode(Node<T> *shortestPathNode, vector<Node<T> *> &unvisitedNodes) {
    for(int i{0}; i < unvisitedNodes.size(); i++)
    {
        if(unvisitedNodes[i] == shortestPathNode)
        {
            unvisitedNodes.erase(unvisitedNodes.begin() + i);
            return;
        }
    }
}

template<typename T>
void GraphAlgorithms<T>::dijkstraTableAssignment(vector<Node<T>*>& visitedNodes, vector<Node<T>*>& unvisitedNodes, vector<PathDescriptor<T>>& pathDescriptorsTable)
{
    if(pathDescriptorsTable.empty())
        throw runtime_error("Incorrect data source!");
    
    if(unvisitedNodes.empty())
    {
        return;
    }
    int index = shortestPathDescriptorIndex(pathDescriptorsTable, unvisitedNodes);
    PathDescriptor<T>& descriptor =  pathDescriptorsTable[index];
    Node<T>* shortestPathNode = descriptor.node;
    
    for(Edge<T>* e : shortestPathNode->getAllEdges())
    {
        Node<T>* targetNode = descriptor.node == e->toNode ? e->fromNode : e->toNode;
        if(find(visitedNodes.begin(), visitedNodes.end(), targetNode) != visitedNodes.end())
            continue;
        
        int distance = descriptor.distanceFromStartingNode + e->weight;
        PathDescriptor<T>& targetNodePathDescriptor = getPathDescriptor(targetNode, pathDescriptorsTable);
        if(distance < targetNodePathDescriptor.distanceFromStartingNode)
        {
            targetNodePathDescriptor.distanceFromStartingNode = distance;
            targetNodePathDescriptor.previousNode = shortestPathNode;
        }
    }
    
    visitedNodes.push_back(shortestPathNode);
    deleteUnvisitedNode(shortestPathNode, unvisitedNodes);
    dijkstraTableAssignment(visitedNodes, unvisitedNodes, pathDescriptorsTable);
}

template<typename T>
void GraphAlgorithms<T>::findShortestPathBetween(Node<T>* startingNode, Node<T>* toNode)
{
    vector<Node<T>*> unvisitedNodes = this->nodes;
    vector<Node<T>*> visitedNodes;
    vector<PathDescriptor<T>> pathDescriptorsTable;
    for(Node<T>* node : unvisitedNodes)
    {
        PathDescriptor<T> descriptor = node == unvisitedNodes[0] ? PathDescriptor<T>(node, STARTING_DISTANCE) : PathDescriptor<T>(node);
        pathDescriptorsTable.push_back(descriptor);
    }
    dijkstraTableAssignment(visitedNodes, unvisitedNodes, pathDescriptorsTable);
    
    vector<Node<T>*> pathway;
    Node<T>* iterativeNode = toNode;
    int pathDistance { getPathDescriptor(toNode, pathDescriptorsTable).distanceFromStartingNode };
    while(iterativeNode != nullptr)
    {
        PathDescriptor<T>& currentDescriptor = getPathDescriptor(iterativeNode, pathDescriptorsTable);
        if(currentDescriptor.previousNode != nullptr)
        pathway.insert(pathway.begin(), currentDescriptor.previousNode);
        
        iterativeNode = currentDescriptor.previousNode;
    }
     
    for(Node<T>* node : pathway)
    {
        cout<<node->value<<" ";
    }
    cout<<toNode->value;
    
    cout << endl;
    cout << "Distance: " << pathDistance << endl;
}
#endif /* GraphAlgorithms_h */
