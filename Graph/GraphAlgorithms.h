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
#include <list>

#define STARTING_DISTANCE 0

template<typename T>
struct PathDescriptor{
    Node<T>* node;
    int distanceFromStartingNode;
    Node<T>* previousNode;
    PathDescriptor(Node<T>* node, int distanceFromStartingNode = INFINITY, Node<T>* previousNode = nullptr) : node(node), distanceFromStartingNode(distanceFromStartingNode), previousNode(previousNode) {} ;
    bool operator==(const PathDescriptor<T>& pathDescriptor)
    {
        return node == pathDescriptor.node && distanceFromStartingNode == pathDescriptor.distanceFromStartingNode && previousNode == pathDescriptor.previousNode;
    }
};

template<typename T>
class GraphAlgorithms : public Graph<T>{
public:
    GraphAlgorithms(T rootValue, bool directioned = false, bool weighted = false) : Graph<T>(rootValue, directioned, weighted) {};
    void assignValueToNodes(vector<T> varticeValues);
    static GraphAlgorithms<T> constructGraph(vector<int> verticeDegrees, vector<T> varticeValues);
    GraphAlgorithms(int** matrix, vector<T> varticeValues);
    
    // MARK: - Dijkstra
    
    PathDescriptor<T>& getPathDescriptor(Node<T>* node, vector<PathDescriptor<T>>& pathDescriptorsTable);
    int shortestPathDescriptorIndex(vector<PathDescriptor<T>>& pathDescriptorsTable, vector<Node<T>*>& unvisitedNodes);
    void deleteUnvisitedNode(Node<T> *shortestPathNode, vector<Node<T> *> &unvisitedNodes);
    void dijkstraTableAssignment(vector<Node<T>*>& visitedNodes, vector<Node<T>*>& unvisitedNodes, vector<PathDescriptor<T>>& pathDescriptorsTable);
    void findShortestPathBetween(Node<T>* startingNode, Node<T>* toNode);
    
    //MARK: Minimum Spanning Tree
    //Prim's algorithm
    vector<Edge<T>*> composeMinimumSpanningTree();
     //MARK: Kosaraju's algorithm
    int stronglyConnectedVertices();
    //MARK: Halmiltonian path problem

};


template<typename T>
void GraphAlgorithms<T>::assignValueToNodes(vector<T> varticeValues)
{
    if(varticeValues.size() != this->nodes.size())
        throw runtime_error("Invalid number of node values!");
    
    for(int i{0}; i < this->nodes.size(); i++)
    {
        this->nodes[i]->value = varticeValues[i];
    }
    this->root = this->nodes[0];
}

template<typename T>
GraphAlgorithms<T> GraphAlgorithms<T>::constructGraph(vector<int> verticeDegrees, vector<T> varticeValues)
{
    int nodesCount = static_cast<int>(verticeDegrees.size());
    int **adjacencyMatrix = new int*[nodesCount];
    for(int row{0}; row < nodesCount; row++)
    {
        for(int col{0}; col < nodesCount; col++)
        {
            adjacencyMatrix[row] = new int[col];
        }
    }
    
    for(int row{0}; row < nodesCount - 1; row++) {
        for(int col{row + 1}; col < nodesCount; col++) {
            
            if(verticeDegrees[row] > 0 && verticeDegrees[col] > 0) {
                adjacencyMatrix[row][col] = 1;
                
                adjacencyMatrix[col][row] = 1;
                verticeDegrees[col] -= 1;
                verticeDegrees[row] -= 1;
            }
            
        }
    }
    return GraphAlgorithms<T>(adjacencyMatrix, varticeValues);
}


template<typename T>
GraphAlgorithms<T>::GraphAlgorithms(int** matrix, vector<T> verticeValues) : Graph<T>()
{
    int nodesCount = static_cast<int>(verticeValues.size());
    for(int i{0}; i < nodesCount; i++)
    {
        Node<T>* newNode = new Node<T>;
        this->nodes.push_back(newNode);
    }
    
    for(int row{0}; row < nodesCount; row++)
    {
        for(int col{0}; col < nodesCount; col++)
        {
            if(row == col)
                continue;
            
            if(matrix[row][col] == 1)
            {
                Node<T>* startingNode = this->nodes[row];
                Node<T>* toNode = this->nodes[col];
                if(!this->edges.empty())
                {
                    bool areAlreadyConnected = false;
                    
                    for(Edge<T>* e: this->edges)
                    {
                        if((e->fromNode == startingNode || e->fromNode == toNode) && (e->toNode == startingNode || e->toNode == toNode))
                        {
                            areAlreadyConnected = true;
                            break;
                        }
                    }
                    if(!areAlreadyConnected)
                        this->connectNodes(this->nodes[row], this->nodes[col]);
                }
                else {
                    this->connectNodes(startingNode, toNode);
                }
            }
        }
    }
    
    assignValueToNodes(verticeValues);
    for(int row{0}; row < nodesCount; row++)
    {
        delete[] matrix[row];
    }
    delete[] matrix;
}


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

template<typename T>
inline void deleteNode(Node<T> *outboundNode, vector<Node<T> *>& unvisitedNodes) {
    for(int i{0}; i < unvisitedNodes.size(); i++)
    {
        if(unvisitedNodes[i] == outboundNode)
        {
            unvisitedNodes.erase(unvisitedNodes.begin() + i);
            break;
        }
    }
}

//MARK: Minimum Spanning Tree
template<typename T>
vector<Edge<T>*> GraphAlgorithms<T>::composeMinimumSpanningTree()
{
    vector<Node<T>*> visitedNodes;
    vector<Node<T>*> unvisitedNodes = this->nodes;
    vector<Edge<T>*> mstEdges;
    unordered_map<Node<T>*, int> nodesKeyValues;
    for(Node<T>* node : unvisitedNodes)
    {
        nodesKeyValues.emplace(node, INFINITY);
    }
    Node<T>* currentNode = this->nodes[rand() % this->nodes.size()];
    nodesKeyValues[currentNode] = 0;
    visitedNodes.push_back(currentNode);
    deleteNode(currentNode, unvisitedNodes);
    while(!unvisitedNodes.empty())
    {
        Edge<T>* minimumWeightEdge = nullptr;
        for(Edge<T>* edge : currentNode->getAllEdges())
        {
            if(minimumWeightEdge == nullptr)
                minimumWeightEdge = edge;
            
            else
                minimumWeightEdge = edge->weight < minimumWeightEdge->weight ? edge : minimumWeightEdge;
        }
        Node<T>* outboundNode =  currentNode == minimumWeightEdge->toNode ? minimumWeightEdge->fromNode : minimumWeightEdge->toNode;
        if(find(visitedNodes.begin(), visitedNodes.end(), outboundNode) ==  visitedNodes.end())
        {
            nodesKeyValues[outboundNode] = nodesKeyValues[outboundNode] > minimumWeightEdge->weight ?  minimumWeightEdge->weight :  nodesKeyValues[outboundNode];
            visitedNodes.push_back(outboundNode);
            mstEdges.push_back(minimumWeightEdge);
            
            deleteNode(outboundNode, unvisitedNodes);
        }
        Node<T>* lowestCostNode = nullptr;
        for(Node<T>* node: unvisitedNodes)
        {
            if(lowestCostNode == nullptr)
                lowestCostNode = node;
                       
            else
                lowestCostNode = nodesKeyValues[node] < nodesKeyValues[lowestCostNode] ? node : lowestCostNode;
        }
        currentNode = lowestCostNode;
    }
    return mstEdges;
}

//MARK: Kosaraju's algorithm

template<typename T>
void visitNode(Node<T>* node, vector<Node<T>*>& visitedNodes, list<Node<T>*>& connectedVertices)
{
    if(find(visitedNodes.begin(), visitedNodes.end(), node) == visitedNodes.end())
    {
        visitedNodes.push_back(node);
        for(Edge<T>* outboundEdge : node->outboundEdges)
        {
            visitNode(outboundEdge->toNode, visitedNodes, connectedVertices);
        }
        connectedVertices.push_front(node);
    }
}

template<typename T>
int findInAssignedComponents(Node<T>* node, vector<vector<Node<T>*>> assignedComponents)
{
    for(int i{0}; i < assignedComponents.size(); i++)
    {
        vector<Node<T>*>& component = assignedComponents[i];
        if(find(component.begin(), component.end(), node) != component.end())
            return i;
    }
    return -1;
}

template<typename T>
void assign(Node<T>* node, Node<T>* root, vector<vector<Node<T>*>>& assignedComponents, vector<Node<T>*>& visitedNodes)
{
    //Graph cycle detection
    if(find(visitedNodes.begin(), visitedNodes.end(), node) != visitedNodes.end())
    {
        return;
    }
    
    if(node == root)
    {
        if(findInAssignedComponents(node, assignedComponents) == -1) {
            vector<Node<T>*> connectedComponent;
            connectedComponent.push_back(node);
            assignedComponents.push_back(connectedComponent);
        }
        else
        {
            return;
        }
    }
        
    visitedNodes.push_back(node);
    for(Edge<T>* inboundEdge : node->inboundEdges)
    {
        if(find(visitedNodes.begin(), visitedNodes.end(), inboundEdge->fromNode) == visitedNodes.end())
        {
            int index = findInAssignedComponents(root, assignedComponents);
            assign(inboundEdge->fromNode, root, assignedComponents, visitedNodes);
            assignedComponents[index].push_back(node);
        }
    }
};

template<typename T>
bool subcomponentsIncluded(vector<list<Node<T>*>>& aggregatedSubComponents, list<Node<T>*> connectedVertices)
{
    for(list<Node<T>*>& list : aggregatedSubComponents)
    {
        for(Node<T>* element : list)
        {
            if(find(connectedVertices.begin(), connectedVertices.end(), element) != connectedVertices.end())
            {
                if(list.size() < connectedVertices.size())
                    list = connectedVertices;
                return true;
            }
        }
    }
    return false;
};

template<typename T>
 int GraphAlgorithms<T>::stronglyConnectedVertices()
{
    vector<Node<T>*> visitedNodes;
    vector<Node<T>*> unvisitedNodes = this->nodes;
    vector<vector<Node<T>*>> assignedComponents;
    vector<list<Node<T>*>> aggregatedSubComponents;
    for(Node<T>* node : unvisitedNodes)
    {
      list<Node<T>*> connectedVertices;
      visitNode(node, visitedNodes, connectedVertices);
      if(!subcomponentsIncluded(aggregatedSubComponents, connectedVertices))
        aggregatedSubComponents.push_back(connectedVertices);
    }
    visitedNodes = vector<Node<T>*>();
    for(list<Node<T>*> list : aggregatedSubComponents)
    {
        for(Node<T>* element : list)
        {
            assign(element, element, assignedComponents, visitedNodes);
        }
    }
    return assignedComponents.size();
}

#endif /* GraphAlgorithms_h */
