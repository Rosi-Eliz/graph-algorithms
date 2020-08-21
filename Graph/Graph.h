//
//  Graph.h
//  Graph
//
//  Created by Rosi-Eliz Dzhurkova on 30.06.20.
//  Copyright © 2020 Rosi-Eliz Dzhurkova. All rights reserved.
//

#ifndef Graph_h
#define Graph_h
#include <vector>
#include <queue>
#include <list>
#include <unordered_map>
using namespace std;

//enum Direction {Inbound, Outbound, None};
#define DIRECTIONED_EDGE " -> "
#define UNDIRECTIONED_EDGE " <-> "

template<typename T>
struct Node;

template<typename T>
struct Edge{
    Node<T>* fromNode;
    Node<T>* toNode;
    int weight;
    Edge() : fromNode(nullptr), toNode(nullptr), weight(0) {};
    Edge(Node<T>* fromNode, Node<T>* toNode, int weight = 0): fromNode(fromNode), toNode(toNode), weight(weight) {};
};

template<typename T>
struct Node{
    T value;
    vector<Edge<T>*> inboundEdges;
    vector<Edge<T>*> outboundEdges;
    vector<Edge<T>*> getAllEdges() const {
        vector<Edge<T>*> result = inboundEdges;
        result.insert(result.end(), outboundEdges.begin(), outboundEdges.end());
        return result;
    };
    Node() {};
    Node(T& value) : value(value) {};
    bool operator==(const Node& node) const { return this == &node;};
};

template<typename T>
class Graph{
protected:
    bool directioned;
    bool weighted;
    vector<Node<T>*> nodes;
    vector<Edge<T>*> edges;
    void deleteNodeInGraph(Node<T>* node);
    void deleteEdge(Edge<T>* edge);
public:
    Node<T>* root;
    void recursiveGraphPopulation(Node<T>* targetNode, Node<T>* sourceNode, vector<Node<T>*>& sourceVisitedNodes, unordered_map<Node<T>*, Node<T>*>& nodesLookUpTable);
    Graph(T rootValue, bool directioned = false, bool weighted = false);
    Graph(const Graph& graph);
    Graph() {};
    
    void operator=(const Graph& graph);
    void eraseEdge(Node<T>* node, Edge<T>* edge);
    void recursiveDeletion(Node<T>* node, vector<Node<T>*>& deletedNodes, vector<Edge<T>*>& deletedEdges);
    void deletionWrapper();
    
    ~Graph();
    Node<T>* addNode(Node<T>* toNode, T value, int length = 0);
    void connectNodes(Node<T>* first, Node<T>* second, int lenght = 0);
    void deleteNode(Node<T>* node);
    bool contains(const T& value) const;
    bool empty() const;
    vector<Node<T>*> sortTopologically(Node<T>* node,vector<Node<T>*>& visitedNodes);
    vector<Node<T>*> kahnTopologicalSorting();
    void print() const;
    void printFromNode(Node<T>* node, list<Node<T>*>& visitedNodes) const;
    int** matrixRepresentation() const;
};

template<typename T>
Graph<T>::Graph(T rootValue, bool directioned, bool weighted) : directioned(directioned), weighted(weighted)
{
    root = new Node<T>(rootValue);
    nodes.push_back(root);
};

template<typename T>
void Graph<T>::recursiveGraphPopulation(Node<T>* targetNode, Node<T>* sourceNode, vector<Node<T>*>& sourceVisitedNodes, unordered_map<Node<T>*, Node<T>*>& nodesLookUpTable)
{
    targetNode->value = sourceNode->value;
    if(sourceNode->value == 4) {
        //        co\ut<<sourceNode->value<<endl;
    }
    
    sourceVisitedNodes.push_back(sourceNode);
    nodesLookUpTable.emplace(sourceNode, targetNode);
    for(Edge<T>* e : sourceNode->getAllEdges())
    {
        Edge<T>* newEdge = new Edge<T>;
        edges.push_back(newEdge);
        
        newEdge->weight = e->weight;
        
        if(sourceNode == e->toNode)
        {
            Node<T>* newNode = nullptr;
            if(nodesLookUpTable.find(e->fromNode) == nodesLookUpTable.end())
            {
                newNode = new Node<T>;
                nodes.push_back(newNode);
                newNode->value = e->fromNode->value;
                nodesLookUpTable.emplace(e->fromNode, newNode);
            }
            else
            {
                newNode = nodesLookUpTable[e->fromNode];
            }
            newEdge->fromNode = newNode;
            newEdge->toNode = targetNode;
            targetNode->inboundEdges.push_back(newEdge);
            if(find(sourceVisitedNodes.begin(), sourceVisitedNodes.end(), e->fromNode) == sourceVisitedNodes.end())
            {
                recursiveGraphPopulation(newNode, e->fromNode, sourceVisitedNodes, nodesLookUpTable);
            }
        }
        else
        {
            Node<T>* newNode = nullptr;
            if(nodesLookUpTable.find(e->toNode) == nodesLookUpTable.end())
            {
                newNode = new Node<T>;
                nodes.push_back(newNode);
                newNode->value = e->toNode->value;
                nodesLookUpTable.emplace(e->toNode, newNode);
            }
            else
            {
                newNode = nodesLookUpTable[e->toNode];
            }
            newEdge->fromNode = targetNode;
            newEdge->toNode = newNode;
            targetNode->outboundEdges.push_back(newEdge);
            if(find(sourceVisitedNodes.begin(), sourceVisitedNodes.end(), e->toNode) == sourceVisitedNodes.end())
            {
                recursiveGraphPopulation(newNode, e->toNode, sourceVisitedNodes, nodesLookUpTable);
            }
        }
    }
}

template<typename T>
void Graph<T>::operator=(const Graph& graph)
{
    if(this == &graph)
        return;
    
    deletionWrapper();
    
    this->directioned = graph.directioned;
    this->weighted = graph.weighted;
    root = new Node<T>;
    nodes.push_back(root);
    vector<Node<T>*> sourceVisitedNodes;
    unordered_map<Node<T>*, Node<T>*> nodesLookUpTable;
    recursiveGraphPopulation(root, graph.root, sourceVisitedNodes, nodesLookUpTable);
    
}

template<typename T>
Graph<T>::Graph(const Graph& graph)
{
    *this = graph;
}



template<typename T>
void Graph<T>::eraseEdge(Node<T>* node, Edge<T>* edge)
{
    vector<Edge<T>*>& edges = node == edge->toNode ? node->inboundEdges : node->outboundEdges;
    for(int i{0}; i < edges.size(); i++)
    {
        if(edges[i] == edge)
        {
            edges.erase(edges.begin() + i);
            break;
        }
    }
}

template<typename T>
void Graph<T>::recursiveDeletion(Node<T>* node, vector<Node<T>*>& deletedNodes, vector<Edge<T>*>& deletedEdges)
{
    
    deletedNodes.push_back(node);
    
    for(Edge<T>* e : node->getAllEdges())
    {
        if(e == nullptr || find(deletedEdges.begin(), deletedEdges.end(), e) != deletedEdges.end())
            continue;
        
        if(find(deletedNodes.begin(), deletedNodes.end(), e->toNode) == deletedNodes.end())
        {
            if(e->toNode != nullptr)
                recursiveDeletion(e->toNode, deletedNodes, deletedEdges);
        }
        //deleteEdge(e);
        if(find(deletedEdges.begin(), deletedEdges.end(), e) == deletedEdges.end())
            deletedEdges.push_back(e);
    }
    
    //deleteNodeInGraph(node);
}

template <typename T>
void Graph<T>::deletionWrapper() {
    vector<Node<T>*> deletedNodes;
    vector<Edge<T>*> deletedEdges;
    recursiveDeletion(nodes[0], deletedNodes, deletedEdges);
    for(Edge<T>* edge : deletedEdges)
    {
        delete edge;
    }
    
    for(Node<T>* node : deletedNodes)
    {
        delete node;
    }
    edges = vector<Edge<T>*>();
    nodes = vector<Node<T>*>();
}

template<typename T>
Graph<T>::~Graph()
{
    deletionWrapper();
}

template<typename T>
void Graph<T>::deleteNodeInGraph(Node<T>* node)
{
    for(int i{0}; i < nodes.size(); i++)
    {
        if(nodes[i] == node)
            nodes.erase(nodes.begin() + i);
    }
    
}

template<typename T>
void Graph<T>::deleteEdge(Edge<T>* edge)
{
    
    for(int i{0}; i < edges.size(); i++)
    {
        if(edges[i] == edge)
            edges.erase(edges.begin() + i);
    }
}


template<typename T>
Node<T>* Graph<T>::addNode(Node<T>* toNode, T value, int length)
{
    Node<T>* attachedNode = new Node<T>(value);
    if(toNode != nullptr)
    {
        Edge<T>* connectingEdge = new Edge<T>(toNode, attachedNode, length);
        attachedNode->inboundEdges.push_back(connectingEdge);
        toNode->outboundEdges.push_back(connectingEdge);
        edges.push_back(connectingEdge);
    }
    nodes.push_back(attachedNode);
    return attachedNode;
}

template<typename T>
void  Graph<T>::connectNodes(Node<T>* first, Node<T>* second, int lenght)
{
    if(first == nullptr || second == nullptr)
        throw runtime_error("Non-existing node!");
    
    Edge<T>* connectingEdge = new Edge<T>(first, second, lenght);
    edges.push_back(connectingEdge);
    
    first->outboundEdges.push_back(connectingEdge);
    second->inboundEdges.push_back(connectingEdge);
    
}

template<typename T>
void Graph<T>::deleteNode(Node<T>* node)
{
    for(Edge<T>* e: node->getAllEdges())
    {
        deleteEdge(e);
        delete e;
    }
    deleteNodeInGraph(node);
    delete node;
}

template<typename T>
Node<T>* nodeBreadthSearch(Node<T>* startingNode, const T& value)
{
    std::queue<Node<T>*> queue;
    queue.push(startingNode);
    Node<T>* result = nullptr;
    while(!queue.empty())
    {
        Node<T>* currentNode = queue.pop();
        if(currentNode->value == value)
            return currentNode;
        for(Edge<T>* edge: currentNode->outboundEdges)
        {
            result = nodeBreadthSearch(currentNode->outboundEdges->toNode, value);
            if(result != nullptr)
            {
                return result;
            }
        }
    }
    return result;
}

//Breadth-first traversal
template<typename T>
bool Graph<T>::contains(const T& value) const
{
    return (nodeBreadthSearch(nodes[0], value) != nullptr);
}

template<typename T>
bool Graph<T>::empty() const
{
    return nodes.empty();
}

template<typename T>
vector<Node<T>*> Graph<T>::sortTopologically(Node<T>* node, vector<Node<T>*>& visitedNodes)
{
    if(directioned)
    {
        Node<T>* currentNode = nodes[0];
        visitedNodes.push_back(currentNode);
        vector<Edge<T>*> currentEdges = currentNode->outboundEdges;
        for(int i{0}; i < currentEdges.size(); i++)
        {
            if(find(visitedNodes.begin(), visitedNodes().end(), currentEdges[i]) ==  visitedNodes.end())
            {
                return sortTopologically(currentEdges[i], visitedNodes);
            }
        }
        
    }
    else {
        return nodes;
    }
    return visitedNodes;
}

//    L ← Empty list that will contain the sorted elements
//    S ← Set of all nodes with no incoming edge
//
//    while S is not empty do
//        remove a node n from S
//        add n to tail of L
//        for each node m with an edge e from n to m do
//            remove edge e from the graph
//            if m has no other incoming edges then
//                insert m into S
//
//    if graph has edges then
//        return error   (graph has at least one cycle)
//    else
//        return L   (a topologically sorted order)

template<typename T>
Edge<T>* findOutgoingEdgeFromTo(Node<T>* fromNode, Node<T>* toNode)
{
    for(Edge<T>* edge: fromNode->outboundEdges())
    {
        if(edge->toNode == toNode)
            return edge;
    }
    return nullptr;
}

template<typename T>
vector<Node<T>*> Graph<T>::kahnTopologicalSorting()
{
    vector<Node<T>*> sortedNodes;
    vector<Node<T>*> sources;
    for(int i{0}; i < nodes.size(); i++)
    {
        if(nodes[i]->incomingEdges.empty())
            sources.push_back(nodes[i]);
    }
    
    while(!sources.empty())
    {
        Node<T>* currentNode = sources.back();
        sortedNodes.push_back(sources.back());
        sources.pop_back();
        for(Node<T>* node: nodes)
        {
            Edge<T>* edge = findOutgoingEdgeFromTo(currentNode, node);
            if(edge != nullptr)
            {
                delete edge;
            }
            sources.push_back(node);
        }
    }
    if(!this->edges.empty())
        throw runtime_error("Impossible sorting, graph has at least one cycle!");
    
    else
        return sortedNodes;
}

template<typename T>
void Graph<T>::printFromNode(Node<T>* node, list<Node<T>*>& visitedNodes) const
{
    
    cout<<"[";
    cout<<"("<<node->value<<")";
    if(visitedNodes.size() == nodes.size())
    {
        cout<<"]";
        return;
    }
    
    string edge = directioned ? DIRECTIONED_EDGE : UNDIRECTIONED_EDGE;
    vector<Edge<T>*> edges = directioned ? node->outboundEdges : node->getAllEdges();
    
    cout<< edge ;
    for(Edge<T>* e : edges)
    {
        if(e->toNode != nullptr) {
            if(node != e->toNode)
                cout<<"("<<e->toNode->value<<")";
            else
                cout<<"("<<e->fromNode->value<<")";
        }
    }
    cout<<"] ";
    visitedNodes.push_back(node);
    for(Edge<T>* e : edges)
    {
        if(find(visitedNodes.begin(), visitedNodes.end(), e-> toNode) == visitedNodes.end())
        {
            printFromNode(e-> toNode, visitedNodes);
        }
    }
}


template<typename T>
void Graph<T>::print() const
{
    list<Node<T>*> visitedNodes;
    printFromNode(nodes[0], visitedNodes);
}

template<typename T>
int** Graph<T>::matrixRepresentation() const
{
    int** adjacencyMatrix = new int*[nodes.size()];
    for(int i{0}; i < nodes.size(); i++)
    {
        adjacencyMatrix[i] = new int[nodes.size()];
    }
    
    for(int row{0}; row < nodes.size(); row++)
    {
        for(int column{0}; column < nodes.size(); column++)
        {
            if(directioned)
            {
                if(findOutgoingEdgeFromTo(nodes[row], nodes[column]))
                    adjacencyMatrix[row][column] = 1;
                else
                   adjacencyMatrix[row][column] = 0;
            }
            else
            {
                vector<Edge<T>*> currentNodeEdges = nodes[row]->getAllEdges();
                if(find(currentNodeEdges.begin(), currentNodeEdges.end(), nodes[column]) != currentNodeEdges.end)
                    adjacencyMatrix[row][column] = 1;
                else
                   adjacencyMatrix[row][column] = 0;
            }
        }
    }
    return adjacencyMatrix;
}


#endif /* Graph_h */

//print [(1)->(2)(3)(4)], [(2)->(5)]
