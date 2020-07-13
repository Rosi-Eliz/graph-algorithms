//
//  main.cpp
//  Graph
//
//  Created by Rosi-Eliz Dzhurkova on 02.07.20.
//  Copyright © 2020 Rosi-Eliz Dzhurkova. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "GraphAlgorithms.h"

using namespace std;

int main()
{
//    Graph<int> graph(1);
//    Node<int>* node2 = graph.addNode(graph.root, 2);
//    Node<int>* node4 = graph.addNode(graph.root, 4);
//    Node<int>* node5 = graph.addNode(node2, 5);
//    Node<int>* node6 = graph.addNode(node2, 6);
//    Node<int>* node3 = graph.addNode(node4, 3);
//    graph.connectNodes(node3, node2);
//    graph.print();
//
//    Graph<int> copyGraph(2);
//    copyGraph = graph;
//    cout<<endl;
//    copyGraph.print();
    
//    GraphAlgorithms<int> dGraph(1);
//    Node<int>* node2 = dGraph.addNode(dGraph.root, 2, 1);
//    Node<int>* node3 = dGraph.addNode(dGraph.root, 3, 5);
//    dGraph.connectNodes(node2, node3, 1);
//    Node<int>* node4 = dGraph.addNode(node2, 4, 6);
//    dGraph.connectNodes(node2, node4, 6);
//    dGraph.connectNodes(node4, node3, 3);
//    dGraph.findShortestPathBetween(dGraph.root, node4);
    
    GraphAlgorithms<int> dGraph(1);
    vector<int> degrees = {2,2,1,1};
    vector<int> values = {1,2,3,4};
    GraphAlgorithms<int> dGraph2 = GraphAlgorithms<int>::constructGraph(degrees, values);
    dGraph2.print();
    return 0;
}

