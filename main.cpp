#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <queue>

using std::ifstream;
using std::ofstream;
using std::vector;
using std::priority_queue;
using std::pair;
using std::make_pair;

#define INF 999999

class Node{
    private:
        unsigned int data;
        unsigned int parent;
        unsigned int distance;

    public:
        Node();

        Node(unsigned int dataC, unsigned int parentC, unsigned int distanceC): data{dataC}, parent{parentC}, distance{distanceC}{}

        unsigned int getData() const {return this->data;}
        unsigned int getParent() const {return this->parent;}
        unsigned int getDistance() const {return this->distance;}

        void setParent(unsigned int otherParent) {this->parent = otherParent;}
        void setDistance(unsigned int otherDistance) {this->distance = otherDistance;}
};

class Edge{
    friend class Node;

    private:
        unsigned int extr1, extr2, weight;

    public:
        Edge();

        Edge(unsigned int extr1C, unsigned int extr2C, unsigned int weightC): extr1{extr1C}, extr2{extr2C}, weight{weightC}{}

        unsigned int getExtr1() const {return this->extr1;}
        unsigned int getExtr2() const {return this->extr2;}
        unsigned int getWeight() const {return this->weight;}
};

void RELAX(Node& sourceNode, Node& neighborNode, unsigned int weight, priority_queue<pair<unsigned int, unsigned int>, vector<pair<unsigned int, unsigned int>>, std::greater<>>& pq) {
    if(neighborNode.getDistance() > sourceNode.getDistance() + weight) {
        neighborNode.setDistance(sourceNode.getDistance() + weight);
        neighborNode.setParent(sourceNode.getData());
        pq.emplace(neighborNode.getDistance(), neighborNode.getData());
    }
}

void DIJKSTRA(unsigned int S, vector<Node>& nodes, vector<Edge>& edges) {
    vector<pair<unsigned int, unsigned int>> parcurse;
    priority_queue<pair<unsigned int, unsigned int>, vector<pair<unsigned int, unsigned int>>, std::greater<>> pq;
    pq.emplace(nodes[S].getDistance(), nodes[S].getData());

    while(!pq.empty()) {
        pair<unsigned int, unsigned int> current = pq.top();
        pq.pop();

        parcurse.push_back(current);
        for(auto& edge: edges)
            if(edge.getExtr1() == current.second)
                RELAX(nodes[edge.getExtr1()], nodes[edge.getExtr2()], edge.getWeight(), pq);
    }
}


int main(int argc, char** argv){
    std::ifstream fin(argv[1]);
    std::ofstream fout(argv[2]);

    unsigned int V, E, S;

    fin >> V >> E >> S;

    vector<Node> nodes;
    vector<Edge> edges;

    for(unsigned int i = 0; i < V; i++) {
        if(i != S)
            nodes.emplace_back(i, INF, INF);
        else
            nodes.emplace_back(S, 0, 0);
    }

    unsigned int extr1, extr2, w;

    while(fin >> extr1 >> extr2 >> w)
        edges.emplace_back(extr1, extr2, w);


    DIJKSTRA(S, nodes, edges);

    for(auto& node: nodes)
        if(node.getDistance() != INF)
            fout << node.getDistance() << " ";
        else
            fout << "INF" << " ";

    fin.close();
    fout.close();
    return 0;
}
